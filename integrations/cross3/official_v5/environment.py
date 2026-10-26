"""Cross3 CARLA environment shared verbatim by all three signal controllers."""
import hashlib,json,math,random,sys,time
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
import run_cross3 as harness
import carla
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.local_planner import RoadOption
from control_four_phases import expected_states,PHASES
from schema import PHASE_BITS,PHASE_NAMES,position_bin
from learning import IntervalReturn

DT=.05
assert tuple(PHASES)==PHASE_NAMES

def demand(seed,kind,horizon=120):
    rng=random.Random(seed);arrivals=[]
    for road in (1,2,3,4):
        for lane in (-1,-2,-3):
            rate=.035
            if kind=='east_west':rate*=2 if road in (1,3) else .5
            if kind=='north_south':rate*=2 if road in (2,4) else .5
            if kind=='left_heavy':rate*=2 if lane==-1 else .65
            t=rng.uniform(0,5)
            while t<horizon:
                arrivals.append(dict(time=round(t,3),road=road,lane=lane))
                t+=rng.expovariate(rate)
    return sorted(arrivals,key=lambda r:(r['time'],r['road'],r['lane']))

class Environment:
    def __init__(self,port,output):
        self.client=carla.Client('127.0.0.1',port);self.client.set_timeout(120)
        self.world,self.design,self.lights=harness.load_map(self.client,output)
        self.map=self.world.get_map();self.moves=self.design['movements']
        self.keys=[(m['road'],m['lane']) for m in self.moves]
        self.outkeys=[(r,l) for r in (1,2,3,4) for l in (1,2,3)]
        self.route_cache={k:harness.route_for(self.map,m,60) for k,m in zip(self.keys,self.moves)}
        self.phase_indices=[[i for i,m in enumerate(self.moves) if m['road'] in roads and m['lane']==lane]
            for roads,lane in (((2,4),-2),((2,4),-1),((1,3),-2),((1,3),-1))]
        self.lane_lights={(c['road_id'],c['incoming_lanes'][0]):self.lights[sid] for sid,c in self.design['signals'].items()}
        self.stop_lines={c['road_id']:c['reference_s'] for c in self.design['signals'].values()}
        # Check the executable lights against the feature/action order, rather
        # than relying on identical-looking hard-coded phase names.
        for phase,indices in enumerate(self.phase_indices):
            actual=expected_states(self.design,PHASES[phase],'GREEN','always-green')
            controlled={(c['road_id'],c['incoming_lanes'][0]) for sid,c in self.design['signals'].items()
                        if actual[sid]=='Green' and c['movement']!='right'}
            assert controlled=={self.keys[i] for i in indices},(phase,controlled)
        parent=self
        class Driver(BasicAgent):
            def _vehicle_obstacle_detected(self,vehicle_list=None,max_distance=None):
                # CARLA 0.9.12's angle test excludes exactly collinear targets
                # (strict 0 < angle); queued cars can therefore be missed.
                own=self._vehicle.get_transform();wp=self._map.get_waypoint(own.location)
                forward=own.get_forward_vector()
                for other in vehicle_list or []:
                    if other.id==self._vehicle.id:continue
                    t=other.get_transform();other_wp=self._map.get_waypoint(t.location)
                    if (wp.road_id,wp.lane_id)!=(other_wp.road_id,other_wp.lane_id):continue
                    delta=t.location-own.location
                    along=delta.x*forward.x+delta.y*forward.y
                    lateral=abs(delta.x*forward.y-delta.y*forward.x)
                    gap=along-self._vehicle.bounding_box.extent.x-other.bounding_box.extent.x
                    if along>0 and lateral<1.5 and gap<(max_distance or 5.):return True,other
                return super()._vehicle_obstacle_detected(vehicle_list,max_distance)
            def _affected_by_traffic_light(self,lights_list=None,max_distance=None):
                wp=self._map.get_waypoint(self._vehicle.get_location());l=parent.lane_lights.get((wp.road_id,wp.lane_id))
                if self._last_traffic_light!=l:self._last_traffic_light=None
                return super()._affected_by_traffic_light([l],max_distance) if l else (False,None)
        self.Driver=Driver

    def observe(self,fleet,phase):
        coming=[0]*36;leaving=[0]*36;inq=[0]*12;outq=[0]*12;positions=[];queued=0
        for item in fleet:
            actor=item['actor'];loc=actor.get_location();wp=self.map.get_waypoint(loc)
            v=actor.get_velocity();speed=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
            key=(wp.road_id,wp.lane_id) if wp else None
            if key in self.keys:
                i=self.keys.index(key);b=position_bin(wp.s)
                if b is not None:coming[i*3+b]+=1
                inq[i]+=int(speed<.1)
            if key in self.outkeys:
                i=self.outkeys.index(key);b=position_bin(150-wp.s)
                if b is not None:leaving[i*3+b]+=1
                outq[i]+=int(speed<.1)
            queued+=int(speed<.1)
            positions.append((item,wp,speed))
        pressure=[]
        for indices in self.phase_indices:
            pressure.append(sum(inq[i]-outq[self.outkeys.index((self.moves[i]['target_road'],self.moves[i]['target_lane']))] for i in indices))
        return dict(cur_phase=PHASE_BITS[phase][:],coming_vehicle=coming,leaving_vehicle=leaving), -.25*abs(sum(inq)-sum(outq)),pressure,queued,positions

    def clearance_blockers(self,positions):
        """Include approach cars beyond the stop line and exiting vehicle tails."""
        blockers=[]
        for item,wp,_ in positions:
            if item['move']['lane']==-3 or wp is None:continue
            # Incoming s increases toward 150; outgoing s decreases from 150.
            # Six metres beyond the junction conservatively covers the rear of
            # our Model 3; also honour larger vehicle extents if configured later.
            boundary=min(self.stop_lines.get(wp.road_id,144),
                         150-item['actor'].bounding_box.extent.x-1.)
            if wp.is_junction or wp.s>boundary:blockers.append(item['actor'].id)
        return blockers

    def run(self,folder,arrivals,select,seed,limit=300,initial_phase=0):
        assert 0 <= initial_phase < 4
        folder.mkdir(parents=True,exist_ok=False);(folder/'demand.json').write_text(json.dumps(arrivals))
        world=self.world;original=world.get_settings();saved=[(l,l.get_state(),l.is_frozen()) for l in self.lights.values()]
        fleet=[];all_vehicles=[];collisions=[];samples=[];elapsed=0.;active=initial_phase;target=initial_phase;previous=None;pending=None
        desired_prev=None;next_decision=0.;switch_at=-100.;switching=False;remaining=list(arrivals);waiting=[]
        extended_clearance=0.;transitions=[];last_stage=None;signal_diagnostics=[]
        queue_integral=0.;spawn_wait=0.;mismatches=0;lags=0;trace=[];failure=None
        summary={'seed':seed,'planned':len(arrivals),'completed':0,'collisions':collisions}
        try:
            settings=world.get_settings();settings.synchronous_mode=True;settings.fixed_delta_seconds=DT;settings.no_rendering_mode=True;world.apply_settings(settings)
            # Settle the initial phase BEFORE starting demand/time accounting.
            # Switching asynchronous/synchronous mode between episodes can leave
            # more than one stale native-light snapshot in CARLA 0.9.12.
            initial=expected_states(self.design,PHASES[initial_phase],'GREEN','always-green')
            for sid,l in self.lights.items():
                l.freeze(True);l.set_state(getattr(carla.TrafficLightState,initial[sid]))
            self.client.apply_batch_sync([],False)
            stable=0
            for frame in range(20):
                world.tick()
                actual={sid:str(l.get_state()).split('.')[-1] for sid,l in self.lights.items()}
                frozen=all(l.is_frozen() for l in self.lights.values())
                stable=stable+1 if actual==initial and frozen else 0
                signal_diagnostics.append(dict(stage='initialization',frame=frame,actual=actual,
                                               expected=initial,frozen=frozen))
                if stable>=3:break
            else:raise RuntimeError('Initial signal synchronization failed')
            summary['initial_signal_settle_frames']=frame+1
            desired_prev=initial
            start=world.get_snapshot().timestamp.elapsed_seconds
            while elapsed<limit:
                while remaining and remaining[0]['time']<=elapsed:waiting.append(remaining.pop(0))
                for arrival in list(waiting):
                    key=(arrival['road'],arrival['lane']);route=self.route_cache[key];loc=route[0].transform.location
                    if any(item['actor'].get_location().distance(loc)<9 for item in fleet):continue
                    transform=carla.Transform(carla.Location(loc.x,loc.y,loc.z+.4),route[0].transform.rotation)
                    bp=world.get_blueprint_library().find('vehicle.tesla.model3');bp.set_attribute('role_name','official_benchmark')
                    actor=world.try_spawn_actor(bp,transform)
                    if actor is None:continue
                    sensor=world.spawn_actor(world.get_blueprint_library().find('sensor.other.collision'),carla.Transform(),attach_to=actor)
                    sensor.listen(lambda e:collisions.append(dict(frame=e.frame,actor=e.actor.id,other=e.other_actor.id)))
                    driver=self.Driver(actor,target_speed=20,opt_dict={});driver.set_global_plan([(w,RoadOption.LANEFOLLOW) for w in route])
                    item=dict(actor=actor,sensor=sensor,driver=driver,move=self.moves[self.keys.index(key)],arrival=arrival['time'],spawn=elapsed,waiting=0.,completed=False)
                    fleet.append(item);all_vehicles.append(item);waiting.remove(arrival)
                state,reward,pressures,queued,positions=self.observe(fleet,active)
                if not switching and elapsed+1e-5>=next_decision:
                    if pending is not None:
                        samples.append(pending['return'].sample(pending['state'],pending['action'],state,pending['time'],False,False))
                    action=int(select(state,pressures,active,elapsed));assert 0<=action<4
                    previous=active;target=action;switch_at=elapsed if action!=active else -100.
                    switching=action!=active
                    if action==active:active=action
                    pending=dict(state=state,action=action,time=elapsed,**{'return':IntervalReturn()})
                    next_decision=elapsed+10.
                    trace.append(dict(time=elapsed,phase=active,action=action,pressure=pressures,queue=queued,external_queue=len(waiting)))
                age=elapsed-switch_at
                # A 2-second clearance alone is insufficient for CARLA turning
                # trajectories. Hold red until all controlled movement vehicles
                # (including their rears) leave the junction, then give 5 s green.
                blockers=self.clearance_blockers(positions)
                occupied=bool(blockers)
                if switching and age<3:phase=previous;stage='YELLOW'
                elif switching and (age<5 or occupied):
                    phase=None;stage='RED'
                    if age>=5:extended_clearance+=DT
                else:
                    if switching:
                        assert not occupied and age>=5,'Unsafe phase release'
                        next_decision=elapsed+5.
                    switching=False;active=target;phase=active;stage='GREEN'
                if (phase,stage)!=last_stage:
                    transitions.append(dict(time=elapsed,phase=phase,stage=stage,target=target,
                                            blockers=blockers,switch_age=age))
                    last_stage=(phase,stage)
                desired=expected_states(self.design,PHASES[phase] if phase is not None else None,stage,'always-green')
                changed=desired!=desired_prev
                if changed:
                    for sid,color in desired.items():self.lights[sid].set_state(getattr(carla.TrafficLightState,color))
                    self.client.apply_batch_sync([],False)
                    world.get_settings()
                for item in fleet:item['actor'].apply_control(item['driver'].run_step())
                world.tick();snapshot=world.get_snapshot();elapsed=snapshot.timestamp.elapsed_seconds-start
                actual={sid:str(l.get_state()).split('.')[-1] for sid,l in self.lights.items()}
                if actual!=desired:
                    signal_diagnostics.append(dict(stage=stage,time=elapsed,actual=actual,expected=desired,
                                                   changed=changed,frame=snapshot.frame))
                    if changed:lags+=1
                    else:mismatches+=1;raise RuntimeError('Stable signal mismatch')
                desired_prev=desired
                queue_integral+=queued*DT;spawn_wait+=len(waiting)*DT
                pending['return'].add(reward,DT)
                for item,wp,speed in positions:
                    item['waiting']+=DT*int(speed<.1)
                    if wp and wp.road_id in (1,2,3,4) and wp.lane_id>0:
                        m=item['move']
                        if (wp.road_id,wp.lane_id)!=(m['target_road'],m['target_lane']):raise RuntimeError('Wrong exit lane')
                        if wp.s<=88:
                            item['completed']=True;item['finish']=elapsed
                            item['sensor'].stop();item['sensor'].destroy();item['actor'].destroy();fleet.remove(item)
                if collisions:
                    involved={e[k] for e in collisions for k in ('actor','other')}
                    summary['collision_context']=[]
                    for item in fleet:
                        if item['actor'].id not in involved:continue
                        a=item['actor'];t=a.get_transform();wp=self.map.get_waypoint(t.location)
                        summary['collision_context'].append(dict(actor=a.id,move=item['move'],x=t.location.x,y=t.location.y,
                            yaw=t.rotation.yaw,road=wp.road_id,lane=wp.lane_id,s=wp.s,spawn=item['spawn'],phase=phase,stage=stage,time=elapsed))
                    raise RuntimeError('Collision callback')
                if not remaining and not waiting and not fleet:break
            # Empty demand/fleet is a true terminal; time limit is a truncation (bootstrap).
            if pending and pending['return'].duration:
                state,reward,_,_,_=self.observe(fleet,active)
                terminal=not remaining and not waiting and not fleet
                samples.append(pending['return'].sample(pending['state'],pending['action'],state,pending['time'],terminal,not terminal))
            completed=[v for v in all_vehicles if v['completed']]
            summary.update(status='finished',completed=len(completed),elapsed=elapsed,
                waiting_vehicle_seconds=queue_integral+spawn_wait,mean_wait_seconds=(queue_integral+spawn_wait)/len(arrivals),
                mean_travel_seconds=sum(v.get('finish',elapsed)-v['arrival'] for v in all_vehicles)/max(1,len(all_vehicles)),
                unspawned=len(waiting)+len(remaining),signal_mismatches=mismatches,signal_lag_frames=lags,
                extended_clearance_seconds=extended_clearance)
        except BaseException as exc:
            summary.update(status='failed',error=repr(exc),elapsed=elapsed,
                           completed=sum(v['completed'] for v in all_vehicles),
                           signal_mismatches=mismatches,signal_lag_frames=lags);failure=exc
        finally:
            for item in fleet:
                if item['sensor'].is_alive:item['sensor'].stop();item['sensor'].destroy()
                if item['actor'].is_alive:item['actor'].destroy()
            for light,color,frozen in saved:light.set_state(color);light.freeze(frozen)
            world.tick();world.apply_settings(original)
            (folder/'vehicles.json').write_text(json.dumps([{k:v for k,v in item.items()
                if k not in ('actor','sensor','driver')} for item in all_vehicles],indent=2))
            (folder/'summary.json').write_text(json.dumps(summary,indent=2));(folder/'decisions.json').write_text(json.dumps(trace))
            (folder/'samples.json').write_text(json.dumps(samples))
            (folder/'transitions.json').write_text(json.dumps(transitions,indent=2))
            (folder/'signal_diagnostics.json').write_text(json.dumps(signal_diagnostics,indent=2))
        if failure:raise failure
        return summary,samples
