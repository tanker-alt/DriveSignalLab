"""One-owner CARLA rollout with continuous arrivals and internal VLA head PPO."""
import json
import math
import os
import sys
from pathlib import Path
import torch
from .core import Metrics, Signals, observation
from .safety import SafetyMetrics
from .measurements import Recorder

class CarlaEnv:
    dt=.05
    def __init__(self, folder, rows, seed, limit, policy, joint=False, train=False,
                 port=31610, waypoint_head=None, fairness_weight=0., max_wait_weight=0., **kwargs):
        import fcntl
        # Hold a process lock before touching the world. Existing active actors are
        # additionally rejected by load_map, including experiments without this lock.
        self.lock=open('/tmp/cross3-transfer-%d.lock'%port,'w')
        fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        folder.mkdir(parents=True,exist_ok=False)
        official=Path(__file__).resolve().parents[1]/'official'
        sys.path.insert(0,str(official))
        from environment import Environment
        import carla
        from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
        from srunner.scenariomanager.timer import GameTime
        self.carla=carla; self.provider=CarlaDataProvider; self.game_time=GameTime
        self.base=Environment(port,folder)
        base=self.base
        class LaneSignalDriver(base.Driver):
            def _affected_by_traffic_light(driver,lights_list=None,max_distance=None):
                actor=driver._vehicle; wp=base.map.get_waypoint(actor.get_location())
                light=base.lane_lights.get((wp.road_id,wp.lane_id)) if wp else None
                if light is None: return False,None
                color=str(light.get_state()).split('.')[-1]
                if color not in ('Red','Yellow'): return False,None
                velocity=actor.get_velocity(); speed=(velocity.x**2+velocity.y**2)**.5
                distance=base.stop_lines[wp.road_id]-wp.s-actor.bounding_box.extent.x
                braking=speed*speed/6.+speed*.5+3.
                return (True,light) if 0<distance<=max(braking,max_distance or 0) else (False,None)
        self.Driver=LaneSignalDriver
        self.world=self.base.world; self.client=self.base.client
        self.folder=folder; self.rows=rows; self.limit=limit; self.policy=policy
        self.joint=joint; self.train=train; self.fairness_weight=fairness_weight; self.max_wait_weight=max_wait_weight; self.time=0.; self.signals=Signals(); self.metrics=Metrics(rows); self.metrics.safety=SafetyMetrics()
        self.fleet={}; self.completed=set(); self.events=[]; self.collisions=[]; self.seen_collisions=set()
        self.agent=self.wrapper=self.hook=None
        self.ego_ids=[r['id'] for r in rows if r['ego']]
        self.saved_settings=self.world.get_settings()
        self.saved_lights=[(l,l.get_state(),l.is_frozen()) for l in self.base.lights.values()]
        self.recorder=Recorder(folder,'carla')
        self.positions=(folder/'positions.jsonl').open('w')
        self.trace=(folder/'decisions.jsonl').open('w'); self.head_trace=(folder/'head_actions.jsonl').open('w')
        self.closed=False
        try:
            settings=self.world.get_settings(); settings.synchronous_mode=True
            settings.fixed_delta_seconds=self.dt; settings.no_rendering_mode=not joint
            self.world.apply_settings(settings)
            self.provider.set_client(self.client); self.provider.set_world(self.world); GameTime.restart()
            self.apply_signals()
            for _ in range(3): self.world.tick()
            if joint:
                if len(self.ego_ids)!=2: raise ValueError('Joint mode requires two VLA egos')
                for row in rows:
                    if row['ego']: self.spawn(row)
                self.setup_vla(waypoint_head)
            self.origin=self.world.get_snapshot().timestamp.elapsed_seconds
        except BaseException:
            self.close(); raise
    def spawn(self,row):
        from agents.navigation.local_planner import RoadOption
        c=self.carla; route=self.base.route_cache[(row['road'],row['lane'])]
        loc=route[0].transform.location
        snapshot=self.world.get_snapshot()
        if any((snapshot.find(v['actor'].id).get_transform().location
                if snapshot.find(v['actor'].id) is not None else v['spawn_location']).distance(loc)<9
               for v in self.fleet.values()): return False
        bp=self.world.get_blueprint_library().find('vehicle.lincoln.mkz_2017' if row['ego'] else 'vehicle.tesla.model3')
        bp.set_attribute('role_name','hero_%d'%self.ego_ids.index(row['id']) if row['ego'] else 'transfer_background')
        actor=self.world.try_spawn_actor(bp,c.Transform(c.Location(loc.x,loc.y,loc.z+.4),route[0].transform.rotation))
        if actor is None: return False
        sensor=self.world.spawn_actor(self.world.get_blueprint_library().find('sensor.other.collision'),c.Transform(),attach_to=actor)
        sensor.listen(lambda e:self.collisions.append((e.actor.id,e.other_actor.id)))
        driver=None
        if row['ego']:
            self.provider._carla_actor_pool[actor.id]=actor; self.provider.register_actor(actor)
            actor.apply_control(c.VehicleControl(brake=1))
        else:
            driver=self.Driver(actor,target_speed=20,opt_dict={})
            driver.set_global_plan([(w,RoadOption.LANEFOLLOW) for w in route])
        self.fleet[row['id']]=dict(actor=actor,sensor=sensor,driver=driver,row=row,spawn_location=loc)
        self.metrics.spawned(row['id'],self.time)
        return True
    def setup_vla(self,waypoint_head):
        from environment import harness
        from leaderboard.autoagents.agent_wrapper import AgentWrapper
        from leaderboard.utils.route_manipulation import location_route_to_gps,_get_latlon_ref
        from agents.navigation.local_planner import RoadOption
        os.environ.update(HF_HUB_OFFLINE='1',TRANSFORMERS_OFFLINE='1',LMDRIVE_SKIP_BACKBONE_PRETRAIN='1',
                          SAVE_PATH=str(self.folder/'lmdrive'),LMDRIVE_LIGHT_MESSAGES='off',ROUTES='transfer_joint')
        if waypoint_head: os.environ['LMDRIVE_WAYPOINT_HEAD']=str(Path(waypoint_head).resolve())
        else: os.environ.pop('LMDRIVE_WAYPOINT_HEAD',None)
        from team_code.fixed_signal_lmdrive import FixedSignalLMDrive
        # Some upstream model asset paths are relative to the model repository.
        previous=os.getcwd()
        try:
            os.chdir(str(harness.MDRIVE))
            self.agent=FixedSignalLMDrive(str(harness.DRIVE/'configs/lmdrive_four_arm_config.py'),2)
        finally: os.chdir(previous)
        self.agent.net.eval(); self.agent.net.requires_grad_(False)
        layer=self.agent.net.waypoints_predictor[-1]
        self.policy.bind_head(layer)
        # Actual inference runs without graphs; PPO later differentiates this very
        # layer using saved input activations. Updated weights are copied back here.
        def sample_head(module,inputs,output):
            features=inputs[0].detach().reshape(-1,module.in_features)[-1].cpu().float()
            with torch.no_grad():
                mean=self.policy.head(features)
                dist=torch.distributions.Normal(mean,self.policy.std)
                action=dist.sample() if self.train else mean
            result=output.clone(); result.reshape(-1,10)[-1]=action.to(output)
            if self.train:
                self.events.append(dict(features=features,action=action,mean=mean,logp=float(dist.log_prob(action).sum()),
                                        ego=self.agent.active_ego,time=self.time,camera_frame=self.agent.active_camera_frame))
            self.head_trace.write(json.dumps(dict(time=self.time,ego=self.agent.active_ego,
                internal_layer='waypoints_predictor.2',mean=mean.tolist(),action=action.tolist()))+'\n')
            return result
        self.hook=layer.register_forward_hook(sample_head)
        routes=[[(w.transform,RoadOption.LANEFOLLOW) for w in self.base.route_cache[
            (self.fleet[vid]['row']['road'],self.fleet[vid]['row']['lane'])]] for vid in self.ego_ids]
        lat,lon=_get_latlon_ref(self.world)
        self.agent.town_id='OpenDriveMap'
        self.agent.set_global_plan([location_route_to_gps(r,lat,lon) for r in routes],routes)
        self.wrapper=AgentWrapper(self.agent)
        for i,vid in enumerate(self.ego_ids): self.wrapper.setup_sensors(self.fleet[vid]['actor'],i,self.folder)
    def vehicles(self):
        vs={}; locations={}
        snapshot=self.world.get_snapshot()
        for vid,item in self.fleet.items():
            a=item['actor']; physical=snapshot.find(a.id)
            # A newly spawned actor has no physics snapshot until the next tick.
            # Actor RPC getters can return a default origin in this interval.
            # Do not turn that placeholder into a measured position/TTC.
            if physical is None: continue
            transform=physical.get_transform()
            wp=self.base.map.get_waypoint(transform.location); vel=physical.get_velocity()
            row=item['row']; speed=(vel.x**2+vel.y**2+vel.z**2)**.5
            half=a.bounding_box.extent.x
            yaw=math.radians(transform.rotation.yaw); loc=transform.transform(a.bounding_box.location)
            # Project the oriented front onto the lane tangent at the stop line.
            stop_wp=self.base.map.get_waypoint_xodr(row['road'],row['lane'],144.)
            tangent=stop_wp.transform.get_forward_vector(); delta=loc-stop_wp.transform.location
            projection=half*abs(math.cos(yaw)*tangent.x+math.sin(yaw)*tangent.y)+a.bounding_box.extent.y*abs(-math.sin(yaw)*tangent.x+math.cos(yaw)*tangent.y)
            front_distance=-(delta.x*tangent.x+delta.y*tangent.y)-projection
            approach=bool(wp and (wp.road_id,wp.lane_id)==(row['road'],row['lane']))
            outgoing=bool(wp and (wp.road_id,wp.lane_id)==(row['target_road'],row['target_lane']))
            boundary=min(self.base.stop_lines.get(wp.road_id,144),150-a.bounding_box.extent.x-1) if wp else 144
            vs[vid]=dict(road=row['road'],lane=row['lane'],speed=speed,approach=approach,outgoing=outgoing,
                distance=front_distance if approach else 0.,gap=None,leader_speed=speed,leader_id=None,
                front_stop_distance=front_distance,exit_front_distance=wp.s-half-88 if outgoing else None,
                x=loc.x,y=loc.y,yaw=yaw,vx=vel.x,vy=vel.y,vz=vel.z,half_length=half,half_width=a.bounding_box.extent.y,
                lane_key='%d/%d'%(wp.road_id,wp.lane_id) if wp else None,raw_position=wp.s if wp else None,
                raw_edge=wp.road_id if wp else None,raw_lane=wp.lane_id if wp else None,spawn_time=self.metrics.rows[vid]['spawn'],
                stop_x=stop_wp.transform.location.x,stop_y=stop_wp.transform.location.y,stop_dx=tangent.x,stop_dy=tangent.y,
                occupied=bool(wp and (wp.is_junction or (approach and front_distance<=0) or (outgoing and wp.s>150-half-1))),wp=wp)
            locations[vid]=(transform,a.bounding_box.extent.x)
        for vid,v in vs.items():
            wp=v['wp']; transform,length=locations[vid]; forward=transform.get_forward_vector()
            for oid,other in vs.items():
                owp=other['wp']
                if oid==vid or not wp or not owp or (wp.road_id,wp.lane_id)!=(owp.road_id,owp.lane_id): continue
                delta=locations[oid][0].location-transform.location
                along=(owp.s-wp.s)*(1 if wp.lane_id<0 else -1)
                if along>0:
                    gap=max(0.,along-length-locations[oid][1])
                    if v['gap'] is None or gap<v['gap']: v.update(gap=gap,leader_speed=other['speed'],leader_id=oid)
        return vs
    def observe(self):
        vs=self.vehicles()
        for row in self.rows:
            if row['time']<=self.time and row['id'] not in vs and row['id'] not in self.completed:
                vs[row['id']]=dict(road=row['road'],lane=row['lane'],speed=0.,approach=True,distance=90.)
        return observation(vs,self.signals,self.ego_ids)
    def apply_signals(self):
        from control_four_phases import expected_states,PHASES
        desired=expected_states(self.base.design,PHASES[self.signals.active],self.signals.stage,'with-straight')
        for sid,state in desired.items():
            light=self.base.lights[sid]; light.freeze(True)
            light.set_state(getattr(self.carla.TrafficLightState,state))
        self.client.apply_batch_sync([],False)
        return desired
    def retire(self,vid):
        item=self.fleet.pop(vid); item['sensor'].stop(); item['sensor'].destroy()
        if vid in self.ego_ids and self.wrapper:
            i=self.ego_ids.index(vid); self.wrapper.cleanup_single(i); self.wrapper.del_ego_sensor(i)
            self.provider._carla_actor_pool.pop(item['actor'].id,None)
        item['actor'].destroy(); self.completed.add(vid)
    def step(self,action):
        obs=self.observe(); self.signals.request(action,obs['incoming'])
        before=self.metrics.vehicle_time; self.events=[]; duration=0.
        for _ in range(round(1/self.dt)):
            for row in self.rows:
                if row['time']<=self.time and row['id'] not in self.fleet and row['id'] not in self.completed:
                    if not row['ego']: self.spawn(row)
            vs=self.vehicles(); desired=self.apply_signals()
            for vid,item in self.fleet.items():
                if item['driver'] and vid in vs: item['actor'].apply_control(item['driver'].run_step())
            snap=self.world.get_snapshot(); self.game_time.on_carla_tick(snap.timestamp); self.provider.on_carla_tick()
            if self.wrapper and any(v in self.fleet for v in self.ego_ids):
                from agent_interfaces import validate_controls
                controls=validate_controls(self.wrapper(),2)
                for i,vid in enumerate(self.ego_ids):
                    if vid in self.fleet: self.fleet[vid]['actor'].apply_control(controls[i])
            previous=self.time
            self.world.tick(); self.time=self.world.get_snapshot().timestamp.elapsed_seconds-self.origin
            dt=self.time-previous; duration+=dt
            actual={sid:str(light.get_state()).split('.')[-1] for sid,light in self.base.lights.items()}
            if actual!=desired: raise RuntimeError('Signal application mismatch; refuse to train on incorrect physical actions')
            post=self.vehicles(); complete=[]
            for vid,v in post.items():
                wp=v['wp']; row=self.fleet[vid]['row']
                if wp and wp.road_id in (1,2,3,4) and wp.lane_id>0:
                    if not v['outgoing']: raise RuntimeError('Wrong exit for '+vid)
                    if v['exit_front_distance']<=0: complete.append(vid)
            physical={'%d/%d'%(cfg['road_id'],cfg['incoming_lanes'][0]):actual[sid].upper() for sid,cfg in self.base.design['signals'].items()}
            colors={vid:physical['%d/%d'%(v['road'],v['lane'])] for vid,v in vs.items()}
            self.metrics.sample(self.time,dt,vs,complete,self.collisions,after=post)
            self.metrics.safety.sample(self.time,dt,vs,post,colors)
            self.recorder.write(self.time,dt,vs,post,complete,colors,physical,self.collisions)
            self.signals.tick(dt,any(v['occupied'] for v in post.values()))
            for vid in complete: self.retire(vid)
            if self.time>=self.limit-1e-6 or self.collisions or len(self.completed)==len(self.rows): break
        terminated=bool(self.collisions) or len(self.completed)==len(self.rows)
        truncated=self.time>=self.limit-1e-6 and not terminated
        reward=-(self.metrics.vehicle_time-before)/20-100*bool(self.collisions)
        if self.fairness_weight or self.max_wait_weight:
            waits=[r['waiting'] for r in self.metrics.rows.values() if r['time']<=self.time and r['finish'] is None]
            by=[sum(r['waiting'] for r in self.metrics.rows.values() if r['road']==road and r['time']<=self.time and r['finish'] is None) for road in (1,2,3,4)]
            reward-=self.fairness_weight*(max(by)-min(by))/20.
            reward-=self.max_wait_weight*(max(waits,default=0.)/20.)
        self.positions.write(json.dumps(dict(time=self.time,vehicles={vid:{k:x for k,x in v.items() if k!='wp'} for vid,v in self.vehicles().items()}))+'\n')
        self.trace.write(json.dumps(dict(time=self.time,action=action,phase=self.signals.active,stage=self.signals.stage,
                                        reward=reward,vla_samples=len(self.events)))+'\n'); self.trace.flush()
        return self.observe(),reward,terminated,truncated,dict(duration=duration,head_events=self.events[:])
    def close(self):
        if getattr(self,'closed',False): return
        self.closed=True
        try:
            if self.hook: self.hook.remove()
            if self.wrapper: self.wrapper.cleanup()
            if self.agent: self.agent.destroy()
            for item in self.fleet.values():
                if item['sensor'].is_alive: item['sensor'].stop(); item['sensor'].destroy()
                if item['actor'].is_alive: item['actor'].destroy()
            for _ in range(3): self.world.tick()
            for light,state,frozen in self.saved_lights: light.set_state(state); light.freeze(frozen)
            self.world.apply_settings(self.saved_settings)
            self.provider._carla_actor_pool.clear()
        finally:
            self.trace.close(); self.head_trace.close(); self.positions.close(); self.recorder.close(); self.lock.close()
