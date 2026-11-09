"""Experimental finite random demand with lane-separated provisional signal timings."""
import argparse,datetime,hashlib,json,random,time
from pathlib import Path
import carla
from roundabout_drive_check import route,sparse_path


def run(a):
    folder=Path(a.map_dir);d=json.loads((folder/'manifest.json').read_text());plan=json.loads((folder/'lane_phase_candidate.json').read_text())
    c=carla.Client('localhost',2000);c.set_timeout(15);w=c.get_world();m=w.get_map();world_id=w.id
    if w.get_settings().synchronous_mode or any(w.get_actors().filter(p) for p in ('vehicle.*','sensor.*','walker.*')):raise RuntimeError('World busy')
    if hashlib.sha256(m.to_opendrive().encode()).hexdigest()!=d['opendrive_sha256']:raise RuntimeError('Load target map first')
    out=folder/'traffic_runs'/datetime.datetime.now().strftime('%Y%m%d_%H%M%S');out.mkdir(parents=True)
    rng=random.Random(a.seed);tm=c.get_trafficmanager(8012);original=w.get_settings()
    lights={l.get_opendrive_id():l for l in w.get_actors().filter('traffic.traffic_light')}
    vehicles={};sensors={};rows=[];contacts=[];events=[];result=dict(seed=a.seed,vehicles_per_approach=a.vehicles_per_approach,rows=rows,collisions=contacts,phases=events,experimental=True,opendrive_sha256=d['opendrive_sha256'],timings=dict(green=18,yellow=3,all_red=4))
    def same():return c.get_world().id==world_id
    def clear(key):
        s=sensors.pop(key,None);v=vehicles.pop(key,None)
        if s and s.is_alive:s.stop();s.destroy()
        if v and v.is_alive:v.destroy()
    try:
        setting=w.get_settings();setting.synchronous_mode=True;setting.fixed_delta_seconds=.05;w.apply_settings(setting);tm.set_synchronous_mode(True);tm.set_random_device_seed(a.seed)
        for l in lights.values():l.freeze(True);l.set_state(carla.TrafficLightState.Red)
        for src in range(1,5):
            for k in range(a.vehicles_per_approach):
                lane=-(k%2+1);dst=rng.randint(1,4);path=route(m,d,src,lane,dst)
                wp=m.get_waypoint_xodr(src,lane,d['config']['arm_length_m']-35-(k//2)*10)
                t=wp.transform;t.location.z+=.4
                v=w.try_spawn_actor(w.get_blueprint_library().find('vehicle.tesla.model3'),t)
                row=dict(source=src,lane=lane,target=dst,status='spawn_failed',speed_kmh=rng.uniform(14,18));rows.append(row)
                if not v:continue
                row.update(actor_id=v.id,status='running',stopped_seconds=0.);vehicles[v.id]=v
                s=w.spawn_actor(w.get_blueprint_library().find('sensor.other.collision'),carla.Transform(),attach_to=v);sensors[v.id]=s
                s.listen(lambda e:contacts.append(dict(frame=e.frame,actor=e.actor.id,other=e.other_actor.id,impulse=e.normal_impulse.length())))
                v.set_autopilot(True,8012);tm.auto_lane_change(v,False);tm.ignore_lights_percentage(v,0);tm.set_desired_speed(v,row['speed_kmh']);tm.set_path(v,sparse_path(path))
        last=None
        for tick in range(int(a.duration/.05)):
            wall=time.monotonic()
            if not same():raise RuntimeError('World changed')
            elapsed=tick*.05;slot=int(elapsed//25)%4;within=elapsed%25;stage='green' if within<18 else ('yellow' if within<21 else 'red');key=(slot,stage)
            if key!=last:
                p=plan['phases'][slot];green=p['green_signal_ids'];state=carla.TrafficLightState.Green if stage=='green' else carla.TrafficLightState.Yellow
                for sid,l in lights.items():l.set_state(state if sid in green and stage!='red' else carla.TrafficLightState.Red)
                events.append(dict(seconds=elapsed,phase=p['name'],stage=stage));last=key
            w.tick(10.)
            for row in rows:
                if row['status']!='running':continue
                v=vehicles[row['actor_id']]
                if not v.is_alive:row['status']='actor_lost';clear(row['actor_id']);continue
                wp=m.get_waypoint(v.get_location());row['last_waypoint']=dict(road=wp.road_id,lane=wp.lane_id,s=wp.s)
                if v.get_velocity().length()<.1:row['stopped_seconds']+=.05
                if wp.road_id==row['target'] and wp.lane_id==-row['lane'] and wp.s<d['config']['arm_length_m']-20:
                    row.update(status='completed',completed_seconds=elapsed);clear(row['actor_id'])
            if tick%400==0:
                print('sim',round(elapsed),'completed',sum(r['status']=='completed' for r in rows),'active',len(vehicles),'collision callbacks',len(contacts),flush=True)
                (out/'result.json').write_text(json.dumps(result,indent=2))
            if not vehicles:break
            if a.pace>0:time.sleep(max(0,a.pace-(time.monotonic()-wall)))
        for row in rows:
            if row['status']=='running':row['status']='unfinished_at_timeout'
        result['simulation_seconds']=(tick+1)*.05
    except BaseException as e:
        result['error']=str(e);raise
    finally:
        if same():
            for key in list(vehicles):clear(key)
            for l in lights.values():l.set_state(carla.TrafficLightState.Red);l.freeze(True)
            tm.set_synchronous_mode(False);w.apply_settings(original)
        result['completed']=sum(r['status']=='completed' for r in rows);result['requested']=len(rows)
        (out/'result.json').write_text(json.dumps(result,indent=2));print('Result:',out,'completed',result['completed'],'/',len(rows),'collision callbacks',len(contacts),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('map_dir');p.add_argument('--vehicles-per-approach',type=int,default=2);p.add_argument('--seed',type=int,default=42);p.add_argument('--duration',type=float,default=360);p.add_argument('--pace',type=float,default=.02);a=p.parse_args()
    if not 1<=a.vehicles_per_approach<=12 or a.duration<=0 or a.pace<0:p.error('Invalid vehicle count/duration/pace')
    run(a)
