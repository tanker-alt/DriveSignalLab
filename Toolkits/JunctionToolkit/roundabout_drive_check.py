"""Sequential physical route checks; green lights, one vehicle, no phase claim."""
import argparse,collections,hashlib,json,time,math
from pathlib import Path
import carla


def route(m,design,source,lane,target):
    length=design['config']['arm_length_m']
    start=m.get_waypoint_xodr(source,lane,length-35)
    pending=collections.deque([(start,[])]);seen=set()
    while pending:
        wp,history=pending.popleft();key=(wp.road_id,wp.lane_id,round(wp.s))
        if key in seen:continue
        seen.add(key);path=history+[wp]
        if wp.road_id==target and wp.lane_id==-lane and wp.s<length-25:return path
        if wp.road_id in range(1,5) and wp.lane_id>0 and wp.road_id!=target:continue
        for nxt in wp.next(2.):pending.append((nxt,path))
    raise RuntimeError('No route {} {} -> {}'.format(source,lane,target))


def sparse_path(points):
    """TM anchors outside overlapping junction connectors; retain final destination."""
    anchors=[];last_road=None
    for point in points[1:]:
        if not point.is_junction and point.road_id!=last_road:
            anchors.append(point.transform.location);last_road=point.road_id
    anchors.append(points[-1].transform.location)
    return anchors


def run(folder,limit=0,controller="physical",report=None):
    report=report or ('tm_sparse_validation.json' if controller=='tm' else 'drive_validation.json')
    folder=Path(folder);d=json.loads((folder/'manifest.json').read_text())
    c=carla.Client('localhost',2000);c.set_timeout(20);w=c.get_world();w.wait_for_tick(15)
    if w.get_settings().synchronous_mode or any(w.get_actors().filter(p) for p in ('vehicle.*','sensor.*','walker.*')):raise RuntimeError('World busy')
    m=w.get_map()
    if hashlib.sha256(m.to_opendrive().encode()).hexdigest()!=d['opendrive_sha256']:raise RuntimeError('Map differs')
    original=w.get_settings();lights=[(a,a.get_state(),a.is_frozen()) for a in w.get_actors().filter('traffic.traffic_light')]
    tm=c.get_trafficmanager(8012);v=None;sensor=None;records=[];result=dict(passed=False,routes=records)
    cases=[(src,lane,dst) for src in range(1,5) for lane in (-1,-2) for dst in range(1,5)]
    if limit:cases=cases[:limit]
    try:
        settings=w.get_settings();settings.synchronous_mode=True;settings.fixed_delta_seconds=.05;w.apply_settings(settings)
        tm.set_synchronous_mode(True);tm.set_random_device_seed(42)
        for l,_,_ in lights:l.set_state(carla.TrafficLightState.Green);l.freeze(True)
        for src,lane,dst in cases:
            points=route(m,d,src,lane,dst)
            rec=dict(source=src,lane=lane,target=dst,target_lane=-lane,completed=False,collisions=[],min_z=999.,visited_roads=[])
            records.append(rec)
            try:
                t=points[0].transform;t.location.z+=.4
                v=w.spawn_actor(w.get_blueprint_library().find('vehicle.tesla.model3'),t)
                sensor=w.spawn_actor(w.get_blueprint_library().find('sensor.other.collision'),carla.Transform(),attach_to=v)
                sensor.listen(lambda e,row=rec:row['collisions'].append(dict(frame=e.frame,other=e.other_actor.id)))
                for _ in range(20):w.tick()
                cursor=0;rec['max_tracking_error_m']=0.;rec['controller']=controller
                if controller=='tm':
                    # Avoid ambiguous nearest-waypoint lookup on overlapping junction connectors.
                    anchors=sparse_path(points)
                    v.set_autopilot(True,8012);tm.auto_lane_change(v,False)
                    tm.set_desired_speed(v,18.);tm.set_path(v,anchors)
                    rec['anchor_count']=len(anchors)
                deadline=time.monotonic()+100
                for tick in range(2200):
                    if time.monotonic()>deadline:rec['failure']='wall_timeout';break
                    loc=v.get_location();transform=v.get_transform();speed=v.get_velocity().length()
                    cursor=min(range(cursor,min(cursor+12,len(points))),key=lambda k:loc.distance(points[k].transform.location))
                    rec['max_tracking_error_m']=max(rec['max_tracking_error_m'],loc.distance(points[cursor].transform.location))
                    goal=points[min(cursor+3,len(points)-1)].transform.location
                    dx,dy=goal.x-loc.x,goal.y-loc.y;yaw=math.radians(transform.rotation.yaw)
                    lateral=-dx*math.sin(yaw)+dy*math.cos(yaw)
                    steering=math.atan2(2*2.875*lateral,max(dx*dx+dy*dy,.1))/math.radians(70)
                    if controller=='physical':v.apply_control(carla.VehicleControl(throttle=max(0.,min(.6,.25*(5-speed))),brake=max(0.,min(.5,.3*(speed-5))),steer=max(-1.,min(1.,steering))))
                    w.tick();loc=v.get_location();rec['min_z']=min(rec['min_z'],loc.z);at=m.get_waypoint(loc)
                    if at and (not rec['visited_roads'] or rec['visited_roads'][-1]!=at.road_id):rec['visited_roads'].append(at.road_id)
                    if at and at.road_id==dst and at.lane_id==-lane and at.s<d['config']['arm_length_m']-20:
                        rec['completed']=True;break
                    if loc.z<-1:rec['failure']='fell_below_road';break
                rec['simulation_seconds']=(tick+1)*.05
                rec['final_waypoint']=dict(road=at.road_id,lane=at.lane_id,s=at.s) if at else None
                for _ in range(3):w.tick()
                rec['passed']=rec['completed'] and rec['min_z']>-.5 and not rec['collisions']
                print(json.dumps(rec),flush=True)
            finally:
                if sensor:sensor.stop();sensor.destroy();sensor=None
                if v:v.destroy();v=None
                w.tick()
            (folder/report).write_text(json.dumps(result,indent=2))
        result['passed']=len(records)==len(cases) and all(r['passed'] for r in records)
    finally:
        if sensor:sensor.stop();sensor.destroy()
        if v:v.destroy()
        for light,state,frozen in lights:light.set_state(state);light.freeze(frozen)
        tm.set_synchronous_mode(False);w.apply_settings(original)
        result.update(world_settings_restored=True,opendrive_sha256=d['opendrive_sha256'],expected_cases=len(cases),
                      controller=controller,limits='One Model3 at a time, all lights green. Does not certify signal stopping, multi-vehicle conflict safety, or large vehicles. Tracking error measures distance to sampled points, not pure lateral deviation.')
        (folder/report).write_text(json.dumps(result,indent=2))
    print('Passed:',result['passed'],flush=True)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('map_dir');p.add_argument('--limit',type=int,default=0);p.add_argument('--controller',choices=['physical','tm'],default='physical');p.add_argument('--report');a=p.parse_args()
    if not run(a.map_dir,a.limit,a.controller,a.report)['passed']:raise SystemExit(1)
