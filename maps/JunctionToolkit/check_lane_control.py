"""Test simultaneous red neighbours and one released lane on every approach."""
import argparse,json,math,time
from pathlib import Path
import carla
from project_bridge import World


def run(folder):
    folder=Path(folder);design=json.loads((folder/'manifest.json').read_text())
    client=carla.Client('localhost',2000);client.set_timeout(15);world=client.get_world()
    if world.get_settings().synchronous_mode:raise RuntimeError('World busy')
    world.wait_for_tick(10)
    if any(world.get_actors().filter(p) for p in ('vehicle.*','sensor.*')):raise RuntimeError('Actors present')
    World(world,design)
    lights={l.get_opendrive_id():l for l in world.get_actors().filter('traffic.traffic_light')}
    states=[(l,l.get_state(),l.is_frozen()) for l in lights.values()]
    original=world.get_settings();tm=client.get_trafficmanager(8011);vehicles=[];sensors=[];contacts=[]
    result=dict(passed=False,approaches=[],collisions=contacts)
    try:
        setting=world.get_settings();setting.synchronous_mode=True;setting.fixed_delta_seconds=.05;world.apply_settings(setting)
        tm.set_synchronous_mode(True);tm.set_random_device_seed(11)
        for l in lights.values():l.set_state(carla.TrafficLightState.Red);l.freeze(True)
        m=world.get_map()
        for j in design['junctions']:
            for rid,length in zip(j['roads'],j['lengths']):
                rows=[];rec=dict(road=rid,lanes=rows);result['approaches'].append(rec)
                configs=sorted([(sid,cfg) for sid,cfg in design['signals'].items() if cfg['road_id']==rid],key=lambda x:x[1]['phase_slot'])
                for sid,cfg in configs:
                    wp=m.get_waypoint_xodr(rid,cfg['incoming_lanes'][0],cfg['reference_s']-35)
                    t=wp.transform;t.location.z+=.35
                    v=world.spawn_actor(world.get_blueprint_library().find('vehicle.tesla.model3'),t);vehicles.append(v)
                    sensor=world.spawn_actor(world.get_blueprint_library().find('sensor.other.collision'),carla.Transform(),attach_to=v)
                    sensor.listen(lambda e:contacts.append(dict(frame=e.frame,actor=e.actor.id,other=e.other_actor.id)));sensors.append(sensor)
                    v.set_autopilot(True,8011);tm.auto_lane_change(v,False);tm.ignore_lights_percentage(v,0)
                    rows.append(dict(signal_id=sid,lane_id=cfg['incoming_lanes'][0],vehicle_id=v.id))
                for _ in range(400):world.tick()
                for v,row in zip(vehicles,rows):
                    wp=m.get_waypoint(v.get_location());vel=v.get_velocity()
                    row['red_stop']=wp.road_id==rid and wp.lane_id==row['lane_id'] and wp.s<length and math.sqrt(vel.x**2+vel.y**2)<.1
                    row['observed_signal']=v.get_traffic_light().get_opendrive_id() if v.get_traffic_light() else None
                    row['red_stop']=row['red_stop'] and row['observed_signal']==row['signal_id']
                for idx,(v,row) in enumerate(zip(vehicles,rows)):
                    lights[row['signal_id']].set_state(carla.TrafficLightState.Green)
                    for _ in range(400):
                        world.tick();wp=m.get_waypoint(v.get_location())
                        if wp and wp.road_id!=rid:break
                    row['green_release']=wp is not None and wp.road_id!=rid
                    row['neighbours_still_red']=all(m.get_waypoint(other.get_location()).road_id==rid and
                        other.get_velocity().length()<.1 for other in vehicles[idx+1:])
                    row['states_independent']=all(lights[r['signal_id']].get_state()==(carla.TrafficLightState.Green if k<=idx else carla.TrafficLightState.Red) for k,r in enumerate(rows))
                    # Clear released car before subsequent turns; test is lane isolation, not conflicts.
                    v.set_autopilot(False,8011);v.set_simulate_physics(False)
                rec['passed']=all(all(r[k] for k in ('red_stop','green_release','neighbours_still_red','states_independent')) for r in rows)
                print(json.dumps(rec),flush=True)
                for s in sensors:s.stop();s.destroy()
                sensors=[]
                for v in vehicles:v.destroy()
                vehicles=[]
                for l in lights.values():l.set_state(carla.TrafficLightState.Red)
        result['passed']=all(r['passed'] for r in result['approaches']) and not contacts
        return result
    finally:
        for s in sensors:s.stop();s.destroy()
        for v in vehicles:v.destroy()
        for l,state,frozen in states:l.set_state(state);l.freeze(frozen)
        tm.set_synchronous_mode(False);world.apply_settings(original)
        result['restored']=True
        (folder/'lane_control_validation.json').write_text(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('map_dir');a=p.parse_args()
    if not run(a.map_dir)['passed']:raise SystemExit(1)
