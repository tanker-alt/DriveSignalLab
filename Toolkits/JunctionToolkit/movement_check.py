"""Sequential single-vehicle physical check of every manifest movement."""
import argparse
import json
import time
from pathlib import Path
import carla
from project_bridge import World


def run(folder):
    folder=Path(folder);d=json.loads((folder/'manifest.json').read_text())
    if len(d['junctions'])!=1:raise ValueError('Single junction only')
    c=carla.Client('localhost',2000);c.set_timeout(20);w=c.get_world()
    if w.get_settings().synchronous_mode:raise RuntimeError('Synchronous world already owned')
    w.wait_for_tick(15)
    if any(w.get_actors().filter(p) for p in ('vehicle.*','sensor.*','walker.*')):raise RuntimeError('Active experiment actors')
    World(w,d).get_traffic_lights_in_junction(d['junctions'][0]['id'])
    original=w.get_settings();tm=c.get_trafficmanager(8010);m=w.get_map()
    lengths=dict(zip(d['junctions'][0]['roads'],d['junctions'][0]['lengths']))
    lights=[(l,l.get_state(),l.is_frozen()) for l in w.get_actors().filter('traffic.traffic_light')]
    records=[];vehicle=None;sensor=None;result=dict(passed=False,movements=records)
    try:
        s=w.get_settings();s.synchronous_mode=True;s.fixed_delta_seconds=.05;w.apply_settings(s)
        tm.set_synchronous_mode(True);tm.set_random_device_seed(42)
        for l,_,_ in lights:l.set_state(carla.TrafficLightState.Green);l.freeze(True)
        for move in d['movements']:
            rec=dict(move,completed=False,min_z=999.,collisions=[]);records.append(rec)
            wp=m.get_waypoint_xodr(move['road'],move['lane'],lengths[move['road']]-50)
            t=wp.transform;t.location.z+=.35
            try:
                vehicle=w.spawn_actor(w.get_blueprint_library().find('vehicle.tesla.model3'),t)
                sensor=w.spawn_actor(w.get_blueprint_library().find('sensor.other.collision'),carla.Transform(),attach_to=vehicle)
                sensor.listen(lambda e,rec=rec:rec['collisions'].append(dict(frame=e.frame,other=e.other_actor.id)))
                for _ in range(10):w.tick()
                vehicle.set_autopilot(True,8010);tm.auto_lane_change(vehicle,False)
                tm.set_route(vehicle,[move['movement'].capitalize()])
                deadline=time.monotonic()+90
                for _ in range(1600):
                    if time.monotonic()>deadline:break
                    w.tick();loc=vehicle.get_location();rec['min_z']=min(rec['min_z'],loc.z)
                    at=m.get_waypoint(loc)
                    if at and at.road_id==move['target_road'] and at.lane_id==move['target_lane'] and at.s<lengths[move['target_road']]-15:
                        rec['completed']=True;break
                    if loc.z < -1:break
                for _ in range(3):w.tick()
                rec['passed']=rec['completed'] and rec['min_z']>-.5 and not rec['collisions']
                print(move,'passed:',rec['passed'],flush=True)
            finally:
                if sensor:sensor.stop();sensor.destroy();sensor=None
                if vehicle:vehicle.destroy();vehicle=None
        result['passed']=all(r['passed'] for r in records)
        return result
    finally:
        if sensor:sensor.stop();sensor.destroy()
        if vehicle:vehicle.destroy()
        for light,state,frozen in lights:light.set_state(state);light.freeze(frozen)
        tm.set_synchronous_mode(False);w.apply_settings(original)
        result['world_settings_restored']=True
        result['limits']='One vehicle per movement, all green. Not conflict safety, traffic capacity or complete collision coverage certification.'
        (folder/'movement_validation.json').write_text(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('map_dir');a=p.parse_args()
    if not run(a.map_dir)['passed']:raise SystemExit(1)
