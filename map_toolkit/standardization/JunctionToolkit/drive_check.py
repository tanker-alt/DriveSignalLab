"""Physical smoke check: one straight-going vehicle per grid row.

All lights are green for this isolated non-conflicting test. This is NOT a
multi-junction signal-control evaluation or a collision-free certification.
"""
import argparse
import json
import time
from pathlib import Path
import carla
from project_bridge import World


def run(folder):
    folder=Path(folder);d=json.loads((folder/'manifest.json').read_text())
    if d['config'].get('type') in ('t','t_junction'):
        raise ValueError('Use movement_check.py for a T junction; grid straight smoke requires four arms')
    c=carla.Client('localhost',2000);c.set_timeout(20);w=c.get_world()
    if w.get_settings().synchronous_mode:raise RuntimeError('Synchronous world already owned')
    w.wait_for_tick(15)
    if w.get_actors().filter('vehicle.*') or w.get_actors().filter('sensor.*'):raise RuntimeError('Active experiment actors')
    World(w,d)
    cfg=d['config'];straight=[-(i+1) for i,turns in enumerate(cfg['allowed_turns']) if 'straight' in turns]
    if not straight:raise ValueError('Smoke test requires a straight lane')
    lane=straight[0];m=w.get_map();original=w.get_settings();tm=c.get_trafficmanager(8010)
    actors=[];sensors=[];collisions=[];records=[];lights=list(w.get_actors().filter('traffic.traffic_light'))
    light_states=[(l,l.get_state(),l.is_frozen()) for l in lights]
    try:
        s=w.get_settings();s.synchronous_mode=True;s.fixed_delta_seconds=.05;w.apply_settings(s)
        tm.set_synchronous_mode(True);tm.set_random_device_seed(42)
        for l in lights:l.set_state(carla.TrafficLightState.Green);l.freeze(True)
        for row in range(cfg['rows']):
            j=d['junctions'][row*cfg['columns']]
            wp=m.get_waypoint_xodr(j['roads'][0],lane,j['lengths'][0]-65)
            t=wp.transform;t.location.z+=.35
            v=w.spawn_actor(w.get_blueprint_library().find('vehicle.tesla.model3'),t);actors.append(v)
            sensor=w.spawn_actor(w.get_blueprint_library().find('sensor.other.collision'),carla.Transform(),attach_to=v)
            sensor.listen(lambda e:collisions.append(dict(frame=e.frame,actor=e.actor.id,other=e.other_actor.id)));sensors.append(sensor)
            v.set_autopilot(True,8010);tm.auto_lane_change(v,False);tm.vehicle_percentage_speed_difference(v,-10)
            tm.set_route(v,['Straight']*cfg['columns'])
            target=d['junctions'][row*cfg['columns']+cfg['columns']-1]['roads'][2]
            records.append(dict(vehicle_id=v.id,row=row,target_road=target,visited_junctions=[],min_z=999,completed=False))
        deadline=time.monotonic()+600
        for tick in range(4000):
            if time.monotonic()>deadline:raise RuntimeError('Smoke test wall-clock timeout')
            w.tick()
            for v,rec in zip(actors,records):
                if rec['completed']:continue
                loc=v.get_location();rec['min_z']=min(rec['min_z'],loc.z)
                wp=m.get_waypoint(loc)
                if wp and wp.is_junction and wp.junction_id not in rec['visited_junctions']:rec['visited_junctions'].append(wp.junction_id)
                if wp and wp.road_id==rec['target_road'] and wp.lane_id==-lane:
                    length=next(j['lengths'][2] for j in d['junctions'] if j['roads'][2]==rec['target_road'])
                    if wp.s<length-25:
                        rec['completed']=True;rec['completion_frame']=w.get_snapshot().frame
                        v.set_autopilot(False,8010);v.apply_control(carla.VehicleControl(brake=1,hand_brake=True))
            if all(r['completed'] for r in records):break
        result=dict(passed=all(r['completed'] and len(r['visited_junctions'])==cfg['columns'] and r['min_z']>-.5 for r in records) and not collisions,
                    vehicles=records,collision_callbacks=collisions,signal_policy='all green, isolated straight-only smoke',
                    limits='No conflicting movements, traffic demand, multi-junction controller or complete sensor coverage certification.')
        (folder/'drive_validation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
        return result
    finally:
        for sensor in sensors:
            sensor.stop();sensor.destroy()
        for v in actors:v.destroy()
        for l,state,frozen in light_states:l.set_state(state);l.freeze(frozen)
        tm.set_synchronous_mode(False);w.apply_settings(original)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('map_dir');a=p.parse_args()
    if not run(a.map_dir)['passed']:raise SystemExit(1)
