#!/usr/bin/env python3
"""Controlled live acceptance probe of the unchanged upstream collector API."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
UPSTREAM = HERE.parent/'carla_handoff/upstream_20261023_v4'
sys.path.insert(0, str(UPSTREAM))
sys.path.insert(0, str(ROOT/'Drive/software/CARLA_0.9.12_MDrive/PythonAPI/carla'))

def agent_input(sample, ordered_signals, phase_name, active_signals):
    """Pure consumer: gets only collector data, no CARLA client or world."""
    lanes = []
    for sid in ordered_signals:
        cars = [v for v in sample['vehicles'] if v.get('signal_opendrive_id')==sid]
        lanes.append(dict(signal_id=sid, state=sample['signals'][sid],
            vehicles=len(cars), stopped=sum(v['speed_mps']<.3 for v in cars)))
    return dict(frame=sample['frame'], sim_time=sample['sim_time'], phase=phase_name,
        active_signal_ids=active_signals, lanes=lanes, vehicles=sample['vehicles'],
        vehicle_context=sample['vehicle_context'], source='CARLA ground truth; not LMDrive sensors')

def main():
    import carla
    from evaluation.api import RoadSpec, create_collector
    from evaluation.map_profile import read_profile, adapt_world, validate_world
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',type=int,default=31610)
    p.add_argument('--load-map',action='store_true',help='Load Cross3 only on an idle world')
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    client=carla.Client('127.0.0.1',a.port);client.set_timeout(30)
    world=client.get_world()
    if world.get_settings().synchronous_mode or any(world.get_actors().filter(k) for k in ('vehicle.*','sensor.*','walker.*')):
        raise RuntimeError('World busy; refusing to change it')
    if a.load_map:
        from run_cross3 import load_map
        world,_,_=load_map(client,a.output)
    profile=read_profile(UPSTREAM/'map_profiles/generated_cross3/profile.json')
    wrapped=adapt_world(world,profile);validate_world(wrapped,profile)
    design=profile['_design'];sids=sorted(design['signals'])
    lights={l.get_opendrive_id():l for l in world.get_actors().filter('traffic.traffic_light')}
    original=world.get_settings()
    saved=[(l,l.get_state(),l.is_frozen()) for l in lights.values()]
    actors=[];collector=None;report=dict(status='running',checks=0,frames=0,errors=[],
        client_version=client.get_client_version(),server_version=client.get_server_version(),
        map_sha256=hashlib.sha256(world.get_map().to_opendrive().encode()).hexdigest(),
        phase_limit='Original phase CSV has singular active_signal_opendrive_id; multi-signal phase supplied separately to agent adapter',
        scope='12 controlled vehicles; 12 individual signals and four simultaneous phase patterns; not full traffic metric or LMDrive prompt acceptance')
    def check(ok,message):
        report['checks']+=1
        if not ok:
            report['errors'].append(message)
    try:
        settings=world.get_settings();settings.synchronous_mode=True;settings.fixed_delta_seconds=.05
        world.apply_settings(settings)
        for light in lights.values():light.freeze(True);light.set_state(carla.TrafficLightState.Red)
        for sid in sids:
            cfg=design['signals'][sid]
            wp=world.get_map().get_waypoint_xodr(cfg['road_id'],cfg['incoming_lanes'][0],100)
            t=wp.transform;t.location.z+=.4
            bp=world.get_blueprint_library().find('vehicle.tesla.model3');bp.set_attribute('role_name','collector_probe_'+sid)
            actor=world.spawn_actor(bp,t);actors.append(actor)
            actor.apply_control(carla.VehicleControl(brake=1.))
        for _ in range(30):world.tick()
        collector=create_collector(wrapped,RoadSpec(junction_id=10),output_root=a.output/'collector',collisions=False)
        report['collector_output']=str(collector.output)
        patterns=[('single_'+sid,[sid]) for sid in sids]
        for name,roads,lane in [('NS_STRAIGHT',(2,4),-2),('NS_LEFT',(2,4),-1),('EW_STRAIGHT',(1,3),-2),('EW_LEFT',(1,3),-1)]:
            patterns.append((name,[sid for sid,c in design['signals'].items() if c['incoming_lanes']==[-3] or (c['road_id'] in roads and c['incoming_lanes']==[lane])]))
        samples=[]
        with (a.output/'agent_observations.jsonl').open('w') as output:
            for name,active in patterns:
                for sid,l in lights.items():l.set_state(carla.TrafficLightState.Green if sid in active else carla.TrafficLightState.Red)
                for step in range(40 if name.startswith('EW_') else 8):
                    for actor in actors:actor.apply_control(carla.VehicleControl(throttle=.65 if name.startswith('EW_') else 0.,brake=0. if name.startswith('EW_') else 1.))
                    world.tick();snapshot=world.get_snapshot()
                    sample=collector.sample(snapshot)
                    # Allow asynchronous command application only on first transition frame.
                    if step>=1:
                        check(all(sample['signals'][sid]==('Green' if sid in active else 'Red') for sid in sids),'signal mismatch '+name)
                    check(sample['frame']==snapshot.frame and sample['frame_gap']==0 and not sample['read_overlapped_tick'],'frame alignment')
                    check(len(sample['vehicles'])==12 and len(sample['signals'])==12,'entity counts')
                    for row in sample['vehicles']:
                        expected=row['role_name'].split('_')[-1]
                        check(row['signal_match']=='unique' and row.get('signal_opendrive_id')==expected,'lane association '+expected)
                        check(row.get('signal_state')==sample['signals'][expected],'vehicle light state '+expected)
                        state=snapshot.find(row['vehicle_id']);pos=state.get_transform().location;v=state.get_velocity()
                        check(abs(row['x']-pos.x)<1e-6 and abs(row['y']-pos.y)<1e-6,'position')
                        check(abs(row['speed_mps']-math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z))<1e-6,'velocity')
                    packet=agent_input(sample,sids,name,active)
                    check(len(packet['lanes'])==12 and sum(l['vehicles'] for l in packet['lanes'])==12,'agent consumer')
                    check(len(sample['vehicle_context'])==12 and all(c.get('geometry_match')=='unique'
                        for c in sample['vehicle_context']),'approach geometry context')
                    output.write(json.dumps(packet)+'\n');samples.append(sample)
        check(max(v['speed_mps'] for s in samples for v in s['vehicles'])>.5,'moving vehicle coverage')
        report.update(frames=len(samples),patterns=[n for n,_ in patterns],
            max_speed_mps=max(v['speed_mps'] for s in samples for v in s['vehicles']),
            context_count_last=len(samples[-1]['vehicle_context']),status='passed' if not report['errors'] else 'failed')
        (a.output/'agent_observation_example.json').write_text(json.dumps(packet,indent=2))
    except Exception as exc:
        report.update(status='error',exception=repr(exc))
        raise
    finally:
        if collector:collector.close(report['status'])
        for actor in actors:
            if actor.is_alive:actor.destroy()
        for light,state,frozen in saved:light.set_state(state);light.freeze(frozen)
        world.tick();world.apply_settings(original)
        (a.output/'report.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report),flush=True)
    if report['status']!='passed':raise SystemExit(1)

if __name__=='__main__':main()
