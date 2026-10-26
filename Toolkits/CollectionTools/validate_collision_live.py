"""Two real CARLA impacts with the same actor pair, separated by a quiet reset.

Requires an idle asynchronous world; controls and destroys only owned actors.
Retains per-frame physical evidence, raw callbacks and sensor coverage audit.
"""
import csv
import argparse
import json
import math
from pathlib import Path
from datetime import datetime, timezone
import carla
from evaluation.pipeline import EvaluatedCollector as FrameCollector
from evaluation.map_context import read_inventory


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--junction-id',type=int,default=1427)
    parser.add_argument('--road-id',type=int,default=17)
    parser.add_argument('--lane-id',type=int,default=-1)
    args=parser.parse_args()
    client=carla.Client('localhost',2000);client.set_timeout(10)
    world=client.get_world();original=world.get_settings()
    if original.synchronous_mode or world.get_actors().filter('vehicle.*') or world.get_actors().filter('sensor.*'):
        raise RuntimeError('Requires idle asynchronous world without vehicles')
    inventory=read_inventory(world,args.junction_id)
    stop=next(s for l in inventory['traffic_lights'] for s in l['stop_waypoints']
              if s['road_id']==args.road_id and s['lane_id']==args.lane_id)
    anchor=world.get_map().get_waypoint_xodr(args.road_id,args.lane_id,stop['s'])
    positions=[anchor.previous(distance)[0].transform for distance in (24,12)]
    for transform in positions:
        transform.location.z+=.35
    output=Path('evaluation/outputs')/('collision_acceptance_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
    output.mkdir()
    actors=[];recorder=None;collector=None;phases=[];status='failed'
    handle=(output/'physical_frames.csv').open('w',newline='',encoding='utf-8-sig')
    writer=csv.DictWriter(handle,fieldnames=['frame','sim_time','phase','vehicle_id','x','y','z','speed_mps'])
    writer.writeheader()
    try:
        settings=world.get_settings();settings.synchronous_mode=True;settings.fixed_delta_seconds=.05
        world.apply_settings(settings)
        collector=FrameCollector(world,junction_id=args.junction_id,output_root=output,collisions=True,
                                 mode='controlled_collision_acceptance',collision_config=dict(merge_gap_seconds=1.0))
        recorder=collector.collision_recorder
        for transform in positions:
            bp=world.get_blueprint_library().find('vehicle.tesla.model3')
            bp.set_attribute('role_name','collision_acceptance')
            actor=world.spawn_actor(bp,transform);actors.append(actor)
            actor.apply_control(carla.VehicleControl(brake=1,hand_brake=True))
            # Let collector.sample attach on the first observed frame, as in
            # normal experiments, so the pre-eligibility frame is exercised.
        def advance(label,count):
            start=world.get_snapshot().frame+1
            for _ in range(count):
                world.tick();snapshot=world.get_snapshot()
                live=[]
                for actor in actors:
                    state=snapshot.find(actor.id)
                    if state is None:
                        continue
                    live.append(actor.id)
                    p=state.get_transform().location;v=state.get_velocity()
                    writer.writerow(dict(frame=snapshot.frame,sim_time=snapshot.timestamp.elapsed_seconds,
                        phase=label,vehicle_id=actor.id,x=p.x,y=p.y,z=p.z,
                        speed_mps=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)))
                collector.sample(snapshot)
            handle.flush()
            phases.append(dict(label=label,start_frame=start,end_frame=world.get_snapshot().frame))
        advance('settle_no_contact',50)
        for number in (1,2):
            mover=actors[0]
            direction=positions[0].get_forward_vector()
            mover.apply_control(carla.VehicleControl(throttle=.65))
            mover.set_target_velocity(carla.Vector3D(direction.x*7,direction.y*7,0))
            advance('impact_'+str(number),80)
            for actor,transform in zip(actors,positions):
                actor.set_simulate_physics(False)
                actor.set_transform(transform)
                actor.set_target_velocity(carla.Vector3D())
                actor.set_target_angular_velocity(carla.Vector3D())
                actor.apply_control(carla.VehicleControl(brake=1,hand_brake=True))
                actor.set_simulate_physics(True)
            # Continued real ticks allow callbacks to arrive, but are NOT a
            # sensor delivery barrier or a guarantee of complete delivery.
            advance('separated_quiet_'+str(number),100)
        status='completed'
    finally:
        ids=[a.id for a in actors]
        for actor in actors:
            if actor.is_alive:
                actor.destroy()
        if recorder:
            world.tick()
            collector.sample(world.get_snapshot())
            collector.close(status)
        handle.close()
        world.apply_settings(original)
        records=[json.loads(line) for line in (collector.output/'collisions.jsonl').read_text(encoding='utf-8').splitlines()]
        aggregation=json.loads((collector.output/'collision_summary.json').read_text(encoding='utf-8'))
        callbacks=[r for r in records if r['event']=='collision_callback']
        pair_callbacks=[r for r in callbacks if {r['vehicle_id'],r['other_actor_id']}==set(ids)]
        attached={r['vehicle_id']:r['frame'] for r in records if r['event']=='sensor_attached'}
        coverage=[r for r in records if r['event']=='coverage_frame']
        missing_eligible=[dict(frame=r['frame'],vehicle_id=vid) for r in coverage
                          for vid in r['observed_vehicle_ids'] if vid in attached
                          and r['frame']>attached[vid] and vid not in r['listening_vehicle_ids']]
        coverage_passed=len(attached)==2 and not missing_eligible
        for phase in phases:
            phase['pair_callback_count']=sum(phase['start_frame']<=r['frame']<=phase['end_frame'] for r in pair_callbacks)
            phase['reporters']=sorted({r['vehicle_id'] for r in pair_callbacks if phase['start_frame']<=r['frame']<=phase['end_frame']})
        passed=(status=='completed' and len(phases)==5 and all(p['pair_callback_count']>0
                if p['label'].startswith('impact_') else p['pair_callback_count']==0 for p in phases))
        report=dict(status=status,physical_positive_passed=passed,actor_ids=ids,phases=phases,
            map_name=world.get_map().name,junction_id=args.junction_id,
            coverage_passed=coverage_passed,missing_eligible_vehicle_frames=missing_eligible,
            observed_vehicle_frames=aggregation['coverage']['observed_vehicle_frames'],
            listening_eligible_vehicle_frames=aggregation['coverage']['listening_eligible_vehicle_frames'],
            source_run=str(collector.output),pair_event_count=aggregation['pair_event_count'],
            aggregation_passed=aggregation['pair_event_count']==2,
            raw_callback_count=len(callbacks),pair_callback_count=len(pair_callbacks),
            remaining_owned_actors=sorted(set(ids)&{a.id for a in world.get_actors()}),
            remaining_collision_sensors=len(world.get_actors().filter('sensor.other.collision')),
            restored_synchronous_mode=world.get_settings().synchronous_mode,
            note='Callback counts are not incident counts; no callback-delivery completion guarantee')
        (output/'validation.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(output);print(json.dumps(report,indent=2))
    if not passed or aggregation['pair_event_count']!=2 or not coverage_passed:
        raise RuntimeError('Physical collision validation failed')


if __name__=='__main__':
    main()
