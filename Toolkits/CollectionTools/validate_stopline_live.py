"""Validate configured stop edges with physical vehicle bounding boxes.

Idle asynchronous world required. No signal writes; all created actors cleaned.
Placement cases are static geometry checks, not real driving/violation episodes.
"""
import argparse
import json
import math
from pathlib import Path
from datetime import datetime, timezone
import carla
from evaluation.pipeline import EvaluatedCollector as FrameCollector


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True)
    args=parser.parse_args()
    calibration=json.loads(args.config.read_text(encoding='utf-8'))
    client=carla.Client('localhost',2000);client.set_timeout(10)
    world=client.get_world();original=world.get_settings()
    if original.synchronous_mode or world.get_actors().filter('vehicle.*'):
        raise RuntimeError('Requires an idle asynchronous world')
    output=Path('evaluation/outputs')/('stopline_acceptance_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
    output.mkdir()
    collector=None;actors=[];checks=[];status='failed'
    try:
        settings=world.get_settings();settings.synchronous_mode=True;settings.fixed_delta_seconds=.05
        world.apply_settings(settings)
        collector=FrameCollector(world,junction_id=calibration['junction_id'],output_root=output,
            stop_reference_config=calibration,mode='static_stopline_acceptance')
        def tick():
            world.tick()
            return collector.sample(world.get_snapshot())
        for key,item in calibration['approaches'].items():
            line=item['stop_line'];p=line['center'];nx,ny=line['normal_xy']
            yaw=math.degrees(math.atan2(ny,nx))
            for target in (.5,0.,-.5):
                bp=world.get_blueprint_library().find('vehicle.tesla.model3')
                bp.set_attribute('role_name','stopline_acceptance')
                actor=world.spawn_actor(bp,carla.Transform(carla.Location(p['x']-5*nx,p['y']-5*ny,p['z']+.5),carla.Rotation(yaw=yaw)))
                actors.append(actor)
                actor.set_simulate_physics(False)
                distance=target+actor.bounding_box.extent.x+actor.bounding_box.location.x
                actor.set_transform(carla.Transform(carla.Location(p['x']-distance*nx,p['y']-distance*ny,p['z']+.35),carla.Rotation(yaw=yaw)))
                actor.apply_control(carla.VehicleControl(brake=1,hand_brake=True))
                actor.set_simulate_physics(True)
                for _ in range(30):
                    packet=tick()
                row=next(r for r in packet['vehicles'] if r['vehicle_id']==actor.id)
                vertices=json.loads(row['bbox_vertices_world_json'])
                # Independent plane projection; no production path projection.
                plane=min((p['x']-x)*nx+(p['y']-y)*ny for x,y,z in vertices)
                detail=next(d for d in packet['vehicle_context'] if d['vehicle_id']==str(actor.id))
                actual=detail.get('front_reference_distance_m')
                error=abs(actual-plane) if actual is not None else None
                plane_field=detail.get('stop_line_front_distance_m')
                passed=(detail.get('approach_id')==key and actual is not None and error<.10
                        and plane_field is not None and abs(plane_field-plane)<1e-6
                        and abs(plane-target)<.10 and row['speed_mps']<.1)
                if target:
                    passed=passed and actual*target>0
                check=dict(approach_id=key,target_front_distance_m=target,independent_plane_distance_m=plane,
                    actual=detail,error_m=error,passed=passed,frame=packet['frame'],row=row)
                checks.append(check)
                print(key,target,actual,plane,passed,flush=True)
                actor.destroy();tick()
                if actor.id in {a.id for a in world.get_actors().filter('vehicle.*')}:
                    raise RuntimeError('Test actor cleanup failed')
                actors=[]
        status='completed' if all(c['passed'] for c in checks) else 'failed'
    finally:
        for actor in actors:
            if actor.is_alive:
                actor.destroy()
        if collector:
            collector.close(status)
        world.apply_settings(original)
        result=dict(status=status,calibration=str(args.config),checks=checks,
            max_error_m=max((c['error_m'] for c in checks if c['error_m'] is not None),default=None),
            source_run=str(collector.output) if collector else None,
            remaining_vehicles=len(world.get_actors().filter('vehicle.*')),
            synchronous_mode=world.get_settings().synchronous_mode,
            note='Static geometry acceptance. Teleports are not driving or violation evidence.')
        (output/'report.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        print(output,status)
    if status!='completed':
        raise RuntimeError('Stop-line acceptance failed; inspect report')


if __name__=='__main__':
    main()
