"""Export painted-stop-line review material or preview a candidate road s.

Exports are survey drafts, NOT accepted calibration configurations. No ticking,
map changes or signal writes. --capture owns temporary RGB sensors only.
"""
import argparse
import json
import math
import queue
from pathlib import Path
from datetime import datetime, timezone
import carla
from evaluation.map_context import read_inventory


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',default='localhost')
    parser.add_argument('--port',type=int,default=2000)
    parser.add_argument('--junction-id',type=int,default=1427)
    parser.add_argument('--approach',help='road_section_lane')
    parser.add_argument('--s',type=float,help='Candidate OpenDRIVE road s, NOT distance from stop')
    parser.add_argument('--forward-m',type=float,help='Preview straight forward offset in metres (visual survey only)')
    parser.add_argument('--draw',action='store_true')
    parser.add_argument('--focus',action='store_true')
    parser.add_argument('--capture',action='store_true')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args(argv)
    if args.s is not None and (not args.approach or not math.isfinite(args.s) or args.s<0):
        parser.error('--s requires --approach and a finite nonnegative value')
    if args.forward_m is not None and (not args.approach or args.s is not None or not math.isfinite(args.forward_m)):
        parser.error('--forward-m requires --approach and cannot be combined with --s')
    client=carla.Client(args.host,args.port);client.set_timeout(10)
    world=client.get_world();road_map=world.get_map()
    inventory=read_inventory(world,args.junction_id)
    records={}
    for light in inventory['traffic_lights']:
        for stop in light['stop_waypoints']:
            key='{}_{}_{}'.format(stop['road_id'],stop['section_id'],stop['lane_id'])
            records[key]=stop
    if args.approach:
        if args.approach not in records:
            parser.error('Unknown approach: '+args.approach)
        records={args.approach:records[args.approach]}
    if args.capture and world.get_settings().synchronous_mode:
        parser.error('Capture requires an asynchronous world; this tool never ticks')
    output=args.output or Path('evaluation/outputs')/('stopline_review_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
    output.mkdir(parents=True,exist_ok=False)
    survey=dict(map_name=inventory['map_name'],junction_id=args.junction_id,
        opendrive_sha256=inventory['opendrive_sha256'],status='unverified_survey_draft',
        convention=None,approaches={})
    for key,stop in records.items():
        s=args.s if args.s is not None else stop['s']
        wp=road_map.get_waypoint_xodr(stop['road_id'],stop['lane_id'],s)
        if wp is None or wp.section_id!=stop['section_id']:
            raise ValueError('Candidate leaves the supported road section')
        center=wp.transform.location
        if args.forward_m is not None:
            forward=wp.transform.get_forward_vector()
            center.x+=forward.x*args.forward_m;center.y+=forward.y*args.forward_m
        right=wp.transform.get_right_vector()
        endpoints=[carla.Location(x=center.x+sign*right.x*wp.lane_width/2,
                                 y=center.y+sign*right.y*wp.lane_width/2,z=center.z+.10) for sign in (-1,1)]
        if args.draw:
            world.debug.draw_line(endpoints[0],endpoints[1],thickness=.04,color=carla.Color(255,0,255),life_time=120)
            world.debug.draw_string(carla.Location(center.x,center.y,center.z+.15),
                ('{} offset={:.3f}m'.format(key,args.forward_m) if args.forward_m is not None
                 else '{} s={:.3f}'.format(key,s)),color=carla.Color(255,0,255),life_time=120)
        camera_pose=carla.Transform(carla.Location(center.x,center.y,center.z+16),
                                   carla.Rotation(pitch=-90,yaw=wp.transform.rotation.yaw))
        if args.focus:
            world.get_spectator().set_transform(camera_pose)
        if args.capture:
            bp=world.get_blueprint_library().find('sensor.camera.rgb')
            for name,value in [('image_size_x','1024'),('image_size_y','1024'),('fov','70'),('sensor_tick','.1')]:
                bp.set_attribute(name,value)
            for name in ('lens_k','lens_kcube'):
                if bp.has_attribute(name):
                    bp.set_attribute(name,'0')
            sensor=world.spawn_actor(bp,camera_pose)
            images=queue.Queue()
            try:
                sensor.listen(images.put)
                image=images.get(timeout=15)
                image.save_to_disk(str(output/(key+'.png')))
            finally:
                sensor.stop();sensor.destroy()
        survey['approaches'][key]=dict(carla_reference_s=stop['s'],preview_s=s if args.forward_m is None else None,
            forward_offset_m=args.forward_m,
            reference_location=stop['location'],preview_location=dict(x=center.x,y=center.y,z=center.z),
            measured_s=None,estimated_error_m=None,reviewer=None,evidence=None)
        print(key,'reference_s=',stop['s'],'preview_s=',s,flush=True)
    (output/'survey_draft.json').write_text(json.dumps(survey,indent=2),encoding='utf-8')
    (output/'inventory.json').write_text(json.dumps(inventory,indent=2),encoding='utf-8')
    print(output)


if __name__=='__main__':
    main()
