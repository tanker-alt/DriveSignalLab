"""One physical vehicle traverses a cross/T/mixed network; spectator follows it.

Run with carla-env. All-green isolated connectivity acceptance, not a traffic
signal/controller evaluation. Uses CARLA next() for road/junction traversal,
and explicit smooth same-direction lane changes on broken approach markings.
"""
import argparse
import collections
import hashlib
import json
import math
import time
from pathlib import Path

import carla
from network_itinerary import itinerary


def waypoint_record(w):
    p = w.transform.location
    return dict(road=w.road_id, lane=w.lane_id, s=w.s, x=p.x, y=p.y, z=p.z)


def audit_seams(report, route, design):
    if design['config'].get('type')=='heterogeneous':
        from heterogeneous_route import audit
        return audit(report,route,design)
    if design['config'].get('type')=='roundabout':
        from roundabout_route import audit
        return audit(report,route,design)
    seams = {frozenset((s['road_a'], s['road_b'])) for s in design['seams']}
    transitions = report['road_transitions']
    observed = [[a['road'], b['road']] for a,b in zip(transitions, transitions[1:])
                if frozenset((a['road'], b['road'])) in seams]
    expected = [[a['target_road'], b['road']] for a,b in zip(route['movements'], route['movements'][1:])]
    report['observed_seams'] = observed
    report['expected_seams'] = expected
    report['seam_sequence_passed'] = observed == expected
    return observed == expected


def follow(start, road, lane, station):
    todo = collections.deque([(start, [])]); seen = set()
    while todo:
        w, history = todo.popleft()
        key = (w.road_id, w.lane_id, round(w.s, 1))
        if key in seen:
            continue
        seen.add(key); path = history + [w]
        if w.road_id == road and w.lane_id == lane and abs(w.s-station) < 1.1:
            return path
        if len(seen) > 20000:
            break
        todo.extend((n, path) for n in w.next(1.))
    raise ValueError('No waypoint continuation to %s/%s/%s' % (road, lane, station))


def plan(m, d, required=None):
    selected = itinerary(d, required)
    junctions = {j['id']:j for j in d['junctions']}
    points = []; changes = []; movements = []
    start = None
    for k, move in enumerate(selected):
        j = junctions[move['junction_id']]; src = move['road']; dst = move['target_road']
        movements.append(move)
        if start is None:
            start = m.get_waypoint_xodr(src, move['lane'], 25.)
        if start.lane_id != move['lane']:
            # Change one adjacent lane at a time, over 35m per lane.
            old = start.lane_id
            while old != move['lane']:
                new = old + (1 if move['lane'] > old else -1)
                station = start.s
                for n in range(36):
                    a = m.get_waypoint_xodr(src, old, station+n)
                    b = m.get_waypoint_xodr(src, new, station+n)
                    if not a or not b or a.is_junction or b.is_junction:
                        raise ValueError('Lane change leaves approach')
                    if not (a.lane_id*b.lane_id > 0):
                        raise ValueError('Lane change crosses opposing traffic')
                    # In CARLA, negative lane IDs: left is the larger lane ID.
                    allowed = carla.LaneChange.Left if new > old else carla.LaneChange.Right
                    if not (int(a.lane_change) & int(allowed)):
                        raise ValueError('Lane change prohibited at %s/%s/%s' % (src, old, a.s))
                    t = n/35.; blend = 10*t**3-15*t**4+6*t**5
                    pa, pb = a.transform.location, b.transform.location
                    points.append(carla.Location(x=pa.x+(pb.x-pa.x)*blend,
                                                 y=pa.y+(pb.y-pa.y)*blend, z=pa.z))
                changes.append(dict(road=src, from_lane=old, to_lane=new, start_s=station, end_s=station+35.))
                start = b; old = new
        if k+1 < len(selected):
            target_road = selected[k+1]['road']
            target_lane = -move['target_lane']; target_s = 25.
        else:
            target_road = dst; target_lane = move['target_lane']; target_s = 65.
        path = follow(start, target_road, target_lane, target_s)
        crossed = {w.junction_id for w in path if w.is_junction}
        if crossed != {j['id']}:
            raise ValueError('Unexpected junctions in route segment: %s' % crossed)
        points.extend(w.transform.location for w in path)
        start = path[-1]
    first = m.get_waypoint_xodr(movements[0]['road'], movements[0]['lane'], 25.)
    return points, first, dict(junction_order=[v['junction_id'] for v in selected],
                              start=waypoint_record(first), destination=waypoint_record(start),
                              movements=movements, lane_changes=changes,
                              length_m=sum(a.distance(b) for a,b in zip(points,points[1:])))


def run(args):
    folder = Path(args.folder); raw = (folder/'map.xodr').read_bytes()
    output = folder/args.report_subdir if args.report_subdir else folder
    output.mkdir(parents=True,exist_ok=True)
    d = json.loads((folder/'manifest.json').read_text())
    digest = hashlib.sha256(raw).hexdigest()
    if digest != d['opendrive_sha256']:
        raise ValueError('Manifest map hash mismatch')
    offline = carla.Map('acceptance', raw.decode())
    if d['config'].get('type')=='heterogeneous':
        from heterogeneous_route import plan as heterogeneous_plan
        points,start,route=heterogeneous_plan(offline,d,args.start_road,args.start_lane)
    elif d['config'].get('type')=='roundabout':
        from roundabout_route import plan as ring_plan
        points,start,route=ring_plan(offline,d,args.ring_lane)
    else:
        points, start, route = plan(offline, d, args.junctions)
    route['opendrive_sha256'] = digest
    route['points'] = [[p.x, p.y, p.z] for p in points]
    (output/'continuous_route.json').write_text(json.dumps(route, indent=2))
    print('Planned', round(route['length_m']), 'm, junctions', route['junction_order'],
          'lane changes', len(route['lane_changes']), flush=True)
    if args.plan_only:
        return True
    client = carla.Client(args.host, args.port); client.set_timeout(120.)
    world = client.get_world()
    if world.get_actors().filter('vehicle.*') or world.get_settings().synchronous_mode:
        raise RuntimeError('Server contains an active vehicle/session')
    world = client.generate_opendrive_world(raw.decode(), carla.OpendriveGenerationParameters(
        vertex_distance=1., max_road_length=50., wall_height=0., additional_width=0.,
        smooth_junctions=False, enable_mesh_visibility=True, enable_pedestrian_navigation=False))
    m = world.get_map()
    if hashlib.sha256(m.to_opendrive().encode()).hexdigest() != digest:
        raise ValueError('Loaded world differs from planned map')
    world.set_weather(carla.WeatherParameters.ClearNoon)
    if args.beautify=='simple':
        from lane_signals import build as finish_signals
        from network_simple_finish import build as finish_simple
        appearance=dict(mode='simple',signals=finish_signals(world,d),surface=finish_simple(world,d),opendrive_sha256=digest)
        (output/'simple_appearance.json').write_text(json.dumps(appearance,indent=2))
    # Simple/off match the network preview: native sidewalk material throughout.
    # Ring concrete overlays belong exclusively to complete appearance.
    original = world.get_settings(); original.no_rendering_mode = False
    vehicle = None; collision = None; camera = None
    lights = [(l, l.get_state(), l.is_frozen()) for l in world.get_actors().filter('traffic.traffic_light')]
    report = dict(passed=False, opendrive_sha256=digest, route_file='continuous_route.json',
                  required_junctions=args.junctions or [jid for j in d['junctions'] for jid in j.get('opendrive_junction_ids',[j['id']])],
                  expected_junctions=route['junction_order'], visited_junctions=[], collisions=[],
                  road_transitions=[], max_tracking_error_m=0., min_z_m=999., completed=False,
                  signal_policy='all green, isolated single vehicle',
                  physics_enabled=True, teleports=0, respawns=0, spectator='chase', port=args.port,appearance_mode=args.beautify)
    trajectory = []; pending_photos = collections.deque(['start.png'])
    photos = output/'follow_camera'; photos.mkdir(exist_ok=True)
    def photo(image):
        if pending_photos:
            image.save_to_disk(str(photos/pending_photos.popleft()))
    try:
        settings = world.get_settings(); settings.synchronous_mode = True
        settings.fixed_delta_seconds = .05; settings.no_rendering_mode = False
        world.apply_settings(settings)
        for l, _, _ in lights:
            l.set_state(carla.TrafficLightState.Green); l.freeze(True)
        tr = start.transform; tr.location.z += .35
        vehicle = world.spawn_actor(world.get_blueprint_library().find('vehicle.tesla.model3'), tr)
        report['vehicle_id'] = vehicle.id
        collision = world.spawn_actor(world.get_blueprint_library().find('sensor.other.collision'), carla.Transform(), attach_to=vehicle)
        collision.listen(lambda e: report['collisions'].append(dict(frame=e.frame, other=e.other_actor.type_id,
                                                                   impulse=e.normal_impulse.length())))
        bp = world.get_blueprint_library().find('sensor.camera.rgb')
        bp.set_attribute('image_size_x', '960'); bp.set_attribute('image_size_y', '540'); bp.set_attribute('sensor_tick', '.25')
        camera = world.spawn_actor(bp, carla.Transform(carla.Location(x=-8,z=5), carla.Rotation(pitch=-20)), attach_to=vehicle)
        camera.listen(photo)
        for _ in range(15): world.tick()
        cursor = 0; previous_road = None; wall_start = time.monotonic()
        tick_budget=max(14000,int((route['length_m']/3.+30.)/.05))
        report['simulation_budget_seconds']=tick_budget*.05
        for tick in range(tick_budget):
            if not vehicle.is_alive:
                raise RuntimeError('Vehicle disappeared')
            loc = vehicle.get_location(); tr = vehicle.get_transform(); speed = vehicle.get_velocity().length()
            cursor = min(range(cursor, min(cursor+20, len(points))), key=lambda i:loc.distance(points[i]))
            error = loc.distance(points[cursor]); report['max_tracking_error_m'] = max(report['max_tracking_error_m'],error)
            report['min_z_m'] = min(report['min_z_m'], loc.z)
            wp = m.get_waypoint(loc, project_to_road=False, lane_type=carla.LaneType.Driving)
            if wp:
                if wp.road_id != previous_road:
                    report['road_transitions'].append(dict(frame=world.get_snapshot().frame, road=wp.road_id, lane=wp.lane_id, s=wp.s))
                    previous_road = wp.road_id
                if wp.is_junction and (not report['visited_junctions'] or wp.junction_id != report['visited_junctions'][-1]):
                    report['visited_junctions'].append(wp.junction_id)
                    pending_photos.append('visit_%02d_junction_%s.png' % (len(report['visited_junctions']),wp.junction_id))
                    print('Visited', report['visited_junctions'], 'vehicle', vehicle.id, flush=True)
            goal = points[min(cursor+6,len(points)-1)]
            dx,dy = goal.x-loc.x,goal.y-loc.y; yaw = math.radians(tr.rotation.yaw)
            lateral = -dx*math.sin(yaw)+dy*math.cos(yaw)
            steer = math.atan2(2*2.875*lateral,max(dx*dx+dy*dy,.1))/math.radians(70)
            target_speed = 5. if (wp and wp.is_junction) or abs(steer)>.10 else 8.
            vehicle.apply_control(carla.VehicleControl(throttle=max(0.,min(.6,.3*(target_speed-speed))),
                                   brake=max(0.,min(.6,.3*(speed-target_speed))), steer=max(-1.,min(1.,steer))))
            forward = tr.get_forward_vector()
            world.get_spectator().set_transform(carla.Transform(loc-forward*9+carla.Location(z=5),
                                                carla.Rotation(pitch=-20,yaw=tr.rotation.yaw)))
            world.tick()
            if args.realtime:
                time.sleep(max(0., wall_start+(tick+1)*.05-time.monotonic()))
            if tick % 10 == 0:
                trajectory.append(dict(tick=tick,x=loc.x,y=loc.y,z=loc.z,speed=speed,road=wp.road_id if wp else None,
                                       lane=wp.lane_id if wp else None,cursor=cursor))
            if tick % 200 == 0:
                (output/'drive_progress.json').write_text(json.dumps(dict(report,tick=tick,cursor=cursor,position=[loc.x,loc.y,loc.z]),indent=2))
            if cursor >= len(points)-5 and loc.distance(points[-1])<2.5:
                report['completed']=True; break
            if loc.z < -.5 or error > 4. or report['collisions']:
                report['error']='collision, off route, or below surface'; break
            if time.monotonic()-wall_start > 1200:
                report['error']='wall timeout'; break
        report['ticks']=tick+1; report['simulation_seconds']=(tick+1)*.05
        if not report['completed'] and 'error' not in report:
            report['error']='simulation time budget exhausted'
        report['wall_seconds']=time.monotonic()-wall_start
        report['end_distance_m']=vehicle.get_location().distance(points[-1])
        seam_passed = audit_seams(report, route, d)
        report['passed']=(report['completed'] and seam_passed and report['visited_junctions']==route['junction_order']
                          and set(report['required_junctions']) <= set(report['visited_junctions'])
                          and not report['collisions'] and report['min_z_m']>-.5
                          and report['max_tracking_error_m']<2.)
        vehicle.apply_control(carla.VehicleControl(brake=1.,hand_brake=True))
        pending_photos.append('finish.png')
        for _ in range(30): world.tick()
    except Exception as e:
        report['error']=repr(e)
        raise
    finally:
        if collision: collision.stop(); collision.destroy()
        if camera: camera.stop(); camera.destroy()
        if vehicle:
            vehicle.apply_control(carla.VehicleControl(brake=1.,hand_brake=True))
            if not args.keep_vehicle: vehicle.destroy()
        for l,state,frozen in lights: l.set_state(state); l.freeze(frozen)
        world.apply_settings(original)
        report['settings_restored']=True
        (output/'continuous_drive_validation.json').write_text(json.dumps(report,indent=2))
        (output/'continuous_trajectory.json').write_text(json.dumps(trajectory))
    print(json.dumps({k:v for k,v in report.items() if k!='road_transitions'},indent=2),flush=True)
    return report['passed']


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder'); parser.add_argument('--host',default='127.0.0.1')
    parser.add_argument('--port',type=int,default=2000)
    parser.add_argument('--plan-only',action='store_true'); parser.add_argument('--keep-vehicle',action='store_true')
    parser.add_argument('--realtime',action='store_true',help='Pace chase-view playback to simulated time')
    parser.add_argument('--beautify',choices=('simple','off'),default='simple',help='Use the default network simple appearance during driving')
    parser.add_argument('--junctions',type=int,nargs='+',help='Required junction subset for branching networks')
    parser.add_argument('--report-subdir',default='',help='Separate output for each coverage route')
    parser.add_argument('--ring-lane',type=int,choices=(-1,-2),default=-1)
    parser.add_argument('--start-road',type=int,help='Constrain heterogeneous itinerary initial road')
    parser.add_argument('--start-lane',type=int,help='Constrain heterogeneous itinerary first turn lane')
    if not run(parser.parse_args()): raise SystemExit(1)
