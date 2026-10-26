"""Four protected phases for junction_toolkit maps (CARLA Python 3.7).

prepare creates a separate map with one native signal per dedicated lane.
run controls the already loaded map; --load explicitly loads the prepared map.
"""
import argparse
import copy
import csv
import hashlib
import json
import math
import queue
import signal
import sys
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DEFAULT_MAP = ROOT.parent / 'Maps/MapDataset/maps/cross3'
PHASES = ('NS_STRAIGHT', 'NS_LEFT', 'EW_STRAIGHT', 'EW_LEFT')


def normalize_design(design):
    """Derive movement groups from native lane signals without changing the map."""
    design = copy.deepcopy(design)
    if len(design['junctions']) != 1:
        raise ValueError('Four-phase controller requires a single junction')
    junction = design['junctions'][0]
    arms = junction.get('arms')
    if arms is None and design.get('four_phase'):
        arms = dict(zip(('west', 'south', 'east', 'north'), junction['roads']))
    if not arms or set(arms) != {'west', 'south', 'east', 'north'}:
        raise ValueError('Explicit four compass arms required')
    road_arms = {road: arm for arm, road in arms.items()}
    covered = set()
    for cfg in design['signals'].values():
        if len(cfg['incoming_lanes']) != 1 or cfg['road_id'] not in road_arms:
            raise ValueError('Independent per-lane signals required; prepare the old map first')
        lane = cfg['incoming_lanes'][0]
        key = (cfg['road_id'], lane)
        if key in covered or cfg['junction_id'] != junction['id']:
            raise ValueError('Duplicate lane signal or wrong junction')
        covered.add(key)
        moves = {m['movement'] for m in design['movements'] if (m['road'], m['lane']) == key}
        if len(moves) != 1 or not moves <= {'left', 'straight', 'right'}:
            raise ValueError('Dedicated turn lanes required')
        cfg.update(arm=road_arms[cfg['road_id']], movement=next(iter(moves)))
    if covered != {(m['road'], m['lane']) for m in design['movements']}:
        raise ValueError('Movement lanes and signal coverage differ')
    for arm in arms:
        if not {'left', 'straight'} <= {c['movement'] for c in design['signals'].values() if c['arm'] == arm}:
            raise ValueError('Each arm requires left and straight movements')
    return design


def expected_states(design, phase, stage, right_turn='always-green'):
    active = active_ids(design, phase, right_turn) if phase else set()
    return {sid: ('Green' if right_turn == 'always-green' and cfg['movement'] == 'right'
                  else stage.title() if sid in active and stage in ('GREEN', 'YELLOW') else 'Red')
            for sid, cfg in design['signals'].items()}


def prepare(source, destination):
    source, destination = Path(source), Path(destination)
    design = json.loads((source / 'manifest.json').read_text(encoding='utf-8'))
    raw = (source / 'map.xodr').read_bytes()
    if hashlib.sha256(raw).hexdigest() != design['opendrive_sha256']:
        raise ValueError('Source map fingerprint mismatch')
    if design.get('signal_policy') == 'one_signal_per_incoming_lane' or design.get('four_phase'):
        design = normalize_design(design)
        if destination.exists() and any(destination.iterdir()):
            raise ValueError('Output directory must be empty')
        destination.mkdir(parents=True, exist_ok=True)
        design['source_map'] = str(source.resolve())
        (destination / 'map.xodr').write_bytes(raw)
        (destination / 'manifest.json').write_text(json.dumps(design, indent=2), encoding='utf-8')
        print('Copied existing independent signals:', destination)
        return design
    if len(design['junctions']) != 1 or len(design['junctions'][0]['roads']) != 4:
        raise ValueError('This plan requires one four-arm junction')
    xml = ET.fromstring(raw)
    configs = {}
    junction = design['junctions'][0]
    arms = junction.get('arms') or dict(zip(('west', 'south', 'east', 'north'), junction['roads']))
    if set(arms) != {'west', 'south', 'east', 'north'}:
        raise ValueError('Four compass arms required')
    road_arms = {v: k for k, v in arms.items()}
    for node in list(xml.findall('controller')):
        xml.remove(node)
    jnode = xml.find("./junction[@id='{}']".format(junction['id']))
    for node in list(jnode.findall('controller')):
        jnode.remove(node)
    for old_id, cfg in design['signals'].items():
        road = xml.find("./road[@id='{}']".format(cfg['road_id']))
        signals = road.find('signals')
        template = signals.find("signal[@id='{}']".format(old_id))
        if template is None or len(signals.findall('signal')) != 1:
            raise ValueError('Expected original toolkit approach signal')
        signals.remove(template)
        for lane in cfg['incoming_lanes']:
            moves = {m['movement'] for m in design['movements']
                     if m['road'] == cfg['road_id'] and m['lane'] == lane}
            if len(moves) != 1 or not moves <= {'left', 'straight', 'right'}:
                raise ValueError('Dedicated turn lanes required: road {}, lane {}'.format(cfg['road_id'], lane))
            sid = str(100000 + len(configs))
            lamp = copy.deepcopy(template)
            lamp.set('id', sid)
            lamp.set('name', 'FourPhase_' + sid)
            # Place each head over its lane, with lane-specific native validity.
            lamp.set('t', str((lane + .5) * design['config']['lane_width_m']))
            lamp.set('zOffset', '4')
            validity = lamp.find('validity')
            validity.set('fromLane', str(lane))
            validity.set('toLane', str(lane))
            signals.append(lamp)
            controller = ET.Element('controller', id=sid, name='FourPhase_' + sid, sequence='0')
            ET.SubElement(controller, 'control', signalId=sid, type='0')
            xml.insert(list(xml).index(jnode), controller)
            ET.SubElement(jnode, 'controller', id=sid, type='0', sequence='0')
            configs[sid] = dict(cfg, incoming_lanes=[lane], phase_slot=len(configs),
                                arm=road_arms[cfg['road_id']], movement=next(iter(moves)))
    for arm in arms:
        if not {'left', 'straight'} <= {c['movement'] for c in configs.values() if c['arm'] == arm}:
            raise ValueError('Each approach requires dedicated left and straight lanes')
    data = b'<?xml version="1.0" encoding="utf-8"?>\n' + ET.tostring(xml, encoding='utf-8')
    if destination.exists() and any(destination.iterdir()):
        raise ValueError('Output directory must be empty')
    destination.mkdir(parents=True, exist_ok=True)
    (destination / 'map.xodr').write_bytes(data)
    design.update(signals=configs, opendrive_sha256=hashlib.sha256(data).hexdigest(),
                  status='four_phase_requires_runtime_validation', four_phase=True,
                  source_map=str(source.resolve()))
    (destination / 'manifest.json').write_text(json.dumps(design, indent=2), encoding='utf-8')
    print('Prepared {} ({} independent lane signals)'.format(destination, len(configs)))
    return design


def active_ids(design, phase, right_turn):
    axis, movement = phase.split('_')
    arms = ('north', 'south') if axis == 'NS' else ('east', 'west')
    moves = {movement.lower()}
    if movement == 'STRAIGHT' and right_turn == 'with-straight':
        moves.add('right')
    return {sid for sid, cfg in design['signals'].items()
            if (cfg['arm'] in arms and cfg['movement'] in moves)
            or (right_turn == 'always-green' and cfg['movement'] == 'right')}


def bind(world, design):
    if hashlib.sha256(world.get_map().to_opendrive().encode('utf-8')).hexdigest() != design['opendrive_sha256']:
        raise ValueError('Loaded map differs; use run --load for the prepared map')
    lights = {str(l.get_opendrive_id()): l for l in
              world.get_traffic_lights_in_junction(design['junctions'][0]['id'])}
    if set(lights) != set(design['signals']):
        raise ValueError('Native signal IDs differ from prepared manifest')
    for sid, light in lights.items():
        cfg = design['signals'][sid]
        actual = {(w.road_id, w.lane_id) for w in light.get_affected_lane_waypoints()}
        if actual != {(cfg['road_id'], lane) for lane in cfg['incoming_lanes']}:
            raise ValueError('Native lane validity mismatch: {} {}'.format(sid, actual))
    return lights


def run(args, sample_callback=None):
    import carla
    from control_junction_phases import hold_target_lights, restore_target_lights
    folder = Path(args.map_dir)
    design = normalize_design(json.loads((folder / 'manifest.json').read_text(encoding='utf-8')))
    client = carla.Client(args.host, args.port)
    client.set_timeout(30)
    world = client.get_world()
    if args.load:
        if world.get_settings().synchronous_mode:
            raise RuntimeError('Finish the synchronous experiment before loading')
        world.wait_for_tick(10)
        if any(world.get_actors().filter(p) for p in ('vehicle.*', 'sensor.*', 'walker.*')):
            raise RuntimeError('Finish the active experiment before loading')
        raw = (folder / 'map.xodr').read_bytes().decode('utf-8')
        if hashlib.sha256(raw.encode('utf-8')).hexdigest() != design['opendrive_sha256']:
            raise ValueError('Prepared map fingerprint mismatch')
        client.set_timeout(120)
        world = client.generate_opendrive_world(raw, carla.OpendriveGenerationParameters(
            vertex_distance=2., max_road_length=50., wall_height=0., additional_width=0.,
            smooth_junctions=False, enable_mesh_visibility=True, enable_pedestrian_navigation=False))
        world.wait_for_tick(15)
        world.get_spectator().set_transform(carla.Transform(
            carla.Location(x=0, y=0, z=100), carla.Rotation(pitch=-90, yaw=-90)))
    if not world.get_settings().synchronous_mode:
        world.wait_for_tick(10)
    elif args.tick:
        world.tick()
        if sample_callback:
            sample_callback(world.get_snapshot(), 0, '', 'SETUP')
    lights = bind(world, design)
    print('Verified {} native lane signals; phases: {}'.format(len(lights), ' -> '.join(PHASES)))
    if args.check_only:
        return
    if any(l.is_frozen() for l in lights.values()):
        raise RuntimeError('Target signals are frozen; release the previous controller first')
    if args.tick and not world.get_settings().synchronous_mode:
        raise ValueError('--tick requires an already synchronous world')
    output = Path(args.output) if args.output else ROOT / 'evaluation/outputs/run_logs' / (
        'four_phases_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.csv')
    output.parent.mkdir(parents=True, exist_ok=True)
    saved = []
    camera = None
    frames = queue.Queue(maxsize=4)
    capture = Path(args.capture_dir) if args.capture_dir else None
    previous_handler = signal.getsignal(signal.SIGINT)
    try:
        if args.visualize or capture:
            view = carla.Transform(carla.Location(x=0, y=0, z=95),
                                   carla.Rotation(pitch=-90, yaw=-90))
            world.get_spectator().set_transform(view)
        if capture:
            capture.mkdir(parents=True, exist_ok=True)
            bp = world.get_blueprint_library().find('sensor.camera.rgb')
            bp.set_attribute('image_size_x', '1200')
            bp.set_attribute('image_size_y', '1000')
            bp.set_attribute('fov', '80')
            camera = world.spawn_actor(bp, view)
            def receive(frame):
                try:
                    frames.put_nowait(frame)
                except queue.Full:
                    pass
            camera.listen(receive)
        with output.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(['frame', 'sim_time', 'cycle', 'phase', 'stage', 'expected', 'actual'])

            def interval(cycle, phase, stage, duration):
                if stage == 'ALL_RED' and args.right_turn == 'always-green':
                    stage = 'MAIN_RED'
                expected = expected_states(design, phase, stage, args.right_turn)
                active = {sid for sid, state in expected.items() if state != 'Red'}
                # Stop all previous movements before granting any new movement.
                for sid, light in lights.items():
                    if sid not in active:
                        light.set_state(carla.TrafficLightState.Red)
                for sid in active:
                    lights[sid].set_state(getattr(carla.TrafficLightState, expected[sid]))
                client.apply_batch_sync([], False)
                # Query the server after writes; post-tick verification below
                # remains authoritative (this query alone is not an acknowledgement).
                world.get_settings()
                start = world.get_snapshot().timestamp.elapsed_seconds
                if args.visualize:
                    colors = dict(Red=carla.Color(255, 30, 30),
                                  Green=carla.Color(20, 255, 40), Yellow=carla.Color(255, 220, 0))
                    for sid, cfg in design['signals'].items():
                        wp = world.get_map().get_waypoint_xodr(cfg['road_id'], cfg['incoming_lanes'][0], cfg['reference_s'])
                        loc = wp.transform.location + carla.Location(z=1)
                        world.debug.draw_point(loc, size=.65, color=colors[expected[sid]], life_time=duration)
                        world.debug.draw_string(loc + carla.Location(z=1), cfg['movement'],
                                                color=colors[expected[sid]], life_time=duration)
                    world.debug.draw_string(carla.Location(z=2), '{} {}'.format(phase, stage),
                                            color=carla.Color(255,255,255), life_time=duration)
                print('cycle={} phase={} stage={} seconds={}'.format(cycle, phase, stage, duration), flush=True)
                now = start
                captured = False
                settled = False
                settle_frames = 0
                while not settled or now - start < duration:
                    if args.tick:
                        world.tick()
                        snapshot = world.get_snapshot()
                    else:
                        snapshot = world.wait_for_tick(10)
                    now = snapshot.timestamp.elapsed_seconds
                    actual = {sid: str(light.get_state()) for sid, light in lights.items()}
                    matches = actual == expected
                    pending = not settled and not matches
                    writer.writerow([snapshot.frame, now, cycle, phase, 'APPLYING_' + stage if pending else stage,
                                     json.dumps(expected, sort_keys=True), json.dumps(actual, sort_keys=True)])
                    stream.flush()
                    if pending:
                        settle_frames += 1
                        if settle_frames > 10:
                            raise RuntimeError('Signal command did not converge within 10 frames; see ' + str(output))
                        if sample_callback:
                            sample_callback(snapshot, cycle, phase, 'APPLYING_' + stage)
                        continue
                    if not matches:
                        raise RuntimeError('Signal state mismatch; see ' + str(output))
                    if not settled:
                        settled = True
                        start = now
                    if sample_callback:
                        sample_callback(snapshot, cycle, phase, stage)
                    if capture and stage == 'GREEN' and not captured and now - start >= .3:
                        # Only accept an image rendered after this green began.
                        while True:
                            frame = frames.get(timeout=10)
                            if frame.timestamp >= start + .15:
                                frame.save_to_disk(str(capture / ('{}_{}.png'.format(cycle, phase))))
                                captured = True
                                break

            hold_target_lights(list(lights.values()), saved)
            interval(0, '', 'ALL_RED', args.all_red_seconds)
            cycle = 0
            while args.cycles == 0 or cycle < args.cycles:
                cycle += 1
                for phase in PHASES:
                    duration = args.left_seconds if phase.endswith('LEFT') else args.straight_seconds
                    interval(cycle, phase, 'GREEN', duration)
                    interval(cycle, phase, 'YELLOW', args.yellow_seconds)
                    interval(cycle, '', 'ALL_RED', args.all_red_seconds)
    except KeyboardInterrupt:
        print('Stopped by user')
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if camera is not None:
            camera.stop()
            camera.destroy()
        restore_target_lights(saved)
        signal.signal(signal.SIGINT, previous_handler)
        print('Phase verification log:', output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    p = sub.add_parser('prepare')
    p.add_argument('--source', default=str(DEFAULT_MAP))
    p.add_argument('--out', default=str(DEFAULT_MAP.with_name('cross3_four_phase')))
    p = sub.add_parser('run')
    p.add_argument('--map-dir', default=str(DEFAULT_MAP))
    p.add_argument('--host', default='localhost')
    p.add_argument('--port', type=int, default=2000)
    p.add_argument('--load', action='store_true')
    p.add_argument('--check-only', action='store_true')
    p.add_argument('--tick', action='store_true', help='Be the sole tick owner in an existing synchronous world')
    p.add_argument('--cycles', type=int, default=0)
    p.add_argument('--straight-seconds', type=float, default=20)
    p.add_argument('--left-seconds', type=float, default=15)
    p.add_argument('--yellow-seconds', type=float, default=3)
    p.add_argument('--all-red-seconds', type=float, default=2)
    p.add_argument('--right-turn', choices=['always-green', 'with-straight', 'red'], default='always-green')
    p.add_argument('--output')
    p.add_argument('--visualize', action='store_true', help='Overhead view and debug lane-state markers')
    p.add_argument('--capture-dir', help='Save a native RGB camera image for each green phase')
    args = parser.parse_args()
    if args.command == 'prepare':
        prepare(args.source, args.out)
    else:
        for key in ('straight_seconds', 'left_seconds', 'yellow_seconds', 'all_red_seconds'):
            if not math.isfinite(getattr(args, key)) or getattr(args, key) <= 0:
                parser.error(key + ' must be finite and positive')
        if args.cycles < 0:
            parser.error('cycles must be nonnegative')
        run(args)


if __name__ == '__main__':
    main()
