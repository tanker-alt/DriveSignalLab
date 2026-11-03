"""Three approach phases, with each approach's rightmost lane always green.

Right-hand-traffic OpenDRIVE maps with independent incoming lane signals only.
The reusable plan/state functions do not import CARLA or advance the world.
"""
import argparse
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

COMPASS = ('west', 'south', 'east', 'north')
DEFAULT_MAP = Path(__file__).resolve().parents[1] / 'Maps/MapDataset/t3_north'


def build_plan(design, junction_id=None):
    junctions = design['junctions']
    if junction_id is None:
        if len(junctions) != 1:
            raise ValueError('Select --junction-id for a network map')
        junction_id = junctions[0]['id']
    matches = [j for j in junctions if j['id'] == junction_id]
    if len(matches) != 1:
        raise ValueError('Unknown or duplicate junction ID')
    arms = matches[0].get('arms', {})
    if len(arms) != 3 or not set(arms) < set(COMPASS) or len(set(arms.values())) != 3:
        raise ValueError('Expected three distinct compass arms and roads')
    missing = next(a for a in COMPASS if a not in arms)
    # Rotate the example west -> south -> east when the missing arm rotates.
    offset = COMPASS.index(missing)
    order = [COMPASS[(offset + i) % 4] for i in (1, 2, 3)]
    signals = {str(sid): cfg for sid, cfg in design['signals'].items()
               if cfg['junction_id'] == junction_id}
    movements = [m for m in design['movements'] if m['junction_id'] == junction_id]
    covered = set()
    always = []
    phases = []
    for arm in order:
        road = arms[arm]
        lanes = {}
        for sid, cfg in signals.items():
            if cfg['road_id'] != road:
                continue
            incoming = cfg['incoming_lanes']
            if len(incoming) != 1 or type(incoming[0]) is not int or incoming[0] == 0:
                raise ValueError('Independent nonzero incoming lane signals required')
            lane = incoming[0]
            if lane in lanes:
                raise ValueError('Duplicate incoming lane signal')
            lanes[lane] = sid
            covered.add((road, lane))
        if len(lanes) < 2 or len({lane > 0 for lane in lanes}) != 1:
            raise ValueError('Each approach needs at least two lanes on the same side')
        # For right-hand traffic both +/- sides increase outward in |lane ID|.
        rightmost = max(lanes, key=abs)
        always.append(lanes[rightmost])
        phases.append(dict(arm=arm, controlled_signal_ids=sorted(
            sid for lane, sid in lanes.items() if lane != rightmost)))
    if covered != {(m['road'], m['lane']) for m in movements}:
        raise ValueError('Movement lanes and incoming signal coverage differ')
    if len(covered) != len(signals):
        raise ValueError('Signals outside the selected arms')
    if any(m['target_road'] not in arms.values() for m in movements):
        raise ValueError('Movement points outside the selected junction')
    return dict(junction_id=junction_id, missing_arm=missing, phases=phases,
                always_green_signal_ids=always, signals=signals,
                opendrive_sha256=design['opendrive_sha256'])


def expected_states(plan, phase_index, stage):
    if stage not in ('GREEN', 'YELLOW', 'MAIN_RED', 'ALL_RED'):
        raise ValueError('Unknown stage')
    states = {sid: 'Red' for sid in plan['signals']}
    if stage == 'ALL_RED':
        return states
    for sid in plan['always_green_signal_ids']:
        states[sid] = 'Green'
    if stage in ('GREEN', 'YELLOW'):
        for sid in plan['phases'][phase_index]['controlled_signal_ids']:
            states[sid] = stage.title()
    return states


def bind(world, plan):
    actual = world.get_map().to_opendrive()
    try:
        def canon(raw):
            root = ET.fromstring(raw)
            def norm(e):
                e.text = e.tail = None
                e.attrib = dict(sorted(e.attrib.items()))
                for child in e: norm(child)
            norm(root)
            return ET.tostring(root, encoding='utf-8')
        digest = hashlib.sha256(canon(actual)).hexdigest()
        source = plan.get('_source_opendrive')
        expected = hashlib.sha256(canon(source)).hexdigest() if source else plan['opendrive_sha256']
    except Exception:
        digest = hashlib.sha256(actual.encode('utf-8')).hexdigest()
        expected = plan['opendrive_sha256']
    # CARLA rewrites OpenDRIVE on runtime generation; signal/lanes below are
    # authoritative for this single-junction controller.
    actors = world.get_traffic_lights_in_junction(plan['junction_id'])
    lights = {str(light.get_opendrive_id()): light for light in actors}
    if len(lights) != len(actors) or set(lights) != set(plan['signals']):
        raise ValueError('Native signal IDs differ')
    for sid, light in lights.items():
        cfg = plan['signals'][sid]
        actual = {(w.road_id, w.lane_id) for w in light.get_affected_lane_waypoints()}
        if actual != {(cfg['road_id'], lane) for lane in cfg['incoming_lanes']}:
            raise ValueError('Native lane validity differs: ' + sid)
        if light.is_frozen():
            raise ValueError('Target lights are frozen; release the previous controller first')
    return lights


def run(args, plan, sample_callback=None, client=None):
    import carla
    from signal_runtime import hold_target_lights, restore_target_lights
    client = client if client is not None else carla.Client(args.host, args.port)
    client.set_timeout(15)
    world = client.get_world()
    world_id = world.id
    synchronous = world.get_settings().synchronous_mode
    if args.tick and not synchronous:
        raise ValueError('--tick requires an already synchronous world')
    # A new client has no actor snapshot until it receives a simulation frame.
    if args.tick:
        world.tick()
    else:
        world.wait_for_tick(10)
    lights = bind(world, plan)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    saved = []
    map_obj = world.get_map()
    controlled_lanes = {(cfg['road_id'], lane) for sid, cfg in plan['signals'].items()
                        if sid not in plan['always_green_signal_ids'] for lane in cfg['incoming_lanes']}
    free_lanes = {(cfg['road_id'], lane) for sid, cfg in plan['signals'].items()
                 if sid in plan['always_green_signal_ids'] for lane in cfg['incoming_lanes']}
    controlled_vehicles, free_vehicles = set(), set()

    def clearance_vehicles(snapshot):
        occupied = []
        live = set()
        for vehicle in world.get_actors().filter('vehicle.*'):
            state = snapshot.find(vehicle.id)
            if state is None:
                continue
            live.add(vehicle.id)
            transform = state.get_transform()
            wp = map_obj.get_waypoint(transform.location)
            lane = (wp.road_id, wp.lane_id)
            if lane in controlled_lanes:
                controlled_vehicles.add(vehicle.id)
                free_vehicles.discard(vehicle.id)
            elif lane in free_lanes:
                free_vehicles.add(vehicle.id)
                controlled_vehicles.discard(vehicle.id)
            elif wp.is_junction and wp.junction_id == plan['junction_id'] and vehicle.id not in free_vehicles:
                controlled_vehicles.add(vehicle.id)
            if vehicle.id not in controlled_vehicles:
                continue
            # Include front and rear bumper: center-only checks release long trucks too early.
            box = vehicle.bounding_box
            points = [transform.location]
            for sign in (-1, 1):
                points.append(transform.transform(carla.Location(
                    x=box.location.x + sign * (box.extent.x + .5),
                    y=box.location.y, z=box.location.z)))
            if any((lambda p: p.is_junction and p.junction_id == plan['junction_id'])(map_obj.get_waypoint(point)) for point in points):
                occupied.append(vehicle.id)
        controlled_vehicles.intersection_update(live)
        free_vehicles.intersection_update(live)
        return sorted(occupied)

    def apply(states):
        # Red writes precede any new green writes. No global freeze/TM reset.
        for color in ('Red', 'Yellow', 'Green'):
            for sid, state in states.items():
                if state == color:
                    lights[sid].set_state(getattr(carla.TrafficLightState, state))
        client.apply_batch_sync([], False)

    try:
        with output.open('w', encoding='utf-8') as log:
            log.write(json.dumps(dict(event='plan', plan=plan)) + '\n')

            def interval(cycle, index, stage, duration):
                states = expected_states(plan, index, stage)
                apply(states)
                start = None
                pending = 0
                while True:
                    if client.get_world().id != world_id:
                        raise RuntimeError('World changed')
                    if args.tick:
                        world.tick()
                        snapshot = world.get_snapshot()
                    else:
                        snapshot = world.wait_for_tick(10)
                    actual = {sid: str(light.get_state()) for sid, light in lights.items()}
                    matched = states == actual
                    now = snapshot.timestamp.elapsed_seconds
                    occupied = clearance_vehicles(snapshot)
                    if matched and start is None:
                        start = now
                    log.write(json.dumps(dict(frame=snapshot.frame, sim_time=now,
                        cycle=cycle, arm=plan['phases'][index]['arm'],
                        stage=stage if start is not None else 'APPLYING_' + stage,
                        expected=states, actual=actual, clearance_vehicle_ids=occupied)) + '\n')
                    log.flush()
                    if sample_callback:
                        sample_callback(snapshot, index, stage, states, actual)
                    if not matched:
                        pending += 1
                        if start is not None or pending > 10:
                            raise RuntimeError('Signal readback mismatch; see ' + str(output))
                    elif now - start >= duration:
                        if stage in ('ALL_RED', 'MAIN_RED') and occupied:
                            if now - start > duration + getattr(args, 'max_clearance_wait', 60.):
                                raise RuntimeError('Clearance timeout; occupied vehicles: ' + str(occupied))
                        else:
                            return

            hold_target_lights(list(lights.values()), saved)
            interval(0, 0, 'ALL_RED', args.clearance_seconds)
            cycle = 0
            while not args.cycles or cycle < args.cycles:
                cycle += 1
                for index in range(3):
                    interval(cycle, index, 'GREEN', args.green_seconds)
                    interval(cycle, index, 'YELLOW', args.yellow_seconds)
                    interval(cycle, index, 'MAIN_RED', args.clearance_seconds)
    except KeyboardInterrupt:
        print('Stopped by user')
    finally:
        if client.get_world().id == world_id:
            restore_target_lights(saved)
            client.apply_batch_sync([], False)
        print('Signal verification log:', output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-dir', type=Path, default=DEFAULT_MAP)
    parser.add_argument('--junction-id', type=int)
    parser.add_argument('--plan-only', action='store_true', help='Inspect the plan without CARLA')
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--tick', action='store_true', help='Be the sole tick owner in an existing synchronous world')
    parser.add_argument('--cycles', type=int, default=0)
    parser.add_argument('--green-seconds', type=float, default=12)
    parser.add_argument('--yellow-seconds', type=float, default=3)
    parser.add_argument('--clearance-seconds', type=float, default=2)
    parser.add_argument('--max-clearance-wait', type=float, default=60,
                        help='Maximum additional wait for controlled vehicle tails to clear')
    parser.add_argument('--output', default='LocalArtifacts/t_phase_control/signals.jsonl')
    args = parser.parse_args()
    if args.cycles < 0 or any(not math.isfinite(v) or v <= 0 for v in
                             (args.green_seconds, args.yellow_seconds, args.clearance_seconds, args.max_clearance_wait)):
        parser.error('Durations must be finite and positive; cycles must be nonnegative')
    design = json.loads((args.map_dir / 'manifest.json').read_text(encoding='utf-8-sig'))
    if hashlib.sha256((args.map_dir / 'map.xodr').read_bytes()).hexdigest() != design['opendrive_sha256']:
        raise ValueError('Map file fingerprint differs from manifest')
    plan = build_plan(design, args.junction_id)
    print(json.dumps(plan, indent=2))
    if not args.plan_only:
        run(args, plan)


if __name__ == '__main__':
    main()
