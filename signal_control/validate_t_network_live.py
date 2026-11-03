"""Mixed-fleet, local-junction regression on the existing t_3x3 CARLA map.

Runs one junction at a time; this is not a network coordination benchmark.
Vehicles arrive in waves and are removed after clearing the selected exit.
"""
import argparse
import datetime
import hashlib
import json
import random
import time
from pathlib import Path

import carla
from control_t_phases import build_plan, bind, run

MAP = Path(__file__).resolve().parents[1] / 'Maps/MapDataset/t_3x3'
FLEET = ['vehicle.tesla.model3', 'vehicle.nissan.patrol',
         'vehicle.mercedes.sprinter', 'vehicle.mitsubishi.fusorosa',
         'vehicle.carlamotors.carlacola', 'vehicle.audi.a2']
FLOWS = {'low': (1, 12.), 'medium': (3, 6.), 'high': (6, 2.)}


def case(design, junction_id, flow, seed, folder, client, tm,
         map_dir=None, rendered=False, random_fleet=False, speed_difference=30):
    folder.mkdir(parents=True)
    plan = build_plan(design, junction_id)
    plan['_source_opendrive'] = ((map_dir or MAP) / 'map.xodr').read_text(encoding='utf-8')
    client.set_timeout(30)
    world = client.get_world()
    world.wait_for_tick(10)
    original = world.get_settings()
    if original.synchronous_mode or any(world.get_actors().filter(p) for p in ('vehicle.*', 'sensor.*', 'walker.*')):
        raise RuntimeError('Active experiment')
    lights = bind(world, plan)
    saved_signals = {sid: (str(l.get_state()), l.get_green_time(), l.get_yellow_time(), l.get_red_time()) for sid, l in lights.items()}
    rng = random.Random(seed)
    count, headway = FLOWS[flow]
    records, collisions, active, pending = [], [], {}, []
    report = dict(passed=False, junction_id=junction_id, missing_arm=plan['missing_arm'],
                  flow=flow, seed=seed, requested=count * len(plan['signals']), vehicles=records,
                  collision_callbacks=collisions, frames=0, mismatch_frames=0,
                  always_green_failures=0, green_arms=[], stopped_signals=[],
                  nominal_wave_headway_seconds=headway, spawn_distance_m=55, following_gap_m=3.,
                  fleet=FLEET, opendrive_sha256=design['opendrive_sha256'])
    report['saved_signals'] = saved_signals
    report.update(map_dir=str(map_dir or MAP), rendered=rendered,
                  random_fleet=random_fleet, speed_difference_percent=speed_difference)
    (folder / 'report.json').write_text(json.dumps(report, indent=2))
    junction = next(j for j in design['junctions'] if j['id'] == junction_id)
    lengths = dict(zip(junction['roads'], junction['lengths']))
    movements = {(m['road'], m['lane']): m for m in design['movements'] if m['junction_id'] == junction_id}
    map_obj = world.get_map()
    library = world.get_blueprint_library()
    for wave in range(count):
        for i, (sid, cfg) in enumerate(sorted(plan['signals'].items())):
            pending.append(dict(signal=sid, blueprint=rng.choice(FLEET) if random_fleet else FLEET[(i + wave + seed) % len(FLEET)],
                                due=2 + wave * headway + rng.uniform(0, .5), wave=wave))
    started, last_spawn = None, -1.
    wall_start = time.monotonic()
    output = (folder / 'vehicles.jsonl').open('w')
    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = .05
        settings.no_rendering_mode = not rendered
        world.apply_settings(settings)
        tm.set_synchronous_mode(True)
        tm.set_random_device_seed(seed)
        world.tick()
        center=junction['center']
        view=carla.Transform(carla.Location(center[0],-center[1],130),carla.Rotation(pitch=-90,yaw=-90))
        last_wall=[time.monotonic()]

        def sample(snapshot, index, stage, expected, actual):
            nonlocal started, last_spawn
            if rendered:
                world.get_spectator().set_transform(view)
                time.sleep(max(0., .05-(time.monotonic()-last_wall[0])))
                last_wall[0]=time.monotonic()
            if time.monotonic() - wall_start > 600:
                raise RuntimeError('Case exceeded 600 wall seconds')
            now = snapshot.timestamp.elapsed_seconds
            if started is None:
                started = now
            elapsed = now - started
            report['frames'] += 1
            report['sim_seconds'] = elapsed
            if actual != expected:
                report['mismatch_frames'] += 1
            if stage != 'ALL_RED' and actual == expected:
                if any(actual[s] != 'Green' for s in plan['always_green_signal_ids']):
                    report['always_green_failures'] += 1
                if stage == 'GREEN' and plan['phases'][index]['arm'] not in report['green_arms']:
                    report['green_arms'].append(plan['phases'][index]['arm'])
            if elapsed - last_spawn >= .25:
                last_spawn = elapsed
                blocked = set()
                for request in list(pending):
                    sid = request['signal']
                    if request['due'] > elapsed or sid in blocked:
                        continue
                    blocked.add(sid)
                    cfg = plan['signals'][sid]
                    wp = map_obj.get_waypoint_xodr(cfg['road_id'], cfg['incoming_lanes'][0], cfg['reference_s'] - 55)
                    # Keep new vehicles clear of existing bodies (including long trucks).
                    if any(not r['completed'] and v.get_location().distance(wp.transform.location) < 16 for v, s, r in active.values()):
                        continue
                    transform = wp.transform
                    transform.location.z += .5
                    vehicle = world.try_spawn_actor(library.find(request['blueprint']), transform)
                    if vehicle is None:
                        continue
                    rec = dict(request, actor_id=vehicle.id, spawned_at=elapsed, completed=False,
                               stopped_on_red=False, red_entry=False, wrong_exit=False,
                               entered=False, signal_crossed=False, red_signal_crossing=False, min_z=999.)
                    records.append(rec)
                    # Register immediately so setup failure still cleans up the vehicle.
                    active[vehicle.id] = (vehicle, None, rec)
                    sensor = world.spawn_actor(library.find('sensor.other.collision'), carla.Transform(), attach_to=vehicle)
                    active[vehicle.id] = (vehicle, sensor, rec)
                    sensor.listen(lambda e: collisions.append(dict(frame=e.frame, actor=e.actor.id, other=e.other_actor.id)))
                    vehicle.set_autopilot(True, 8010)
                    tm.auto_lane_change(vehicle, False)
                    tm.vehicle_percentage_speed_difference(vehicle, speed_difference)
                    tm.distance_to_leading_vehicle(vehicle, 3.)
                    tm.ignore_lights_percentage(vehicle, 0)
                    move = movements[(cfg['road_id'], cfg['incoming_lanes'][0])]
                    tm.set_route(vehicle, [move['movement'].title()])
                    pending.remove(request)
            for actor_id, (vehicle, sensor, rec) in list(active.items()):
                if rec['completed']:
                    continue
                state = snapshot.find(actor_id)
                if state is None:
                    if elapsed - rec['spawned_at'] > 1:
                        rec['disappeared'] = True
                        raise RuntimeError('Vehicle disappeared: ' + str(actor_id))
                    continue  # Spawned after this frame's snapshot.
                loc = state.get_transform().location
                wp = map_obj.get_waypoint(loc)
                speed = state.get_velocity().length()
                if wp.is_junction and speed < .15 and report['frames'] % 100 == 0:
                    rec['stopped_actions'] = [(str(a), p.road_id, p.lane_id, p.s) for a, p in tm.get_all_actions(vehicle)]
                rec['min_z'] = min(rec['min_z'], loc.z)
                cfg = plan['signals'][rec['signal']]
                move = movements[(cfg['road_id'], cfg['incoming_lanes'][0])]
                # The native signal section precedes the junction boundary.
                # A vehicle that crossed on yellow may legitimately enter on red.
                crossed = ((wp.road_id == cfg['road_id'] and wp.lane_id == cfg['incoming_lanes'][0]
                            and wp.s >= cfg['signal_s']) or (wp.is_junction and wp.junction_id == junction_id))
                if crossed and not rec['signal_crossed']:
                    rec['signal_crossed'] = True
                    rec['signal_crossing_time'] = elapsed
                    rec['signal_crossing_light'] = actual[rec['signal']]
                    rec['red_signal_crossing'] = actual[rec['signal']] == 'Red'
                if wp.is_junction and wp.junction_id == junction_id:
                    if not rec['entered']:
                        rec['entry_time'] = elapsed
                        rec['entry_light'] = actual[rec['signal']]
                        rec['red_entry'] = actual[rec['signal']] == 'Red'
                    rec['entered'] = True
                if not rec['entered'] and wp.road_id == cfg['road_id'] and wp.lane_id == cfg['incoming_lanes'][0]:
                    if actual[rec['signal']] == 'Red' and speed < .15 and abs(wp.s - cfg['reference_s']) < 20:
                        rec['stopped_on_red'] = True
                        if rec['signal'] not in report['stopped_signals']:
                            report['stopped_signals'].append(rec['signal'])
                if rec['entered'] and not wp.is_junction:
                    if (wp.road_id, wp.lane_id) != (move['target_road'], move['target_lane']):
                        rec['wrong_exit'] = True
                    if (wp.road_id, wp.lane_id) == (move['target_road'], move['target_lane']) and wp.s < lengths[move['target_road']] - 30:
                        rec['completed'] = True
                        rec['completed_at'] = elapsed
                output.write(json.dumps(dict(frame=snapshot.frame, time=elapsed, actor=actor_id,
                    road=wp.road_id, lane=wp.lane_id, s=wp.s, speed=speed, z=loc.z,
                    light=actual[rec['signal']], phase=index, stage=stage,
                    throttle=vehicle.get_control().throttle, brake=vehicle.get_control().brake,
                    native_signal=vehicle.get_traffic_light().get_opendrive_id() if vehicle.get_traffic_light() else None)) + '\n')
                if rec['completed']:
                    vehicle.set_autopilot(False, 8010)
                    sensor.stop()
                    vehicle.set_simulate_physics(False)
                    vehicle.set_transform(carla.Transform(carla.Location(x=2000 + len(records) * 20 + actor_id % 1000, y=2000, z=20)))
            if not pending and records and all(r['completed'] for r in records):
                # Finish this entire three-phase cycle, then stop normally.
                args.cycles = 1
            if report['frames'] % 100 == 0:
                output.flush()
                (folder / 'progress.json').write_text(json.dumps(report, indent=2))
                if rendered:
                    print('PROGRESS',round(elapsed,1),'spawned',len(records),'completed',sum(r['completed'] for r in records),'/',report['requested'],flush=True)

        args = argparse.Namespace(host='localhost', port=2000, tick=True, cycles=4,
            green_seconds=12., yellow_seconds=3., clearance_seconds=2., output=str(folder / 'signals.jsonl'))
        run(args, plan, sample, client=client)
        report['unspawned'] = pending
        report['completed'] = sum(r['completed'] for r in records)
        report['passed'] = (not pending and len(records) == report['requested'] and
            all(r['completed'] and not r['wrong_exit'] and r['signal_crossed'] and not r['red_signal_crossing'] and r['min_z'] > -.5 for r in records)
            and not collisions and report['mismatch_frames'] == 0
            and report['always_green_failures'] == 0 and len(report['green_arms']) == 3)
    except BaseException as exc:
        report['error'] = repr(exc)
        raise
    finally:
        output.close()
        for vehicle, sensor, rec in active.values():
            try:
                vehicle.set_autopilot(False, 8010)
            except RuntimeError:
                pass
            try:
                if sensor:
                    if sensor.is_listening:
                        sensor.stop()
                    sensor.destroy()
            except RuntimeError as exc:
                report.setdefault('cleanup_errors', []).append(str(exc))
            try:
                vehicle.destroy()
            except RuntimeError as exc:
                report.setdefault('cleanup_errors', []).append(str(exc))
        world.apply_settings(original)
        tm.set_synchronous_mode(False)
        world.wait_for_tick(10)
        report['remaining_vehicles'] = len(world.get_actors().filter('vehicle.*'))
        report['remaining_sensors'] = len(world.get_actors().filter('sensor.*'))
        report['settings_restored'] = world.get_settings().synchronous_mode == original.synchronous_mode
        report['timings_restored'] = all((l.get_green_time(), l.get_yellow_time(), l.get_red_time()) == saved_signals[sid][1:] for sid, l in lights.items())
        report['passed'] = report['passed'] and not report['remaining_vehicles'] and not report['remaining_sensors'] and report['settings_restored'] and report['timings_restored']
        report['wall_seconds'] = time.monotonic() - wall_start
        report['completed'] = sum(r['completed'] for r in records)
        (folder / 'report.json').write_text(json.dumps(report, indent=2))
        print(json.dumps({k: v for k, v in report.items() if k not in ('vehicles', 'collision_callbacks', 'unspawned')}) + ' collisions=' + str(len(collisions)), flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--junctions', nargs='+', type=int, default=[10, 10010, 40010, 70010])
    parser.add_argument('--flows', nargs='+', choices=list(FLOWS), default=list(FLOWS))
    parser.add_argument('--seeds', nargs='+', type=int, default=[42, 73])
    args = parser.parse_args()
    folder = Path('LocalArtifacts/t_network_control') / datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    folder.mkdir(parents=True)
    design = json.loads((MAP / 'manifest.json').read_text())
    if hashlib.sha256((MAP / 'map.xodr').read_bytes()).hexdigest() != design['opendrive_sha256']:
        raise ValueError('Map fingerprint differs')
    reports = []
    summary = dict(passed=False, cases=reports, scope='Independent local junctions on t_3x3; not simultaneous network control')
    print('OUTPUT', folder, flush=True)
    # Keep one client/TM alive for the complete suite; retiring a previous
    # client's native TM can otherwise stop the next case's vehicle controls.
    client = carla.Client('localhost', 2000)
    client.set_timeout(30)
    tm = client.get_trafficmanager(8010)
    for junction in args.junctions:
        for flow in args.flows:
            for seed in args.seeds:
                name = '{}_{}_{}'.format(junction, flow, seed)
                print('CASE', name, flush=True)
                try:
                    result = case(design, junction, flow, seed, folder / name, client, tm)
                except Exception as exc:
                    result = json.loads((folder / name / 'report.json').read_text())
                    result['error'] = repr(exc)
                reports.append(dict(name=name, passed=result['passed'], requested=result['requested'],
                                    completed=result.get('completed', 0), collisions=len(result['collision_callbacks'])))
                (folder / 'summary.json').write_text(json.dumps(summary, indent=2))
                if not result['passed']:
                    raise SystemExit('Failed: ' + name)
    summary['passed'] = all(r['passed'] for r in reports)
    (folder / 'summary.json').write_text(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
