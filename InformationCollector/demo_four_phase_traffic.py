"""Paced CARLA vehicle demonstration with native four-phase signals."""
import argparse
import json
import math
import time
from datetime import datetime
from pathlib import Path

import carla
import control_four_phases as controller
from control_junction_phases import hold_target_lights, restore_target_lights
from evaluation.pipeline import EvaluatedCollector as FrameCollector


def run(args):
    folder = Path(args.map_dir)
    design = controller.normalize_design(json.loads((folder / 'manifest.json').read_text(encoding='utf-8')))
    client = carla.Client(args.host, args.port)
    client.set_timeout(15)
    world = client.get_world()
    original = world.get_settings()
    if original.synchronous_mode:
        raise RuntimeError('An existing synchronous experiment owns the world')
    world.wait_for_tick(10)
    if any(world.get_actors().filter(p) for p in ('vehicle.*', 'sensor.*', 'walker.*')):
        raise RuntimeError('Finish the existing vehicle/sensor experiment first')
    lights = controller.bind(world, design)
    if any(light.is_frozen() for light in lights.values()):
        raise RuntimeError('Existing lights are frozen')
    out = controller.ROOT / 'evaluation/outputs/four_phase_traffic' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    out.mkdir(parents=True)
    vehicles, sensors, saved, records = [], [], [], []
    report = dict(status='starting', vehicles=records, collision_source='collisions.jsonl',
                  output=str(out), seed=args.seed, right_turn=args.right_turn,
                  limits='Native TM vehicle demonstration; collision callbacks are not a complete coverage certification.')
    tm = client.get_trafficmanager(args.tm_port)
    road_map = world.get_map()
    lengths = dict(zip(design['junctions'][0]['roads'], design['junctions'][0]['lengths']))
    frame_file = None
    collector = None
    safety_raw = None
    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = .05
        world.apply_settings(settings)
        tm.set_synchronous_mode(True)
        # Seed before signal takeover: seeding resets the light network.
        tm.set_random_device_seed(args.seed)
        world.tick()
        hold_target_lights(list(lights.values()), saved)
        for light in lights.values():
            light.set_state(carla.TrafficLightState.Red)
        world.get_settings()
        for move in design['movements']:
            cfg = next(cfg for cfg in design['signals'].values()
                       if cfg['road_id'] == move['road'] and move['lane'] in cfg['incoming_lanes'])
            waypoint = road_map.get_waypoint_xodr(move['road'], move['lane'], cfg['reference_s'] - 30)
            transform = waypoint.transform
            transform.location.z += .35
            bp = world.get_blueprint_library().find('vehicle.tesla.model3')
            bp.set_attribute('role_name', 'four_phase_demo')
            bp.set_attribute('color', {'left': '30,120,255', 'straight': '240,190,30', 'right': '235,235,235'}[move['movement']])
            vehicle = world.spawn_actor(bp, transform)
            vehicles.append(vehicle)
            record = dict(move, actor_id=vehicle.id, arm=cfg['arm'], completed=False,
                          red_stop_observed=False, entered_junction=False)
            records.append(record)
            vehicle.apply_control(carla.VehicleControl(brake=1, hand_brake=True))
        for _ in range(15):
            world.tick()
        # Arrival is 20 m upstream of native signal reference, downstream of
        # the demo spawn at reference_s - 30 m. Default approach-end arrivals
        # would be behind these spawned vehicles and leave trips unpaired.
        collector = FrameCollector(world, junction_id=design['junctions'][0]['id'],
            output_root=out / 'collection', radius=200., mode='synchronous',
            collisions=True, arrival_distance_m=20., exit_distance_m=5.,
            safety_config=getattr(args, 'safety_config', None), safety_map_dir=folder)
        report['collection_output'] = str(collector.output)
        report['collision_source'] = str(collector.output / 'collisions.jsonl')
        safety_raw = (collector.output / 'assessment/safety_input.jsonl').open('w', encoding='utf-8')
        # Baseline after settling, before release; no spawn/landing integration.
        collector.sample(world.get_snapshot())
        for vehicle, record in zip(vehicles, records):
            vehicle.apply_control(carla.VehicleControl())
            vehicle.set_autopilot(True, args.tm_port)
            tm.auto_lane_change(vehicle, False)
            tm.ignore_lights_percentage(vehicle, 0)
            tm.ignore_signs_percentage(vehicle, 0)
            tm.set_desired_speed(vehicle, 20)
            tm.set_route(vehicle, [record['movement'].capitalize()])
        frame_file = (out / 'vehicles.jsonl').open('w', encoding='utf-8')
        last_wall = [time.monotonic()]

        def sample(snapshot, cycle, phase, stage):
            packet = collector.sample(snapshot, phase=dict(source='four_phase_demo', cycle=cycle, phase_id=phase, stage=stage))
            for vehicle, record in zip(vehicles, records):
                state = snapshot.find(vehicle.id)
                if state is None:
                    continue
                location = state.get_transform().location
                velocity = state.get_velocity()
                acceleration = state.get_acceleration()
                yaw = state.get_transform().rotation.yaw
                speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
                wp = road_map.get_waypoint(location)
                at_red = vehicle.is_at_traffic_light() and vehicle.get_traffic_light_state() == carla.TrafficLightState.Red
                if at_red and speed < .15 and snapshot.timestamp.elapsed_seconds - start_sim > 2:
                    record['red_stop_observed'] = True
                if wp and wp.is_junction:
                    record['entered_junction'] = True
                if (not record['completed'] and wp and wp.road_id == record['target_road']
                        and wp.lane_id == record['target_lane'] and wp.s < lengths[wp.road_id] - 30):
                    record.update(completed=True, completion_frame=snapshot.frame,
                                  elapsed_seconds=snapshot.timestamp.elapsed_seconds - start_sim)
                    vehicle.set_autopilot(False, args.tm_port)
                    vehicle.apply_control(carla.VehicleControl(brake=1, hand_brake=True))
                    print('Completed {} {} lane {} ({}/{})'.format(record['arm'], record['movement'],
                          record['lane'], sum(r['completed'] for r in records), len(records)), flush=True)
                if snapshot.frame % 5 == 0:
                    frame_file.write(json.dumps(dict(frame=snapshot.frame, actor=vehicle.id,
                        sim_time=snapshot.timestamp.elapsed_seconds, phase=phase, stage=stage,
                        x=location.x, y=location.y, speed=speed, at_red=at_red,
                        road=wp.road_id if wp else None, lane=wp.lane_id if wp else None)) + '\n')
            safety_raw.write(json.dumps(dict(frame=snapshot.frame, sim_time=snapshot.timestamp.elapsed_seconds,
                                             vehicles=packet['vehicles'], signals=packet['signals'])) + '\n')
            safety_raw.flush()
            # Pace at real time so the experiment is watchable in the CARLA window.
            delay = .05 / args.speed - (time.monotonic() - last_wall[0])
            if delay > 0:
                time.sleep(delay)
            last_wall[0] = time.monotonic()

        start_sim = world.get_snapshot().timestamp.elapsed_seconds
        print('LIVE: {} vehicles; gold=straight, blue=left, white=right'.format(len(vehicles)), flush=True)
        print('Output:', out, flush=True)
        control_args = argparse.Namespace(map_dir=str(folder), host=args.host, port=args.port,
            load=False, check_only=False, tick=True, cycles=args.cycles,
            straight_seconds=15., left_seconds=15., yellow_seconds=3., all_red_seconds=3.,
            right_turn=args.right_turn, output=str(out / 'signals.csv'),
            visualize=True, capture_dir=None)
        controller.run(control_args, sample_callback=sample)
        report.update(status='finished', completed=sum(r['completed'] for r in records),
                      red_stops=sum(r['red_stop_observed'] for r in records))
    except BaseException as exc:
        report.update(status='aborted', error=str(exc))
        raise
    finally:
        (out / 'summary.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        if collector is not None:
            collector.close(report['status'])
        if safety_raw is not None:
            safety_raw.close()
        (out / 'assessment').mkdir(exist_ok=True)
        # Scenario evidence is separate from generic task success and collision audit.
        (out / 'assessment/scenario_evidence.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        if frame_file:
            frame_file.close()
        for sensor in sensors:
            sensor.stop()
            sensor.destroy()
        for vehicle in vehicles:
            vehicle.destroy()
        restore_target_lights(saved)
        tm.set_synchronous_mode(False)
        world.apply_settings(original)
        report['world_settings_restored'] = True
        (out / 'summary.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        print('Summary:', out / 'summary.json', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-dir', default=str(controller.DEFAULT_MAP))
    parser.add_argument('--right-turn', choices=['always-green', 'with-straight', 'red'], default='always-green')
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--tm-port', type=int, default=8020)
    parser.add_argument('--safety-config', help='Safety JSON configuration override')
    parser.add_argument('--seed', type=int, default=17)
    parser.add_argument('--cycles', type=int, default=2)
    parser.add_argument('--speed', type=float, default=1)
    args = parser.parse_args()
    if args.cycles < 1 or not math.isfinite(args.speed) or args.speed <= 0:
        parser.error('cycles and speed must be positive')
    run(args)
