"""Run the actual T controller with six native-TM vehicles and collision sensors."""
import argparse
import datetime
import json
from pathlib import Path
import carla
from control_t_phases import build_plan, run, DEFAULT_MAP


def main():
    folder = Path('LocalArtifacts/t_phase_control') / datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    folder.mkdir(parents=True)
    design = json.loads((DEFAULT_MAP / 'manifest.json').read_text())
    plan = build_plan(design)
    client = carla.Client('localhost', 2000)
    client.set_timeout(20)
    world = client.get_world()
    # Keep the visual demo rendered and place the spectator where the signal heads are visible.
    visual = world.get_spectator()
    visual.set_transform(carla.Transform(carla.Location(-35, 7.75, 8), carla.Rotation(pitch=-15, yaw=0, roll=0)))
    original = world.get_settings()
    if original.synchronous_mode or any(world.get_actors().filter(p) for p in ('vehicle.*', 'sensor.*')):
        raise RuntimeError('Finish the active experiment first')
    from control_t_phases import bind
    bind(world, plan)
    tm = client.get_trafficmanager(8010)
    vehicles, sensors, records, collisions = [], [], [], []
    report = dict(passed=False, map='t3_north', phases=plan['phases'], vehicles=records,
                  collision_callbacks=collisions)
    stream = (folder / 'vehicles.jsonl').open('w')
    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = .05
        world.apply_settings(settings)
        tm.set_synchronous_mode(True)
        tm.set_random_device_seed(42)
        # Refresh the synchronous actor snapshot after TM initialization resets signals.
        world.tick()
        for sid, cfg in plan['signals'].items():
            movement = next(m for m in design['movements'] if m['road'] == cfg['road_id'] and m['lane'] == cfg['incoming_lanes'][0])
            wp = world.get_map().get_waypoint_xodr(cfg['road_id'], cfg['incoming_lanes'][0], cfg['reference_s'] - 35)
            transform = wp.transform
            transform.location.z += .4
            v = world.spawn_actor(world.get_blueprint_library().find('vehicle.tesla.model3'), transform)
            vehicles.append(v)
            sensor = world.spawn_actor(world.get_blueprint_library().find('sensor.other.collision'), carla.Transform(), attach_to=v)
            sensors.append(sensor)
            sensor.listen(lambda e: collisions.append(dict(frame=e.frame, actor=e.actor.id, other=e.other_actor.id)))
            v.set_autopilot(True, 8010)
            tm.auto_lane_change(v, False)
            tm.vehicle_percentage_speed_difference(v, 60)
            tm.set_route(v, [movement['movement'].title()])
            records.append(dict(signal=sid, vehicle=v.id, movement=movement, completed=False,
                                stopped_on_red=False, crossed_on_red=False, wrong_exit=False))

        def sample(snapshot, index, stage, expected, actual):
            visual.set_transform(carla.Transform(carla.Location(-35, 7.75, 8), carla.Rotation(pitch=-15, yaw=0, roll=0)))
            for v, rec in zip(vehicles, records):
                if rec['completed']:
                    continue
                wp = world.get_map().get_waypoint(v.get_location())
                speed = v.get_velocity().length()
                cfg = plan['signals'][rec['signal']]
                move = rec['movement']
                red = actual[rec['signal']] == 'Red'
                if wp.road_id == cfg['road_id'] and wp.lane_id == cfg['incoming_lanes'][0]:
                    if red and speed < .1 and abs(wp.s - cfg['reference_s']) < 18:
                        rec['stopped_on_red'] = True
                    # Front-bumper crossing is not claimed: this is center crossing at native signal s.
                    if red and wp.s > cfg['signal_s'] and not rec.get('entered'):
                        rec['crossed_on_red'] = True
                if wp.is_junction:
                    rec['entered'] = True
                if rec.get('entered') and not wp.is_junction:
                    if (wp.road_id, wp.lane_id) != (move['target_road'], move['target_lane']):
                        rec['wrong_exit'] = True
                    if wp.road_id == move['target_road'] and wp.s < cfg['reference_s'] - 25:
                        rec['completed'] = True
                        rec['completion_frame'] = snapshot.frame
                        v.set_autopilot(False, 8010)
                        v.apply_control(carla.VehicleControl(brake=1, hand_brake=True))
                stream.write(json.dumps(dict(frame=snapshot.frame, vehicle=v.id, road=wp.road_id,
                    lane=wp.lane_id, s=wp.s, speed=speed, signal=actual[rec['signal']], stage=stage)) + '\n')

        args = argparse.Namespace(host='localhost', port=2000, tick=True, cycles=2,
            green_seconds=12., yellow_seconds=3., clearance_seconds=2., output=str(folder / 'signals.jsonl'))
        run(args, plan, sample, client=client)
        report['passed'] = all(r['completed'] and not r['wrong_exit'] and not r['crossed_on_red'] for r in records) and not collisions
    except BaseException as exc:
        report['error'] = repr(exc)
        raise
    finally:
        stream.close()
        for sensor in sensors:
            sensor.stop()
            sensor.destroy()
        for vehicle in vehicles:
            vehicle.destroy()
        tm.set_synchronous_mode(False)
        world.apply_settings(original)
        world.wait_for_tick(10)
        report['remaining_vehicles'] = len(world.get_actors().filter('vehicle.*'))
        report['remaining_sensors'] = len(world.get_actors().filter('sensor.*'))
        (folder / 'report.json').write_text(json.dumps(report, indent=2))
        print('REPORT', folder / 'report.json', flush=True)
        print(json.dumps(report, indent=2), flush=True)
    if not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
