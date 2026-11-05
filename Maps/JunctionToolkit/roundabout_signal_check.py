"""One vehicle at a time: native TM red stop and green release for all 16 lights."""
import argparse
import hashlib
import json
import time
from pathlib import Path
import carla
from roundabout_drive_check import route, sparse_path


def run(folder):
    folder = Path(folder); d = json.loads((folder / 'manifest.json').read_text())
    c = carla.Client('localhost', 2000); c.set_timeout(20); w = c.get_world(); m = w.get_map()
    if w.get_settings().synchronous_mode or any(w.get_actors().filter(p) for p in ('vehicle.*', 'sensor.*', 'walker.*')):
        raise RuntimeError('World busy')
    if hashlib.sha256(m.to_opendrive().encode()).hexdigest() != d['opendrive_sha256']:
        raise RuntimeError('Map differs')
    lights = {a.get_opendrive_id(): a for a in w.get_actors().filter('traffic.traffic_light')}
    if set(lights) != set(d['signals']): raise RuntimeError('Signal set differs')
    world_id = w.id
    def same_world():
        return c.get_world().id == world_id
    def tick():
        if not same_world(): raise RuntimeError('World changed; validation interrupted')
        return w.tick(10.)
    states = [(l, l.get_state(), l.is_frozen()) for l in lights.values()]
    original = w.get_settings(); tm = c.get_trafficmanager(8012); vehicle = sensor = None
    result = dict(passed=False, signals=[], opendrive_sha256=d['opendrive_sha256'],
                  limits='Single Model3, target red and other lights green; does not validate simultaneous queues, lane neighbours or phase transitions.')
    try:
        settings = w.get_settings(); settings.synchronous_mode = True; settings.fixed_delta_seconds = .05
        w.apply_settings(settings); tm.set_synchronous_mode(True); tm.set_random_device_seed(42)
        for sid, cfg in d['signals'].items():
            lane = cfg['incoming_lanes'][0]; source = cfg['road_id'] if cfg['role'] == 'entry' else 1
            path = route(m, d, source, lane, source)
            candidates = [i for i, p in enumerate(path) if p.road_id == cfg['road_id'] and p.lane_id == lane]
            nearest = min(candidates, key=lambda i: abs(path[i].s - cfg['reference_s']))
            start = max(0, nearest - 15)
            while start > 0 and path[start].is_junction: start -= 1
            path = path[start:]
            for key, light in lights.items():
                light.freeze(True); light.set_state(carla.TrafficLightState.Red if key == sid else carla.TrafficLightState.Green)
            rec = dict(signal_id=sid, role=cfg['role'], red_stop=False, green_release=False, collisions=[])
            result['signals'].append(rec)
            try:
                t = path[0].transform; t.location.z += .4
                vehicle = w.spawn_actor(w.get_blueprint_library().find('vehicle.tesla.model3'), t)
                sensor = w.spawn_actor(w.get_blueprint_library().find('sensor.other.collision'), carla.Transform(), attach_to=vehicle)
                sensor.listen(lambda e, row=rec: row['collisions'].append(dict(frame=e.frame, other=e.other_actor.id)))
                for _ in range(20): tick()
                vehicle.set_autopilot(True, 8012); tm.auto_lane_change(vehicle, False)
                tm.ignore_lights_percentage(vehicle, 0); tm.set_desired_speed(vehicle, 18.)
                tm.set_path(vehicle, sparse_path(path)); held = 0; deadline = time.monotonic() + 70
                for _ in range(700):
                    if time.monotonic() > deadline: raise RuntimeError('Simulation timeout')
                    tick()
                    if not vehicle.is_alive:
                        rec['failure']='vehicle_removed_before_stop';break
                    observed = vehicle.get_traffic_light()
                    matches = observed is not None and observed.get_opendrive_id() == sid
                    held = held + 1 if matches and vehicle.get_velocity().length() < .1 else 0
                    if held >= 20:
                        rec['red_stop'] = True; break
                if not vehicle.is_alive:
                    rec['passed']=False;print(json.dumps(rec),flush=True);continue
                wp = m.get_waypoint(vehicle.get_location())
                rec['stop_waypoint'] = dict(road=wp.road_id, lane=wp.lane_id, s=wp.s)
                rec['observed_signal'] = observed.get_opendrive_id() if observed else None
                rec['red_stop'] = rec['red_stop'] and wp.road_id == cfg['road_id'] and wp.lane_id == lane
                stopped = vehicle.get_location(); lights[sid].set_state(carla.TrafficLightState.Green)
                for _ in range(400):
                    if time.monotonic() > deadline: raise RuntimeError('Simulation timeout')
                    tick()
                    if not vehicle.is_alive:
                        rec['failure']='vehicle_removed_after_release';break
                    if vehicle.get_location().distance(stopped) > 12 and vehicle.get_velocity().length() > 1:
                        rec['green_release'] = True; break
                for _ in range(3): tick()
                rec['passed'] = rec['red_stop'] and rec['green_release'] and not rec['collisions']
                print(json.dumps(rec), flush=True)
            finally:
                if same_world() and sensor and sensor.is_alive: sensor.stop(); sensor.destroy(); sensor = None
                if same_world() and vehicle and vehicle.is_alive: vehicle.destroy(); vehicle = None
                if same_world(): tick()
            (folder / 'native_signal_validation.json').write_text(json.dumps(result, indent=2))
        result['passed'] = len(result['signals']) == 16 and all(r['passed'] for r in result['signals'])
    finally:
        if same_world() and sensor and sensor.is_alive: sensor.stop(); sensor.destroy()
        if same_world() and vehicle and vehicle.is_alive: vehicle.destroy()
        if same_world():
            for light, state, frozen in states: light.set_state(state); light.freeze(frozen)
            tm.set_synchronous_mode(False); w.apply_settings(original)
        else:
            result["interrupted"] = "world_changed"; result["passed"] = False
        (folder / 'native_signal_validation.json').write_text(json.dumps(result, indent=2))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument('map_dir')
    if not run(parser.parse_args().map_dir)['passed']: raise SystemExit(1)
