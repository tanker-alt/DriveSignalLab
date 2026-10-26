"""Build a conservative lane-separated candidate, without certified timings."""
import argparse
import hashlib
import json
from pathlib import Path
from roundabout_conflicts import run as audit


def build(folder):
    folder = Path(folder)
    d = json.loads((folder / 'manifest.json').read_text())
    crossing = audit(folder)
    def signal(path):
        return next(sid for sid, s in d['signals'].items()
                    if s['road_id'] == path['source'] and path['source_lane'] in s['incoming_lanes'])
    conflicts = [dict(signals=[signal(p) for p in c['paths']], kind='crossing')
                 for c in crossing['conflicts']]
    for i, a in enumerate(d['connections']):
        for b in d['connections'][i+1:]:
            if (a['target'], a['target_lane']) == (b['target'], b['target_lane']) and signal(a) != signal(b):
                conflicts.append(dict(signals=[signal(a), signal(b)], kind='merge'))
    phases = []
    for entry, lane, name in [(False, -1, 'ring_inner'), (False, -2, 'ring_outer'),
                              (True, -1, 'entry_inner'), (True, -2, 'entry_outer')]:
        green = sorted(sid for sid, s in d['signals'].items()
                       if (s['role'] == 'entry') == entry and s['incoming_lanes'] == [lane])
        bad = [c for c in conflicts if set(c['signals']).issubset(green)]
        phases.append(dict(name=name, green_signal_ids=green, detected_conflicts=bad))
    assigned = [sid for p in phases for sid in p['green_signal_ids']]
    assert len(assigned) == len(set(assigned)) == 16 and set(assigned) == set(d['signals'])
    result = dict(opendrive_sha256=d['opendrive_sha256'], phases=phases, conflicts=conflicts,
                  static_check_passed=all(not p['detected_conflicts'] for p in phases),
                  traffic_ready=False, green_seconds=None, yellow_seconds=None, clearance_seconds=None,
                  limits='Candidate only. No swept vehicle volumes, stopping, queue storage or transition clearance validation. Do not run traffic from this plan.')
    (folder / 'lane_phase_candidate.json').write_text(json.dumps(result, indent=2))
    print('Lane plan static check:', result['static_check_passed'], 'conflict pairs:', len(conflicts))
    return result


def show(folder, plan):
    import carla
    c = carla.Client('localhost', 2000); c.set_timeout(20)
    w = c.get_world()
    def idle():
        if w.get_settings().synchronous_mode or any(w.get_actors().filter(p) for p in ('vehicle.*', 'walker.*')):
            raise RuntimeError('World busy; visual demo only')
    idle()
    if hashlib.sha256(w.get_map().to_opendrive().encode()).hexdigest() != plan['opendrive_sha256']:
        raise RuntimeError('Map differs')
    lights = {a.get_opendrive_id(): a for a in w.get_actors().filter('traffic.traffic_light')}
    if set(lights) != {s for p in plan['phases'] for s in p['green_signal_ids']}:
        raise RuntimeError('Signal set differs')
    rows = []
    try:
        for phase in plan['phases']:
            idle()
            for sid, light in lights.items():
                light.freeze(True)
                light.set_state(carla.TrafficLightState.Green if sid in phase['green_signal_ids'] else carla.TrafficLightState.Red)
            for _ in range(60):
                idle(); w.wait_for_tick(10)
            states = {sid: str(light.get_state()) for sid, light in lights.items()}
            passed = all((light.get_state() == carla.TrafficLightState.Green) == (sid in phase['green_signal_ids'])
                         and light.get_state() in (carla.TrafficLightState.Green, carla.TrafficLightState.Red)
                         for sid, light in lights.items())
            rows.append(dict(name=phase['name'], states=states, passed=passed))
    finally:
        for light in lights.values():
            light.set_state(carla.TrafficLightState.Red); light.freeze(True)
    (Path(folder) / 'lane_phase_display_validation.json').write_text(json.dumps(dict(
        passed=all(r['passed'] for r in rows), phases=rows, traffic_test=False), indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map_dir'); parser.add_argument('--show', action='store_true')
    args = parser.parse_args(); plan = build(args.map_dir)
    if not plan['static_check_passed']: raise SystemExit(1)
    if args.show: show(args.map_dir, plan)
