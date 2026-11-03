"""Independent checks of saved T-network experiment evidence (no CARLA)."""
import argparse
import collections
import hashlib
import json
from pathlib import Path


def audit(folder):
    folder = Path(folder)
    summary = json.loads((folder / 'summary.json').read_text())
    totals = dict(passed=False, cases=0, audited_passed_cases=0, failed_cases=[], vehicles=0, frames=0, collision_callbacks=0,
                  green_releases=0, clearance_extensions_before_green=0, max_clearance_before_green_seconds=0,
                  red_signal_crossings=0, red_junction_entries_after_legal_crossing=0,
                  wrong_exits=0, directions={}, flows={}, fleet={})
    fleet = collections.Counter()
    for case in summary['cases']:
        path = folder / case['name']
        report = json.loads((path / 'report.json').read_text())
        totals['cases'] += 1
        if not report['passed']:
            totals['failed_cases'].append(dict(name=case['name'], reason=report.get('error', 'Acceptance checks failed')))
            continue
        assert report['passed'] and report['completed'] == report['requested']
        assert len(report['vehicles']) == report['requested']
        assert not report['remaining_vehicles'] and not report['remaining_sensors']
        assert report['settings_restored'] and report['timings_restored']
        assert not report['collision_callbacks']
        assert all(v['completed'] and v['signal_crossed'] and not v['red_signal_crossing'] and not v['wrong_exit'] for v in report['vehicles'])
        observed_fleet = set(v['blueprint'] for v in report['vehicles'])
        if report.get('random_fleet'):
            assert observed_fleet <= set(report['fleet'])
        else:
            assert observed_fleet == set(report['fleet'])
        order = {'north': ['west', 'south', 'east'], 'east': ['north', 'west', 'south'],
                 'south': ['east', 'north', 'west'], 'west': ['south', 'east', 'north']}[report['missing_arm']]
        lines = iter((path / 'signals.jsonl').read_text().splitlines())
        plan = json.loads(next(lines))['plan']
        # Independently derive the rightmost lane from the map, including
        # three-lane mixed networks; do not reuse controller state functions.
        map_dir = Path(report.get('map_dir', str(Path(__file__).resolve().parents[1] / 'Maps/MapDataset/t_3x3')))
        design = json.loads((map_dir / 'manifest.json').read_text())
        assert report['opendrive_sha256'] == plan['opendrive_sha256'] == design['opendrive_sha256']
        assert hashlib.sha256((map_dir / 'map.xodr').read_bytes()).hexdigest() == design['opendrive_sha256']
        junction = next(j for j in design['junctions'] if j['id'] == report['junction_id'])
        signal_cfg = {sid: cfg for sid, cfg in design['signals'].items() if cfg['junction_id'] == junction['id']}
        rightmost = {road: max(abs(c['incoming_lanes'][0]) for c in signal_cfg.values() if c['road_id'] == road)
                     for road in junction['arms'].values()}
        vehicle_records = {v['actor_id']: v for v in report['vehicles']}
        crossing = {}
        for line in (path / 'vehicles.jsonl').read_text().splitlines():
            v = json.loads(line)
            cfg = signal_cfg[vehicle_records[v['actor']]['signal']]
            if v['actor'] not in crossing and v['road'] == cfg['road_id'] and v['lane'] in cfg['incoming_lanes'] and v['s'] >= cfg['signal_s']:
                crossing[v['actor']] = v['light']
        assert set(crossing) == set(vehicle_records), 'Missing native-section crossing evidence'
        assert all(color in ('Green', 'Yellow') for color in crossing.values())
        assert all(v['signal_crossing_light'] == crossing[actor] for actor, v in vehicle_records.items())
        totals['red_junction_entries_after_legal_crossing'] += sum(v['red_entry'] for v in report['vehicles'])
        assert set(plan['signals']) == set(signal_cfg)
        previous, segment, segment_start, phase_no, frames = None, None, None, 0, 0
        for line in lines:
            row = json.loads(line)
            frames += 1
            assert not row['stage'].startswith('APPLYING_'), 'Unconfirmed transition in evidence'
            expected = {}
            for sid, cfg in signal_cfg.items():
                assert len(cfg['incoming_lanes']) == 1 and cfg['incoming_lanes'][0] != 0
                color = 'Red'
                if row['stage'] != 'ALL_RED':
                    if abs(cfg['incoming_lanes'][0]) == rightmost[cfg['road_id']]:
                        color = 'Green'
                    elif cfg['road_id'] == junction['arms'][row['arm']] and row['stage'] in ('GREEN', 'YELLOW'):
                        color = row['stage'].title()
                expected[sid] = color
            assert row['actual'] == expected and row['expected'] == expected
            key = (row['cycle'], row['arm'], row['stage'])
            if previous:
                assert row['frame'] == previous['frame'] + 1, 'Missing simulation frames'
            if key != segment:
                if previous:
                    duration = previous['sim_time'] - segment_start
                    minimum = {'GREEN': 12., 'YELLOW': 3., 'MAIN_RED': 2., 'ALL_RED': 2.}[previous['stage']]
                    assert duration + 1e-4 >= minimum
                    if previous['stage'] in ('ALL_RED', 'MAIN_RED'):
                        assert not previous['clearance_vehicle_ids'], 'Released green with occupied controlled path'
                        totals['max_clearance_before_green_seconds'] = max(totals['max_clearance_before_green_seconds'], duration)
                        if duration > 2.1:
                            totals['clearance_extensions_before_green'] += 1
                if row['stage'] == 'GREEN':
                    assert row['arm'] == order[phase_no % 3]
                    assert previous and previous['stage'] in ('MAIN_RED', 'ALL_RED')
                    totals['green_releases'] += 1
                    phase_no += 1
                segment, segment_start = key, row['sim_time']
            previous = row
        assert previous['stage'] == 'MAIN_RED' and not previous['clearance_vehicle_ids']
        assert previous['sim_time'] - segment_start + 1e-4 >= 2
        assert phase_no >= 3 and phase_no % 3 == 0
        assert frames == report['frames']
        totals['audited_passed_cases'] += 1
        totals['frames'] += frames
        totals['vehicles'] += report['completed']
        totals['directions'][report['missing_arm']] = totals['directions'].get(report['missing_arm'], 0) + 1
        totals['flows'][report['flow']] = totals['flows'].get(report['flow'], 0) + report['completed']
        fleet.update(v['blueprint'] for v in report['vehicles'])
    totals['fleet'] = dict(fleet)
    totals['passed'] = summary['passed'] and not totals['failed_cases']
    totals['metric_scope'] = 'Detailed totals cover audited passing cases; failed cases are listed separately.'
    (folder / 'audit.json').write_text(json.dumps(totals, indent=2))
    print(json.dumps(totals, indent=2))
    return totals


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder')
    if not audit(parser.parse_args().folder)['passed']:
        raise SystemExit(1)
