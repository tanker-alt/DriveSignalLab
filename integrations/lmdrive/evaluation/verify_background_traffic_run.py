#!/usr/bin/env python3
"""Audit deterministic background demand against CARLA lane-count records."""
import argparse
import json
from pathlib import Path


def jsonl(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run_dir', type=Path)
    parser.add_argument('--expected-profile', choices=('low', 'medium', 'high'), default='low')
    parser.add_argument('--require-all-exited', action='store_true')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    captures = sorted((args.run_dir / 'image').glob('*'), key=lambda p: p.stat().st_mtime)
    if not captures:
        raise SystemExit('No capture directory')
    capture = captures[-1]
    plan = json.loads((capture / 'background_traffic_plan.json').read_text())
    metrics = json.loads((capture / 'background_traffic_metrics.json').read_text())
    signal_events = jsonl(capture / 'traffic_signals.jsonl')
    background_events = jsonl(capture / 'background_traffic.jsonl')
    lanes = [x for x in signal_events if x.get('type') == 'lane_state']
    assert plan['profile'] == args.expected_profile
    assert plan['schema'] == 'carla_route_background_v1'
    assert metrics['planned'] == plan['total_vehicles']
    assert metrics['spawned'] == metrics['planned']
    assert metrics['approach_seen'] > 0
    assert lanes and all(x['schema'] == 'carla_lane_counts_v1' for x in lanes)
    spawned_ids = {x['actor_id'] for x in background_events if x.get('type') == 'background_spawned'}
    observed_ids = {actor_id for row in lanes for movement in row['vehicle_ids']
                    for side in ('incoming', 'outgoing') for actor_id in movement[side]}
    seen_background_ids = spawned_ids & observed_ids
    assert len(spawned_ids) == metrics['spawned']
    assert seen_background_ids
    assert max(sum(x['incoming']) for x in lanes) >= 2, 'lane counter never saw ego plus background traffic'
    if args.require_all_exited:
        assert metrics['junction_exited'] == metrics['spawned']
    report = {'passed': True, 'run_dir': str(args.run_dir.resolve()),
              'capture_dir': str(capture.resolve()), 'profile': plan['profile'],
              'seed': plan['seed'], 'planned': metrics['planned'],
              'spawned': metrics['spawned'], 'approach_seen': metrics['approach_seen'],
              'junction_entered': metrics['junction_entered'],
              'junction_exited': metrics['junction_exited'],
              'background_ids_seen_by_lane_counter': sorted(seen_background_ids),
              'max_incoming_total': max(sum(x['incoming']) for x in lanes),
              'max_queued_total': max(sum(x['queued']) for x in lanes),
              'lane_state_records': len(lanes)}
    output = args.output or capture / 'background_traffic_audit.json'
    output.write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
