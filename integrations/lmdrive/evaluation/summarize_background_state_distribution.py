#!/usr/bin/env python3
"""Summarize observed signal-controller state ranges for PressLight training."""
import argparse
import json
from pathlib import Path


def percentile(values, q):
    ordered = sorted(values)
    if not ordered:
        return None
    index = (len(ordered) - 1) * q
    left = int(index)
    right = min(left + 1, len(ordered) - 1)
    fraction = index - left
    return ordered[left] * (1 - fraction) + ordered[right] * fraction


def describe(values):
    return {'min': min(values), 'max': max(values),
            'mean': sum(values) / len(values),
            'p50': percentile(values, .5), 'p95': percentile(values, .95)}


def capture_dir(run_dir):
    captures = sorted((run_dir / 'image').glob('*'), key=lambda p: p.stat().st_mtime)
    if not captures:
        raise ValueError('No capture directory in %s' % run_dir)
    return captures[-1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run_dirs', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rows = []
    runs = []
    for run_dir in args.run_dirs:
        capture = capture_dir(run_dir)
        events = [json.loads(line) for line in (capture / 'traffic_signals.jsonl').read_text().splitlines()
                  if line.strip()]
        lanes = [x for x in events if x.get('type') == 'lane_state']
        plan = json.loads((capture / 'background_traffic_plan.json').read_text())
        metrics = json.loads((capture / 'background_traffic_metrics.json').read_text())
        for lane in lanes:
            rows.append({key: lane[key] for key in ('incoming', 'outgoing', 'queued', 'pressure')})
        runs.append({'run_dir': str(run_dir.resolve()), 'profile': plan['profile'],
                     'seed': plan['seed'], 'records': len(lanes),
                     'planned': metrics['planned'], 'spawned': metrics['spawned'],
                     'exited': metrics['junction_exited']})
    features = {}
    for key in ('incoming', 'outgoing', 'queued', 'pressure'):
        features[key] = [describe([row[key][movement] for row in rows]) for movement in (0, 1)]
    totals = {key: describe([sum(row[key]) for row in rows])
              for key in ('incoming', 'outgoing', 'queued')}
    report = {'schema': 'carla_background_state_distribution_v1', 'runs': runs,
              'records': len(rows), 'features': features, 'totals': totals,
              'observed_state_dim': 6,
              'presslight_state_order': ['phase_0', 'phase_1', 'incoming_0',
                                         'outgoing_0', 'incoming_1', 'outgoing_1']}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
