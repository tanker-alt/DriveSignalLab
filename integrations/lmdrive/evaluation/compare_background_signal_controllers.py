#!/usr/bin/env python3
"""Compare matched CARLA background-traffic runs across signal controllers."""
import argparse
import json
import re
from pathlib import Path


def capture_dir(run_dir):
    captures = sorted((run_dir / 'image').glob('*'), key=lambda p: p.stat().st_mtime)
    if not captures:
        raise ValueError('No capture directory in %s' % run_dir)
    return captures[-1]


def mean(values):
    return sum(values) / len(values) if values else None


def percentile(values, q):
    values = sorted(values)
    if not values:
        return None
    pos = (len(values)-1)*q
    left = int(pos); right = min(left+1, len(values)-1)
    return values[left]*(right-pos) + values[right]*(pos-left)


def summarize(label, run_dir):
    capture = capture_dir(run_dir)
    config = json.loads((capture/'experiment_config.json').read_text())
    background = json.loads((capture/'background_traffic_metrics.json').read_text())
    events = [json.loads(line) for line in (capture/'traffic_signals.jsonl').read_text().splitlines()
              if line.strip()]
    lanes = [x for x in events if x.get('type') == 'lane_state']
    snapshots = [x for x in events if x.get('type') == 'signal_snapshot']
    phases = [x for x in events if x.get('type') == 'phase_change']
    vehicles = background['vehicles']
    waiting = [float(x['waiting_s']) for x in vehicles]
    travel = [float(x['sink_at_s'])-1.0-float(x['actual_spawn_s']) for x in vehicles]
    scores = []
    ego_waiting = []
    traffic_metrics = json.loads((capture/'traffic_metrics.json').read_text())
    for ego in range(2):
        result = json.loads((run_dir/('ego_vehicle_%d' % ego)/'results.json').read_text())
        record = result['_checkpoint']['records'][0]
        scores.append({'ego_index': ego, 'route': record['scores']['score_route'],
                       'infractions': sum(len(v) for v in record['infractions'].values())})
        ego_waiting.append(float(traffic_metrics['egos'][ego]['waiting_s']))
    match = re.findall(r'\[SCENARIO_TERMINATED\] sim_t=([0-9.]+)s',
                       (run_dir/'log/log.log').read_text(errors='replace'))
    seed_match = re.search(r'_seed_(\d+)_', run_dir.name)
    return {
        'controller': label,
        'run_dir': str(run_dir.resolve()),
        'lm_seed': int(seed_match.group(1)) if seed_match else config.get('seed'),
        'background_seed': background['seed'],
        'planned_spawned_exited': [background['planned'], background['spawned'],
                                   background['junction_exited']],
        'clearance_time_s': float(match[-1]) if match else None,
        'background_waiting_s': {'mean': mean(waiting), 'p95': percentile(waiting, .95),
                                 'max': max(waiting)},
        'background_travel_time_s': {'mean': mean(travel), 'p95': percentile(travel, .95),
                                     'max': max(travel)},
        'queue_total': {'mean': mean([sum(x['queued']) for x in lanes]),
                        'p95': percentile([sum(x['queued']) for x in lanes], .95),
                        'max': max(sum(x['queued']) for x in lanes)},
        'ego_waiting_s': ego_waiting,
        'phase_changes': len(phases),
        'signal_snapshots': len(snapshots),
        'conflicting_green_snapshots': sum(
            sum(v == 'Green' for v in x['actual_states'].values()) > 1 for x in snapshots),
        'scores': scores,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--fixed', type=Path, required=True)
    parser.add_argument('--maxpressure', type=Path, required=True)
    parser.add_argument('--presslight', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    runs = [summarize('fixed', args.fixed),
            summarize('maxpressure', args.maxpressure),
            summarize('presslight', args.presslight)]
    seeds = {(x['lm_seed'], x['background_seed']) for x in runs}
    if len(seeds) != 1:
        raise ValueError('Runs do not have matched seeds: %r' % (seeds,))
    for run in runs:
        assert run['planned_spawned_exited'][0] == run['planned_spawned_exited'][1] == run['planned_spawned_exited'][2]
        assert run['conflicting_green_snapshots'] == 0
        assert all(x['route'] == 100 and x['infractions'] == 0 for x in run['scores'])
    report = {'schema': 'matched_signal_controller_comparison_v1',
              'traffic_profile': 'medium', 'matched_seeds': list(seeds)[0], 'runs': runs}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
