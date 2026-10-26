#!/usr/bin/env python3
import argparse
import json
from pathlib import Path


def read_jsonl(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run_dir', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    captures = sorted((args.run_dir/'image').glob('*'), key=lambda p:p.stat().st_mtime)
    if not captures:
        raise SystemExit('No capture directory')
    capture = captures[-1]
    events = read_jsonl(capture/'traffic_signals.jsonl')
    phases = [x for x in events if x.get('type') == 'phase_change']
    lanes = [x for x in events if x.get('type') == 'lane_state']
    snapshots = [x for x in events if x.get('type') == 'signal_snapshot']
    config = json.loads((capture/'experiment_config.json').read_text())
    controller = config['signal_controller']
    names = [x['phase'] for x in phases]
    assert lanes and all(x['schema'] == 'carla_lane_counts_v1' for x in lanes)
    assert all(len(x[key]) == 2 for x in lanes for key in ('incoming','outgoing','queued','pressure'))
    assert all(sum(value == 'Green' for value in x['actual_states'].values()) <= 1 for x in snapshots)
    greens = [(i,name) for i,name in enumerate(names) if name.endswith('_green')]
    assert {name for _,name in greens} == {'movement_0_green','movement_1_green'}
    for (left_i,left),(right_i,right) in zip(greens,greens[1:]):
        if left != right:
            between = names[left_i+1:right_i]
            assert any(name.endswith('_yellow') for name in between), between
            assert 'all_red_clearance' in between, between
    if controller == 'presslight':
        assert any(x.get('q_values') and len(x['q_values']) == 2 for x in phases)
        checkpoint = config.get('presslight_checkpoint')
        assert checkpoint and Path(checkpoint).is_file()
    scores = []
    for ego in range(2):
        result = json.loads((args.run_dir/('ego_vehicle_%d' % ego)/'results.json').read_text())
        record = result['_checkpoint']['records'][0]
        scores.append({'ego_index':ego,'scores':record['scores'],
                       'infractions':{key:len(value) for key,value in record['infractions'].items()}})
        assert record['scores']['score_route'] == 100.0
        assert all(not value for value in record['infractions'].values())
    metrics = json.loads((capture/'traffic_metrics.json').read_text())
    assert metrics['throughput_vehicles'] == 2
    report = {'passed':True,'controller':controller,'run_dir':str(args.run_dir.resolve()),
              'capture_dir':str(capture.resolve()),
              'phase_changes':[{'elapsed_s':x['elapsed_s'],'phase':x['phase'],
                                'pressure':x.get('lane_state',{}).get('pressure'),
                                'q_values':x.get('q_values')} for x in phases],
              'lane_state_records':len(lanes),'signal_snapshots':len(snapshots),
              'conflicting_green_snapshots':0,'traffic_metrics':metrics,'scores':scores}
    output = args.output or capture/'adaptive_signal_audit.json'
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
