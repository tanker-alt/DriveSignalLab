"""Replay waiting/trips only, preserving the historical analysis scope."""
import argparse
import csv
import json
from pathlib import Path
from datetime import datetime, timezone
from evaluation.vehicle_tracker import VehicleTracker
from evaluation.trip_tracker import TripTracker
from evaluation.detection_lines import DetectionLines


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('analysis', type=Path)
    args = parser.parse_args()
    run = args.analysis.parent
    scope = json.loads((args.analysis / 'replay_scope.json').read_text(encoding='utf-8'))
    old = json.loads((args.analysis / 'summary.json').read_text(encoding='utf-8'))
    tracker = VehicleTracker(**old['config'])
    trips = TripTracker(old['config']['junction_id'])
    lines = json.loads((run / 'detection_lines_used.json').read_text(encoding='utf-8'))
    detector = DetectionLines(lines['lines'])
    output = run / ('wait_fix_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
    output.mkdir()
    with (run / 'frames.csv').open(encoding='utf-8-sig', newline='') as ff, \
            (run / 'vehicles.csv').open(encoding='utf-8-sig', newline='') as vf:
        source = iter(csv.DictReader(vf))
        row = next(source, None)
        for frame in csv.DictReader(ff):
            number, now = int(frame['frame']), float(frame['sim_time'])
            rows = []
            while row is not None and int(row['frame']) == number:
                rows.append(row)
                row = next(source, None)
            if len(rows) != int(frame['vehicle_count']):
                raise ValueError('Frame vehicle count mismatch')
            if scope['start_frame'] is not None and number < scope['start_frame']:
                continue
            tracker.update(number, now, rows)
            crossings = detector.update(number, now, rows)
            trips.update(number, now, rows, crossings, tracker)
    trips.close()
    (output / 'trips.jsonl').write_text(''.join(json.dumps(t) + '\n' for t in trips.completed), encoding='utf-8')
    report = dict(source_analysis=str(args.analysis), scope=scope, summary=trips.summary(),
                  method='same raw frames, original counting sections and release boundary; waiting/trips only')
    (output / 'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(output)
    print(json.dumps(trips.summary(), indent=2))


if __name__ == '__main__':
    main()
