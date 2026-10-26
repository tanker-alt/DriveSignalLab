"""Read-only browser viewer for live and recorded MDrive dual-ego runs."""
import argparse
import json
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse, parse_qs, unquote

ROOT = Path(os.environ.get('EXPERIMENT_OUTPUT_ROOT', 'outputs')).resolve()
RUN_PATTERN = 'lmdrive_signal_*'
PAGE = Path(__file__).with_name('mdrive_preview.html')

def runs():
    return sorted((p for p in ROOT.glob(RUN_PATTERN) if p.is_dir() and '_partial_' not in p.name and (p / 'image').exists()), key=lambda p: p.stat().st_mtime, reverse=True)

def read_jsonl(path):
    if not path.exists():
        return []
    records = []
    for line in path.read_text().splitlines():
        try:
            records.append(json.loads(line))
        except ValueError:
            pass  # A live writer may not have finished the last line.
    return records


def lmdrive_frames(capture, ego, events):
    controls = [r for r in read_jsonl(capture / 'model_audit.jsonl')
                if r.get('type') == 'final_control' and r.get('ego_index') == ego]
    messages = {(r['carla_frame'], r['ego_index']): r for r in events
                if r.get('type') == 'vehicle_signal_message'}
    frames = []
    for path in sorted((capture / ('meta_%d' % ego)).glob('*_route_debug.json')):
        row = read_json(path)
        if row is None:
            continue
        step = row['step']
        image_path = path.with_name(path.name.replace('_route_debug.json', '.jpg'))
        if step >= len(controls) or not image_path.exists():
            continue
        # LMDrive starts at step zero and audits one final control per ego step.
        audit = controls[step]
        message = messages.get((audit['frame'], ego))
        if message and message['camera_frame'] != audit['camera_frame']:
            message = None
        frames.append(dict(frame=step, x=row['ego_world']['x'], y=row['ego_world']['y'],
                           **{k: row['control'][k] for k in ('throttle', 'steer', 'brake')},
                           signal=message, image='/asset/' + image_path.relative_to(ROOT).as_posix()))
    return frames


def read_json(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def reply(self, data, mime='application/json', status=200):
        self.send_response(status)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        request = urlparse(self.path)
        if request.path == '/':
            return self.reply(PAGE.read_bytes(), 'text/html; charset=utf-8')
        if request.path == '/api':
            candidates = runs()
            selected = parse_qs(request.query).get('run', [''])[0]
            run = next((p for p in candidates if p.name == selected), None) if selected else (candidates[0] if candidates else None)
            payload = {'runs': [p.name for p in candidates], 'run': run.name if run else None, 'egos': [], 'status': None, 'signals': [], 'signal_snapshots': [], 'lane_states': []}
            if run:
                payload['frame_seconds'] = 0.05 if 'lmdrive' in run.name else 0.5
                payload['model'] = 'LMDrive VLA' if 'lmdrive' in run.name else 'autopilot'
                payload['status'] = read_json(run / '_run_status.json')
                captures = sorted((p for p in (run / 'image').glob('*') if p.is_dir()), key=lambda p: p.stat().st_mtime, reverse=True)
                capture = captures[0] if captures else None
                events = []
                if capture:
                    events = read_jsonl(capture / 'traffic_signals.jsonl')
                    payload['signals'] = read_json(capture / 'traffic_light_mapping.json') or []
                    signal_log = capture / 'traffic_signals.jsonl'
                    if signal_log.exists():
                        for line in signal_log.read_text().splitlines():
                            try:
                                event = json.loads(line)
                                if event.get('type') == 'signal_snapshot':
                                    payload['signal_snapshots'].append(event)
                                elif event.get('type') == 'lane_state':
                                    payload['lane_states'].append(event)
                            except ValueError:
                                pass
                for ego in range(2):
                    frames = []
                    if capture:
                        for path in sorted((capture / ('ego_vehicle_%d' % ego) / 'measurements').glob('*.json')):
                            row = read_json(path)
                            image_path = path.parent.parent / 'rgb_front' / (path.stem + '.jpg')
                            if row is not None and image_path.exists():
                                frames.append({'frame': int(path.stem), 'x': row['x'], 'y': row['y'], 'steer': row['steer'], 'throttle': row['throttle'], 'brake': row['brake'], 'signal': row.get('traffic_light_message'), 'image': '/asset/' + image_path.relative_to(ROOT).as_posix()})
                    if capture and (capture / 'model_audit.jsonl').exists():
                        frames = lmdrive_frames(capture, ego, events)
                    score = read_json(run / ('ego_vehicle_%d' % ego) / 'results.json')
                    records = score.get('_checkpoint', {}).get('records', []) if score else []
                    payload['egos'].append({'frames': frames, 'score': records[0].get('scores') if records else None})
            return self.reply(json.dumps(payload).encode())
        if request.path.startswith('/asset/'):
            path = (ROOT / unquote(request.path[len('/asset/'):])).resolve()
            if any(run.resolve() in path.parents for run in runs()) and path.suffix == '.jpg' and path.is_file():
                return self.reply(path.read_bytes(), 'image/jpeg')
        self.reply(b'Not found', 'text/plain', 404)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8766)
    parser.add_argument('--root', type=Path, default=ROOT)
    args = parser.parse_args()
    ROOT = args.root.resolve()
    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    print('MDrive preview: http://127.0.0.1:%d (Ctrl+C to stop)' % args.port, flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
