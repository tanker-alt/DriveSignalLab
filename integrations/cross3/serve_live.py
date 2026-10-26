#!/usr/bin/env python3
"""Read-only live viewer; optionally launch the Cross3 experiment it observes."""
import argparse
import errno
import copy
import io
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs
from urllib.request import urlopen

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]

def read(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None

def tail(path):
    try:
        with path.open('rb') as f:
            f.seek(0, 2)
            f.seek(max(0, f.tell()-131072))
            lines = f.read().splitlines()
        result = []
        for line in lines:
            try:
                result.append(json.loads(line))
            except ValueError:
                pass
        return result
    except OSError:
        return []

def latest_image(run, ego):
    paths = sorted(run.glob('lmdrive/*/meta_%d/*.jpg' % ego))
    # A writer may still be filling the newest image.
    from PIL import Image
    for path in reversed(paths[-3:]):
        try:
            with Image.open(path) as im:
                out = io.BytesIO()
                im.crop((0, 0, 1200, 690)).resize((720, 414)).save(out, 'JPEG')
                return out.getvalue(), path.name
        except (OSError, ValueError):
            continue
    return None, None

def all_rows(path):
    try:
        lines = path.read_text().splitlines()
    except OSError:
        return []
    result = []
    for line in lines:
        try:
            result.append(json.loads(line))
        except ValueError:
            pass
    return result

def full_data(run, baseline):
    ep = run/'episode_000'
    signals = all_rows(ep/'signals.jsonl')
    progress = read(ep/'summary.json') or read(ep/'live.json')
    if not signals or not progress:
        return None
    data = {key: baseline[key] for key in ('manifest', 'training', 'comparison', 'learner', 'adaptation')}
    data.update(signals=signals[::2]+signals[-1:], vehicles=all_rows(ep/'vehicles.jsonl'),
                summary=progress, audit=read(run/'audit.json'), controls=[[], []], cameras=[[], []])
    captures = sorted(run.glob('lmdrive/*/model_audit.jsonl'))
    if captures:
        capture = captures[-1].parent
        audit = all_rows(captures[-1])
        for ego in (0, 1):
            controls = [r for r in audit if r.get('type')=='final_control' and r.get('ego_index')==ego]
            data['controls'][ego] = controls
            previous = -100
            for path in sorted((capture/('meta_%d'%ego)).glob('*.jpg')):
                if not path.stem.isdigit():
                    continue
                step = int(path.stem)+20
                if step>=len(controls) or step-previous<10:
                    continue
                data['cameras'][ego].append(dict(frame=controls[step]['camera_frame'],
                    image='/saved-camera/%d/%s' % (ego, path.stem)))
                previous = step
    data['summary'] = dict(progress, elapsed=signals[-1]['time'])
    return data

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', type=int, default=8768)
    p.add_argument('--carla-port', type=int, default=31600)
    p.add_argument('--run', type=Path)
    p.add_argument('--launch', action='store_true')
    a = p.parse_args()
    if not a.launch and not a.run:
        p.error('Specify --run or --launch')
    run = (a.run or ROOT/'Drive/outputs/cross3'/('live_'+datetime.now().strftime('%Y%m%d_%H%M%S'))).resolve()
    child = None
    log = None
    baseline = read(ROOT/'Drive/outputs/cross3/visualization_data.json')

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, body, mime, status=200):
            self.send_response(status)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            path = urlparse(self.path).path
            if path in ('/', '/complete_process.html'):
                data = full_data(run, baseline) or copy.deepcopy(baseline)
                data['live_view'] = True
                data['waiting'] = not (run/'episode_000/live.json').exists() and not (run/'episode_000/summary.json').exists()
                data['run_path'] = str(run)
                page = (HERE/'full_visualization.html').read_text().replace('__DATA__',
                    json.dumps(data, ensure_ascii=False).replace('<', '\\u003c'))
                return self.reply(page.encode(), 'text/html; charset=utf-8')
            if path == '/full-api':
                data = full_data(run, baseline)
                return self.reply(json.dumps(dict(data=data, run_path=str(run),
                    exit_code=child.poll() if child else None)).encode(), 'application/json')
            if path.startswith('/saved-camera/'):
                parts = path.split('/')
                if len(parts)==4 and parts[2] in ('0', '1') and parts[3].isdigit():
                    paths = sorted(run.glob('lmdrive/*/meta_%s/%s.jpg' % (parts[2], parts[3])))
                    if paths:
                        from PIL import Image
                        try:
                            with Image.open(paths[-1]) as im:
                                out = io.BytesIO()
                                im.crop((0,0,1200,690)).resize((600,345)).save(out,'JPEG')
                            return self.reply(out.getvalue(), 'image/jpeg')
                        except OSError:
                            pass
                return self.reply(b'Waiting for frame', 'text/plain', 404)
            if path in ('/camera/0', '/camera/1'):
                data, _ = latest_image(run, int(path[-1]))
                return self.reply(data or b'Waiting for camera', 'image/jpeg' if data else 'text/plain', 200 if data else 404)
            if path == '/api':
                episodes = sorted(run.glob('episode_*'))
                ep = episodes[-1] if episodes else run/'episode_000'
                signals = tail(ep/'signals.jsonl')
                vehicles = tail(ep/'vehicles.jsonl')
                frame = vehicles[-1]['frame'] if vehicles else None
                summary = read(ep/'summary.json')
                payload = dict(run=str(run), exit_code=child.poll() if child else None,
                    mode='live' if child and child.poll() is None else 'recorded',
                    summary=summary, progress=summary or read(ep/'live.json') or read(ep/'progress.json'),
                    signal=signals[-1] if signals else None,
                    vehicles=[v for v in vehicles if v['frame']==frame],
                    episode=ep.name)
                return self.reply(json.dumps(payload).encode(), 'application/json')
            self.reply(b'Not found', 'text/plain', 404)

    try:
        server = ThreadingHTTPServer(('127.0.0.1', a.port), Handler)
    except OSError as exc:
        if exc.errno != errno.EADDRINUSE:
            raise
        existing = None
        try:
            with urlopen('http://127.0.0.1:%d/api' % a.port, timeout=3) as response:
                candidate = json.load(response)
            existing_run = Path(candidate.get('run', '')).resolve()
            if (ROOT/'Drive/outputs/cross3') in existing_run.parents and all(
                    key in candidate for key in ('mode', 'episode', 'signal', 'vehicles')):
                existing = candidate
        except (OSError, ValueError, TypeError, AttributeError):
            pass
        if existing is None:
            p.exit(1, '端口 %d 已被其他服务占用。未启动实验，也未停止占用进程。\n' % a.port)
        print('Cross3 预览已经启动，本次不重复启动实验。')
        print('在 VS Code「端口」面板转发 %d，然后点击「在浏览器中打开」。' % a.port)
        print('地址：http://127.0.0.1:%d/' % a.port)
        print('已有运行：'+existing['run'])
        print('状态：'+('正在运行' if existing['mode']=='live' else '已停止，网页保留已有数据'))
        if a.run and a.run.resolve() != Path(existing['run']).resolve():
            p.exit(1, '指定目录与现有预览不同；未启动指定实验。\n')
        return
    try:
        if a.launch:
            if run.exists():
                raise RuntimeError('Output already exists: '+str(run))
            with socket.create_connection(('127.0.0.1', a.carla_port), timeout=5):
                pass
            run.parent.mkdir(parents=True, exist_ok=True)
            log = run.with_suffix('.log').open('w')
            child = subprocess.Popen(['bash', str(HERE/'run.sh'), '--port', str(a.carla_port),
                '--egos', '2', '--controller', 'presslight', '--seed', '2027',
                '--checkpoint', str(ROOT/'Drive/models/presslight/cross3_carla_seed2026.pt'),
                '--waypoint-head', str(ROOT/'Drive/models/lmdrive/cross3_waypoint_head.pt'),
                '--output', str(run)], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        print('VS Code: forward port %d; open http://127.0.0.1:%d/' % (a.port, a.port), flush=True)
        print('Run: '+str(run), flush=True)
        if child:
            print('Log: '+str(run.with_suffix('.log')), flush=True)
        print('Ctrl+C stops preview and its experiment. Preview stays open after completion.', flush=True)
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        if child and child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=45)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
        if log:
            log.close()

if __name__ == '__main__':
    main()
