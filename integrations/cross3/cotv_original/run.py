#!/usr/bin/env python3
"""Launch upstream CoTV without substituting its environment or PPO algorithm."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
FILES = ('flow/examples/train_ppo.py',
         'flow/examples/exp_configs/rl/multiagent/CoTV_grid.py',
         'flow/flow/envs/multiagent/traffic_light_grid.py',
         'flow/flow/networks/traffic_light_grid.py',
         'flow/environment.yml', 'flow/requirements.txt')


def main():
    p = argparse.ArgumentParser(__doc__)
    p.add_argument('--source', type=Path, default=Path('/home/zlj/Drive/repos/CoTV'))
    p.add_argument('--python', default=str(ROOT / 'outputs/cotv_original/env/bin/python'))
    p.add_argument('--sumo-home', type=Path)
    p.add_argument('--iterations', type=int, default=1)
    p.add_argument('--run-dir', type=Path, default=ROOT / 'outputs/cotv_original/smoke')
    p.add_argument('--check', action='store_true')
    a = p.parse_args()
    source = a.source.resolve()
    missing = [f for f in FILES if not (source / f).is_file()]
    if missing:
        p.error('Missing upstream files: ' + ', '.join(missing))
    if a.iterations < 1:
        p.error('iterations must be positive')
    manifest = {'source': str(source), 'files': {
        f: hashlib.sha256((source / f).read_bytes()).hexdigest() for f in FILES},
        'iterations': a.iterations, 'runtime_errors': []}
    rev = subprocess.run(['git', '-C', str(source), 'rev-parse', 'HEAD'],
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    manifest['upstream_commit'] = rev.stdout.strip()
    python = shutil.which(a.python)
    if not python:
        manifest['runtime_errors'].append('Missing isolated Python 3.7: ' + a.python)
    else:
        check = subprocess.run([python, '-c',
            'import sys; assert sys.version_info[:2] == (3,7); '
            'import ray,tensorflow,gym; '
            'assert ray.__version__ == "0.8.0"; '
            'assert tensorflow.__version__ == "1.15.2"; '
            'assert gym.__version__ == "0.14.0"'],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if check.returncode:
            manifest['runtime_errors'].append(check.stderr)
    if not a.sumo_home or not (a.sumo_home / 'bin/sumo').is_file():
        manifest['runtime_errors'].append('SUMO 1.10.0 installation required via --sumo-home')
    else:
        v = subprocess.check_output([str(a.sumo_home / 'bin/sumo'), '--version'], text=True)
        manifest['sumo_version'] = v
        if 'Version 1.10.0' not in v:
            manifest['runtime_errors'].append('Exact reproduction requires SUMO 1.10.0')
    a.run_dir.mkdir(parents=True, exist_ok=True)
    (a.run_dir / 'manifest.json').write_text(json.dumps(manifest, indent=2))
    print(json.dumps(manifest, indent=2), flush=True)
    if manifest['runtime_errors']:
        return 2
    if a.check:
        return 0
    # Use an unchanged private copy so Flow writes generated files here, not upstream.
    snapshot = a.run_dir.resolve() / 'CoTV'
    if snapshot.exists():
        p.error('Run directory already contains a snapshot; choose a new run directory')
    shutil.copytree(source, snapshot, ignore=shutil.ignore_patterns('.git', '__pycache__'))
    env = os.environ.copy()
    env['PYTHONPATH'] = str(snapshot / 'flow')
    env['SUMO_HOME'] = str(a.sumo_home.resolve())
    env['PATH'] = str(Path(python).parent) + ':' + str(a.sumo_home.resolve() / 'bin') + ':' + env['PATH']
    env['TUNE_RESULT_DIR'] = str(a.run_dir.resolve() / 'ray_results')
    command = [python, 'train_ppo.py', 'CoTV_grid', '--num_steps', str(a.iterations)]
    with (a.run_dir / 'train.log').open('w') as log:
        return subprocess.call(command, cwd=str(snapshot / 'flow/examples'), env=env,
                               stdout=log, stderr=subprocess.STDOUT)

if __name__ == '__main__':
    sys.exit(main())
