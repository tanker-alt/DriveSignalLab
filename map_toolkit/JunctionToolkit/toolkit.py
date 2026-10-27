"""One entry point dispatching to separately versioned generator/CARLA environments."""
import argparse
import json
import subprocess
import sys
from pathlib import Path
from runtime_config import settings as load_settings, python_for, map_output


def main():
    root=Path(__file__).resolve().parent
    settings=load_settings()
    p=argparse.ArgumentParser(description=__doc__)
    commands=p.add_subparsers(dest='command',required=True)
    g=commands.add_parser('generate');g.add_argument('--config',required=True);g.add_argument('--out',required=True)
    ring=commands.add_parser('roundabout',help='Experimental two-lane ring with four two-in/two-out arms and internal signals')
    ring.add_argument('--out',required=True);ring.add_argument('--radius',type=float,default=30)
    ring.add_argument('--arm-length',type=float,default=120);ring.add_argument('--lane-width',type=float,default=3.5)
    for cmd in ['validate','view','drive-check','experiment']:
        sub=commands.add_parser(cmd);sub.add_argument('map_dir')
    commands.add_parser('doctor')
    a,rest=p.parse_known_args()
    def call(env,script,args):
        exe=python_for(env)
        if not Path(exe).is_file():raise ValueError('Python not found; edit settings.json: '+exe)
        command=[exe,'-u','-B',str(root/script)]+args
        if script!='view.py':
            subprocess.run(command,check=True)
            return
        # CARLA's native logger bypasses Python logging. Filter only this
        # exact informational cache message, never warnings or exceptions.
        cache_hint=b'INFO: Found the required file in cache! Carla/Maps/Nav/Town05.bin'
        with subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT) as child:
            try:
                for line in child.stdout:
                    if b' '.join(line.split())==cache_hint:
                        continue
                    sys.stdout.buffer.write(line)
                    sys.stdout.buffer.flush()
                code=child.wait()
                if code:
                    raise subprocess.CalledProcessError(code,command)
            except BaseException:
                if child.poll() is None:
                    child.terminate()
                    try:child.wait(timeout=10)
                    except subprocess.TimeoutExpired:child.kill();child.wait()
                raise
    if a.command=='doctor':
        for env,module in [('generator','scenariogeneration'),('carla','carla')]:
            subprocess.run([python_for(env),'-c','import sys, '+module+'; print(sys.version); print('+module+'.__file__)'],check=True)
        print('Expected: generator Python 3.12 + scenariogeneration 0.16.7; CARLA Python 3.7 + carla/server 0.9.15')
    elif a.command=='roundabout':
        map_output(a.out)
        if rest:p.error('Unknown roundabout options: '+' '.join(rest))
        call('generator','roundabout.py',['--out',a.out,'--radius',str(a.radius),'--arm-length',str(a.arm_length),'--lane-width',str(a.lane_width)])
        call('carla','validate_roundabout.py',[a.out])
    elif a.command=='generate':
        map_output(a.out)
        if rest:p.error('Unknown generation options: '+' '.join(rest))
        call('generator','generate.py',['--config',a.config,'--out',a.out])
        call('carla','validate.py',[a.out])
    else:
        script={'validate':'validate.py','view':'view.py','drive-check':'drive_check.py','experiment':'experiment.py'}[a.command]
        if a.command=='experiment' and not settings.get('project'):
            raise ValueError('External evaluation integration is disabled; explicitly configure project in settings.local.json to use it')
        extra=['--project',str((root/settings['project']).resolve())] if a.command=='experiment' else []
        call('carla',script,[a.map_dir]+extra+rest)


if __name__=='__main__':
    try:main()
    except subprocess.CalledProcessError as exc:raise SystemExit(exc.returncode)
