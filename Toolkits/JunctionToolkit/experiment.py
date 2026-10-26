"""Run the existing project's single-junction evaluation on a generated map.

Exclusive server use required. Ctrl+C is handled; forcibly closing a console
or killing the Python process cannot guarantee Python finally cleanup.
"""
import argparse
import datetime
import json
import subprocess
import sys
import time
from pathlib import Path
import carla
from project_bridge import World


def run(args):
    folder=Path(args.map_dir).resolve();tool=Path(__file__).resolve().parent
    design=json.loads((folder/'manifest.json').read_text())
    if len(design['junctions'])!=1:
        raise ValueError('Existing project experiment is single-junction only. Use drive_check.py for connected grid smoke validation.')
    if args.vehicles_per_approach<1 or args.vehicles_per_approach>2*design['config']['incoming_lanes']:
        raise ValueError('This launcher supports up to two cars per incoming lane')
    c=carla.Client('localhost',2000);c.set_timeout(15);w=c.get_world()
    if w.get_settings().synchronous_mode:raise RuntimeError('Previous experiment still owns synchronous mode. Do not start a second controller.')
    w.wait_for_tick(15)
    if w.get_actors().filter('vehicle.*') or w.get_actors().filter('sensor.*'):raise RuntimeError('Active vehicles/sensors found')
    World(w,design).get_traffic_lights_in_junction(design['junctions'][0]['id'])
    original=w.get_settings();baseline={a.id for a in w.get_actors()}
    out=Path(args.output_root or (folder/'experiments'))/datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f');out.mkdir(parents=True)
    profile=out/'profile';bridge=str(tool/'project_bridge.py')
    common=[sys.executable,'-u','-B',bridge]
    mapargs=['--map-dir',str(folder),'--project',args.project]
    subprocess.run(common+['prepare']+mapargs+['--junction-id',str(design['junctions'][0]['id']),'--prepare','--output',str(profile)],check=True)
    children=[];report=dict(status='starting',output=str(out));co=None;so=None
    try:
        co=(out/'controller.log').open('w');so=(out/'spawn.log').open('w')
        controller=subprocess.Popen(common+['control']+mapargs+['--profile',str(profile/'profile.json'),
            '--collect','--collisions','--cycles','5','--collection-output',str(out/'runs'),'--output',str(out/'phases.csv')],stdout=co,stderr=subprocess.STDOUT)
        children.append(controller);deadline=time.monotonic()+45
        while True:
            log=(out/'controller.log').read_text(errors='replace')
            if controller.poll() is not None:raise RuntimeError('Controller startup failed:\n'+log[-5000:])
            if 'Frame collection output:' in log:break
            if time.monotonic()>deadline:raise RuntimeError('Controller readiness timeout; inspect '+str(out/'controller.log'))
            time.sleep(.1)
        spawn=subprocess.Popen(common+['spawn']+mapargs+['--profile',str(profile/'profile.json'),
            '--vehicles-per-approach',str(args.vehicles_per_approach),'--vehicle-blueprint',args.blueprint,
            '--first-distance','65','--seed',str(args.seed),'--route','random','--duration-seconds',str(max(140,len(design['signals'])*17+100)),'--output',str(out/'traffic.csv')],stdout=so,stderr=subprocess.STDOUT)
        children.append(spawn);print('Running; wait for Exit codes: 0 0. Output:',out,flush=True)
        deadline=time.monotonic()+args.timeout
        while any(p.poll() is None for p in children):
            if any(p.poll() not in (None,0) for p in children):raise RuntimeError('Child failed; inspect logs in '+str(out))
            if time.monotonic()>deadline:raise RuntimeError('Wall-clock timeout')
            time.sleep(.25)
        report.update(status='completed',controller_exit=controller.returncode,spawn_exit=spawn.returncode)
        print('Exit codes:',controller.returncode,spawn.returncode)
    except BaseException as exc:
        report.update(status='aborted',error=str(exc));raise
    finally:
        for p in children:
            if p.poll() is None:p.terminate()
        for p in children:
            try:p.wait(timeout=10)
            except subprocess.TimeoutExpired:p.kill();p.wait()
        if co:co.close()
        if so:so.close()
        try:
            # Server is exclusively owned; remove only newly created experiment
            # vehicles/collision listeners. Keep scenery and all pre-existing actors.
            for kind in ['sensor.other.collision','vehicle.*']:
                for actor in w.get_actors().filter(kind):
                    if actor.id not in baseline:actor.destroy()
            c.get_trafficmanager(8000).set_synchronous_mode(False)
            for light in w.get_actors().filter('traffic.traffic_light'):light.freeze(False)
            w.apply_settings(original);report['world_settings_restored']=True
        except Exception as exc:report['cleanup_error']=str(exc)
        (out/'processes.json').write_text(json.dumps(report,indent=2))
    return out


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('map_dir')
    p.add_argument('--project',default=str(Path(__file__).resolve().parents[2]/'InformationCollector'))
    p.add_argument('--vehicles-per-approach',type=int,default=3);p.add_argument('--blueprint',default='vehicle.tesla.model3')
    p.add_argument('--output-root')
    p.add_argument('--seed',type=int,default=17);p.add_argument('--timeout',type=float,default=600)
    run(p.parse_args())
