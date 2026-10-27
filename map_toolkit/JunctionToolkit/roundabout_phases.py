"""Eight-group visual demonstration. Rejects vehicles and synchronous owners.

No route-conflict or traffic-safety certification is implied by this demo.
"""
import argparse,hashlib,json,queue,time
from pathlib import Path
import carla


def run(folder,cycles=1,phase=None):
    folder=Path(folder);d=json.loads((folder/'manifest.json').read_text());plan=json.loads((folder/'phase_example.json').read_text())
    c=carla.Client('localhost',2000);c.set_timeout(20);w=c.get_world();w.wait_for_tick(15)
    if hashlib.sha256(w.get_map().to_opendrive().encode()).hexdigest()!=d['opendrive_sha256']:raise RuntimeError('Load this map first')
    def idle():
        if w.get_settings().synchronous_mode or w.get_actors().filter('vehicle.*'):raise RuntimeError('Visual demo stops when vehicles or another tick owner appear')
    idle();lights={a.get_opendrive_id():a for a in w.get_actors().filter('traffic.traffic_light')}
    if set(lights)!=set(d['signals']):raise RuntimeError('Signal set differs')
    rows=[];camera=None
    def set_groups(green,state=carla.TrafficLightState.Green):
        expected={sid:(state if cfg['signal_group'] in green else carla.TrafficLightState.Red) for sid,cfg in d['signals'].items()}
        for sid,l in lights.items():l.freeze(True);l.set_state(expected[sid])
        deadline=time.monotonic()+5
        while True:
            w.wait_for_tick(15)
            if all(lights[s].get_state()==v for s,v in expected.items()):break
            if time.monotonic()>deadline:raise RuntimeError('Light state readback differs: '+str({s:str(lights[s].get_state()) for s in expected}))
        return {s:str(v) for s,v in expected.items()}
    def wait(seconds):
        start=w.get_snapshot().timestamp.elapsed_seconds;deadline=time.monotonic()+max(60,seconds*5)
        while w.get_snapshot().timestamp.elapsed_seconds-start<seconds:
            idle()
            if time.monotonic()>deadline:raise RuntimeError('Simulation stalled')
            w.wait_for_tick(10)
    try:
        if phase:
            p=next(p for p in plan['phases'] if p['name']==phase);states=set_groups(p['green_groups'])
            (folder/'phase_display_state.json').write_text(json.dumps(dict(phase=phase,held=True,states=states,mode='visual_only_no_vehicles'),indent=2))
            print('Held visual phase',phase,'green groups:',p['green_groups']);return
        bp=w.get_blueprint_library().find('sensor.camera.rgb');bp.set_attribute('image_size_x','1400');bp.set_attribute('image_size_y','1400')
        camera=w.spawn_actor(bp,carla.Transform(carla.Location(0,0,105),carla.Rotation(pitch=-90,yaw=-90,roll=0)));q=queue.Queue(maxsize=2)
        def receive(frame):
            try:q.put_nowait(frame)
            except queue.Full:
                try:q.get_nowait()
                except queue.Empty:pass
                try:q.put_nowait(frame)
                except queue.Full:pass
        camera.listen(receive)
        for cycle in range(cycles):
            for p in plan['phases']:
                idle();states=set_groups(p['green_groups']);frame_id=w.get_snapshot().frame
                image=q.get(timeout=15)
                while image.frame<frame_id+3:image=q.get(timeout=15)
                image.save_to_disk(str(folder/('phase_'+p['name']+'.png')))
                rows.append(dict(cycle=cycle,phase=p['name'],green_groups=p['green_groups'],states=states,frame=frame_id,seconds=p['seconds']))
                print('Phase',p['name'],'green',p['green_groups'],'seconds',p['seconds'],flush=True)
                wait(p['seconds']);set_groups(p['green_groups'],carla.TrafficLightState.Yellow);wait(plan['yellow_seconds']);set_groups([]);wait(plan['all_red_seconds'])
        (folder/'phase_demo_validation.json').write_text(json.dumps(dict(passed=True,phases=rows,phase_example=plan,traffic_test=False),indent=2))
    finally:
        if camera:camera.stop();camera.destroy()
        if not phase:set_groups([])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('map_dir');p.add_argument('--cycles',type=int,default=1);p.add_argument('--phase',choices=['A','B','C','D']);a=p.parse_args()
    if a.cycles<1:p.error('cycles must be positive')
    run(a.map_dir,a.cycles,a.phase)
