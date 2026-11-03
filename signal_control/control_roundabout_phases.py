"""Configurable eight-group roundabout signal controller."""
import argparse,datetime,hashlib,json,math,time
from pathlib import Path
import carla

ROOT=Path(__file__).resolve().parent
DEFAULT_MAP=ROOT.parent/'Maps/MapDataset/roundabout4_8groups'

def load_plan(map_dir,path=None):
    d=json.loads((Path(map_dir)/'manifest.json').read_text(encoding='utf-8-sig'))
    digest=hashlib.sha256((Path(map_dir)/'map.xodr').read_bytes()).hexdigest()
    if digest!=d.get('opendrive_sha256'): raise ValueError('Map fingerprint differs')
    source=Path(path) if path else Path(map_dir)/'phase_example.json'
    raw=json.loads(source.read_text(encoding='utf-8-sig'))
    if raw.get('opendrive_sha256') and raw['opendrive_sha256']!=digest: raise ValueError('Plan fingerprint differs')
    groups={int(c['signal_group']) for c in d['signals'].values()}; phases=[]; names=set()
    for p in raw.get('phases',[]):
        name=str(p.get('name','')).strip(); green=sorted({int(x) for x in p.get('green_groups',[])})
        if not name or name in names or not green or not set(green)<=groups: raise ValueError('Invalid phase: '+repr(p))
        names.add(name)
        vals=(p.get('green_seconds',p.get('seconds')),p.get('yellow_seconds',raw.get('yellow_seconds',3)),p.get('all_red_seconds',raw.get('all_red_seconds',1)))
        if any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) or x<=0 for x in vals): raise ValueError('Invalid timings: '+name)
        phases.append(dict(name=name,green_groups=green,green_seconds=float(vals[0]),yellow_seconds=float(vals[1]),all_red_seconds=float(vals[2])))
    if not phases or set().union(*(set(p['green_groups']) for p in phases))!=groups: raise ValueError('Phase groups do not cover all signals')
    return d,dict(schema_version='roundabout_phase_plan.1',source=str(source),opendrive_sha256=digest,phases=phases,signal_groups=sorted(groups))

def states(design,phase,stage):
    active=set(phase['green_groups']) if stage!='ALL_RED' else set()
    return {str(s):stage.title() if int(c['signal_group']) in active else 'Red' for s,c in design['signals'].items()}

def run(a):
    d,plan=load_plan(a.map_dir,a.phase_config); c=carla.Client(a.host,a.port); c.set_timeout(20); w=c.get_world(); wid=w.id
    if hashlib.sha256(w.get_map().to_opendrive().encode()).hexdigest()!=plan['opendrive_sha256']: raise RuntimeError('Load specified roundabout first')
    if w.get_settings().synchronous_mode: raise RuntimeError('World already synchronous')
    if w.get_actors().filter('vehicle.*') and not a.allow_vehicles: raise RuntimeError('Vehicles present (use --allow-vehicles for traffic test)')
    lights={str(x.get_opendrive_id()):x for x in w.get_actors().filter('traffic.traffic_light')}
    if set(lights)!=set(d['signals']): raise RuntimeError('Signal set differs')
    for sid,l in lights.items():
        cfg=d['signals'][sid]; actual={(x.road_id,x.lane_id) for x in l.get_affected_lane_waypoints()}; expected={(cfg['road_id'],x) for x in cfg['incoming_lanes']}
        if actual!=expected: raise RuntimeError('Lane validity differs: '+sid)
    out=Path(a.output) if a.output else ROOT/'evaluation/outputs/roundabout_control'/datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'); out.mkdir(parents=True,exist_ok=True)
    original=w.get_settings(); saved=[]
    for l in lights.values(): saved.append((l,l.get_state(),l.is_frozen(),l.get_green_time(),l.get_yellow_time(),l.get_red_time())); l.set_green_time(1e9);l.set_yellow_time(1e9);l.set_red_time(1e9)
    def apply(expected):
        for sid,v in expected.items(): lights[sid].freeze(True);lights[sid].set_state(getattr(carla.TrafficLightState,v))
        c.apply_batch_sync([],False)
    def restore():
        for l,state,frozen,g,y,r in saved: l.set_green_time(g);l.set_yellow_time(y);l.set_red_time(r);l.set_state(state);l.freeze(frozen)
        c.apply_batch_sync([],False)
    report=dict(schema_version='roundabout_control.2',map_hash=plan['opendrive_sha256'],plan=plan,timings_override=dict(green=a.green,yellow=a.yellow,all_red=a.all_red),rows=[],traffic_validated=False)
    try:
        s=w.get_settings();s.synchronous_mode=True;s.fixed_delta_seconds=.05;w.apply_settings(s)
        for cycle in range(a.cycles or 1):
            for p in plan['phases']:
                for stage,base in (('GREEN',p['green_seconds']),('YELLOW',p['yellow_seconds']),('ALL_RED',p['all_red_seconds'])):
                    duration={'GREEN':a.green or base,'YELLOW':a.yellow or base,'ALL_RED':a.all_red or base}[stage]; expected=states(d,p,stage);apply(expected);start=w.get_snapshot().timestamp.elapsed_seconds; mismatches=0
                    while w.get_snapshot().timestamp.elapsed_seconds-start<duration:
                        if c.get_world().id!=wid: raise RuntimeError('World changed')
                        snap=w.tick(10); actual={sid:str(l.get_state()) for sid,l in lights.items()}
                        if a.realtime: time.sleep(0.05)
                        if actual!=expected: mismatches+=1;raise RuntimeError('Readback mismatch at frame {}'.format(snap.frame))
                    report['rows'].append(dict(cycle=cycle+1,phase=p['name'],stage=stage,duration_seconds=duration,expected=expected,readback_mismatches=mismatches));print(p['name'],stage,duration,flush=True)
        report['finished']=True
    except KeyboardInterrupt: report['stopped_by_user']=True
    except BaseException as e: report['error']=str(e);raise
    finally:
        if c.get_world().id==wid: apply({sid:'Red' for sid in lights});restore();w.apply_settings(original);report['restored']=True
        else: report['restored']=False
        (out/'controller.json').write_text(json.dumps(report,indent=2),encoding='utf-8');print('Controller report:',out/'controller.json',flush=True)

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--map-dir',type=Path,default=DEFAULT_MAP);p.add_argument('--phase-config',type=Path);p.add_argument('--host',default='localhost');p.add_argument('--port',type=int,default=2000);p.add_argument('--cycles',type=int,default=1);p.add_argument('--green',type=float);p.add_argument('--yellow',type=float);p.add_argument('--all-red',type=float);p.add_argument('--output');p.add_argument('--allow-vehicles',action='store_true',help='Allow existing random traffic during signal test');p.add_argument('--realtime',action='store_true',help='Pace visual experiment in real time');a=p.parse_args()
    if a.cycles<0 or any(x is not None and (not math.isfinite(x) or x<=0) for x in (a.green,a.yellow,a.all_red)):p.error('Invalid cycles or timings')
    run(a)
if __name__=='__main__':main()
