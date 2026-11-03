"""Release gate: actual vehicle motion, all-green isolated routes, project preflight."""
import argparse,collections,hashlib,json,math,sys,time
from pathlib import Path
import carla
from project_bridge import World

def find_route(m,src,lane,s,dst,target_lane,target_s,max_nodes=12000):
 start=m.get_waypoint_xodr(src,lane,s)
 todo=collections.deque([(start,[])]);seen=set()
 while todo:
  w,h=todo.popleft();key=(w.road_id,w.lane_id,round(w.s))
  if key in seen:continue
  seen.add(key);path=h+[w]
  if w.road_id==dst and w.lane_id==target_lane and abs(w.s-target_s)<2.1:return path
  if len(seen)>max_nodes:break
  for n in w.next(2.):todo.append((n,path))
 raise ValueError('No route %s %s -> %s %s'%(src,lane,dst,target_lane))

def run(folder,project):
 p=Path(folder);d=json.loads((p/'manifest.json').read_text());raw=(p/'map.xodr').read_bytes()
 client=carla.Client('127.0.0.1',2000);client.set_timeout(90);w=client.get_world()
 if w.get_settings().synchronous_mode or any(w.get_actors().filter(k) for k in ('vehicle.*','sensor.*','walker.*')):raise RuntimeError('Active experiment')
 w=client.generate_opendrive_world(raw.decode(),carla.OpendriveGenerationParameters(vertex_distance=.5,max_road_length=30.,wall_height=0.,additional_width=0.,smooth_junctions=False,enable_mesh_visibility=True,enable_pedestrian_navigation=False))
 m=w.get_map();assert hashlib.sha256(m.to_opendrive().encode()).hexdigest()==d['opendrive_sha256']
 lights=list(w.get_actors().filter('traffic.traffic_light')); assert {l.get_opendrive_id() for l in lights}==set(d['signals'])
 sys.path[:0]=[str(Path(project)),str(Path(project)/'InformationCollector')]
 from Toolkits.CollectionTools.map_setup import inspect_junction,prepare_profile
 from evaluation.map_profile import read_profile
 proxy=World(w,d);preflight=[]
 profiles=p/'experiment_profiles';profiles.mkdir(exist_ok=True)
 for j in d['junctions']:
  result,_=inspect_junction(proxy,j['id']);preflight.append(result)
  if result['status']=='ready':
   dest=profiles/('junction_'+str(j['id']))
   if not dest.exists():prepare_profile(proxy,j['id'],dest)
   profile_path=dest/'profile.json'
   profile=json.loads(profile_path.read_text(encoding='utf-8'))
   profile['generated_map']=dict(manifest='../../manifest.json',sha256=hashlib.sha256((p/'manifest.json').read_bytes()).hexdigest())
   profile_path.write_text(json.dumps(profile,indent=2),encoding='utf-8')
   read_profile(profile_path)
 (p/'project_preflight.json').write_text(json.dumps(dict(junctions=preflight,project=project),indent=2))
 lengths={rid:length for j in d['junctions'] for rid,length in zip(j['roads'],j['lengths'])}
 cases=[]
 if d['config'].get('type')=='roundabout':
  for src in range(1,5):
   for lane in (-1,-2):
    for dst in range(1,5):cases.append(dict(kind='roundabout',source=src,lane=lane,start_s=lengths[src]-20,target=dst,target_lane=-lane,target_s=lengths[dst]-20))
 else:
  for x in d['movements']:cases.append(dict(kind='movement',source=x['road'],lane=x['lane'],start_s=lengths[x['road']]-20,target=x['target_road'],target_lane=x['target_lane'],target_s=lengths[x['target_road']]-20))
  for s in d.get('seams',[]):
   for src,dst in ((s['road_a'],s['road_b']),(s['road_b'],s['road_a'])):
    for lane in range(1,d['config']['incoming_lanes']+1):cases.append(dict(kind='seam',source=src,lane=lane,start_s=15,target=dst,target_lane=-lane,target_s=15))
 original=w.get_settings();states=[(l,l.get_state(),l.is_frozen()) for l in lights];records=[];v=None;sensor=None
 report=dict(passed=False,opendrive_sha256=d['opendrive_sha256'],expected_cases=len(cases),cases=records,preflight_ready=all(x['status']=='ready' for x in preflight),scope='Every manifest movement and every seam lane in both directions; roundabout all 32 entry/lane/exit cases. One physics-enabled Model3, all green, pure-pursuit 5m/s. Not multi-agent conflict or calibrated metric certification.')
 print(p.name,'project statuses',[(r['junction_id'],r['status']) for r in preflight],'physical cases',len(cases),flush=True)
 try:
  s=w.get_settings();s.synchronous_mode=True;s.fixed_delta_seconds=.05;s.no_rendering_mode=True;w.apply_settings(s)
  for l in lights:l.set_state(carla.TrafficLightState.Green);l.freeze(True)
  for i,case in enumerate(cases):
   points=find_route(m,case['source'],case['lane'],case['start_s'],case['target'],case['target_lane'],case['target_s'])
   rec=dict(case,passed=False,collisions=[],min_z=999,max_tracking_error=0);records.append(rec)
   try:
    tr=points[0].transform;tr.location.z+=.35
    v=w.spawn_actor(w.get_blueprint_library().find('vehicle.tesla.model3'),tr)
    sensor=w.spawn_actor(w.get_blueprint_library().find('sensor.other.collision'),carla.Transform(),attach_to=v)
    sensor.listen(lambda e,r=rec:r['collisions'].append(dict(frame=e.frame,other=e.other_actor.type_id,impulse=e.normal_impulse.length())))
    for _ in range(10):w.tick()
    cursor=0;deadline=time.monotonic()+90
    for tick in range(2200):
     if time.monotonic()>deadline:rec['error']='wall_timeout';break
     loc=v.get_location();tr=v.get_transform();speed=v.get_velocity().length()
     cursor=min(range(cursor,min(cursor+12,len(points))),key=lambda k:loc.distance(points[k].transform.location))
     rec['max_tracking_error']=max(rec['max_tracking_error'],loc.distance(points[cursor].transform.location))
     goal=points[min(cursor+3,len(points)-1)].transform.location;dx=goal.x-loc.x;dy=goal.y-loc.y;yaw=math.radians(tr.rotation.yaw)
     lateral=-dx*math.sin(yaw)+dy*math.cos(yaw);steer=math.atan2(2*2.875*lateral,max(dx*dx+dy*dy,.1))/math.radians(70)
     v.apply_control(carla.VehicleControl(throttle=max(0.,min(.6,.25*(5-speed))),brake=max(0.,min(.5,.3*(speed-5))),steer=max(-1.,min(1.,steer))))
     w.tick();loc=v.get_location();rec['min_z']=min(rec['min_z'],loc.z)
     if loc.distance(points[-1].transform.location)<2.5 and cursor>=len(points)-4:rec['completed']=True;break
     if loc.z<-.5:rec['error']='fell_below_surface';break
    for _ in range(3):w.tick()
    rec['passed']=rec.get('completed',False) and rec['min_z']>-.5 and not rec['collisions'] and rec['max_tracking_error']<2.0
   finally:
    if sensor:sensor.stop();sensor.destroy();sensor=None
    if v:v.destroy();v=None
    w.tick()
   if not rec['passed']:print('FAILED',rec,flush=True)
   if (i+1)%10==0 or i+1==len(cases):print(p.name,i+1,'/',len(cases),'failures',sum(not x['passed'] for x in records),flush=True)
   (p/'release_drive_validation.json').write_text(json.dumps(report,indent=2))
  report['passed']=len(records)==len(cases) and all(r['passed'] for r in records)
 finally:
  if sensor:sensor.stop();sensor.destroy()
  if v:v.destroy()
  for l,state,frozen in states:l.set_state(state);l.freeze(frozen)
  w.apply_settings(original);report['settings_restored']=True
  (p/'release_drive_validation.json').write_text(json.dumps(report,indent=2))
 print('FINISHED',p.name,report['passed'],flush=True)
 return report

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('folder');p.add_argument('--project',default='D:/CoDriveSignal');a=p.parse_args();run(a.folder,a.project)
