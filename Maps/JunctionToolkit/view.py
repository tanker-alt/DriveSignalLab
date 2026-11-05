"""Load a validated map, decorate it and capture real RGB images (CARLA Python 3.7)."""
import argparse
import hashlib
import json
import queue
import time
from pathlib import Path
import carla
from scenery import build
from lane_signals import build as build_lane_signals
from safety_rules import load_rules, draw_stop_lines
from tree_scenery import build as build_trees


def capture(world, folder, center, radius, overview_center=None, wide=False):
    bp=world.get_blueprint_library().find('sensor.camera.rgb')
    bp.set_attribute('image_size_x','1280');bp.set_attribute('image_size_y','960');bp.set_attribute('fov','90')
    overview_center=overview_center or center
    for name, transform in [
        ('overview',carla.Transform(carla.Location(x=overview_center[0],y=-overview_center[1],z=radius),carla.Rotation(pitch=-90,yaw=-90))),
        ('street',carla.Transform(carla.Location(x=center[0]+38,y=-center[1]-55,z=43),carla.Rotation(pitch=-33,yaw=125)))]:
        bp.set_attribute('image_size_x','1920' if wide and name=='overview' else '1280')
        bp.set_attribute('image_size_y','640' if wide and name=='overview' else '960')
        q=queue.Queue();camera=world.spawn_actor(bp,transform)
        try:
            camera.listen(q.put)
            deadline=time.monotonic()+3
            while True:
                frame=q.get(timeout=20)
                if time.monotonic()>=deadline:break
            frame.save_to_disk(str(folder/(name+'.png')))
        finally:camera.stop();camera.destroy()


def decorate(world, design):
    # Keep sidewalks clear: no random furniture or roadside garden lamps.
    return [], []


def load(folder, host='localhost',port=2000,photos=True,hidden_load=True,beautify='auto'):
    folder=Path(folder);design=json.loads((folder/'manifest.json').read_text())
    if isinstance(beautify, bool):
        beautify = 'complete' if beautify else 'off'
    if beautify not in ('auto','simple','complete','off'):
        raise ValueError('Unknown appearance mode: '+str(beautify))
    if beautify == 'auto':
        beautify = 'simple' if design['config'].get('type') != 'roundabout' and design['config'].get('rows',1)*design['config'].get('columns',1)>1 else 'complete'
    if design['config'].get('type')=='roundabout':
        from roundabout_view import load as load_ring
        return load_ring(folder,host,port,photos,beautify=beautify != 'off',mode=beautify)
    if design['config'].get('rows',1)*design['config'].get('columns',1)>1:
        from network_preview import main as load_network
        return load_network(folder,host,port,photos,mode=beautify)
    validation=json.loads((folder/'topology_validation.json').read_text())
    if not validation['passed']:raise RuntimeError('Topology validation failed')
    raw=(folder/'map.xodr').read_bytes()
    if hashlib.sha256(raw).hexdigest()!=design['opendrive_sha256']:raise RuntimeError('Map changed after generation')
    safety_rules=load_rules(folder)
    c=carla.Client(host,port);c.set_timeout(120);w=c.get_world()
    # Do not trust empty actor snapshots in a stalled synchronous world.
    if w.get_settings().synchronous_mode:raise RuntimeError('World is synchronous: finish the current run before loading a map')
    w.wait_for_tick(15)
    if any(w.get_actors().filter(p) for p in ['vehicle.*','sensor.*','walker.*']):
        raise RuntimeError('Active vehicles/sensors/walkers: finish the current experiment first')
    original_settings=w.get_settings()
    if hidden_load:
        hidden_settings=w.get_settings()
        hidden_settings.synchronous_mode=False
        hidden_settings.no_rendering_mode=True
        w.apply_settings(hidden_settings)
        w.wait_for_tick(15)
    print('Loading OpenDRIVE:', folder, flush=True)
    w=c.generate_opendrive_world(raw.decode('utf-8'),carla.OpendriveGenerationParameters(
        vertex_distance=.5 if design['config'].get('standard_profile') else 2.,max_road_length=50.,wall_height=0.,additional_width=0.,
        smooth_junctions=False,enable_mesh_visibility=True,enable_pedestrian_navigation=False))
    if hidden_load:
        hidden_settings=w.get_settings()
        hidden_settings.no_rendering_mode=True
        w.apply_settings(hidden_settings)
    for _ in range(10):w.wait_for_tick(15)
    import xml.etree.ElementTree as ET
    def node(e): return (e.tag, sorted(e.attrib.items()), (e.text or '').strip(), [node(c) for c in e])
    if node(ET.fromstring(w.get_map().to_opendrive())) != node(ET.fromstring(raw.decode('utf-8-sig'))):
        raise RuntimeError('Server loaded different OpenDRIVE content')
    lights=list(w.get_actors().filter('traffic.traffic_light'))
    by_id={a.get_opendrive_id():a for a in lights}
    if set(by_id)!=set(design['signals']):raise RuntimeError('Actual signal IDs differ from map manifest')
    for sid,cfg in design['signals'].items():
        actual={(p.road_id,p.lane_id) for p in by_id[sid].get_affected_lane_waypoints()}
        if actual!={(cfg['road_id'],lane) for lane in cfg['incoming_lanes']}:raise RuntimeError('Signal lane validity mismatch: '+sid)
    w.set_weather(carla.WeatherParameters(cloudiness=25,precipitation=0,wetness=0,sun_altitude_angle=48,sun_azimuth_angle=110))
    scenery=dict(actor_ids=[],enabled=False);lane_signals=None;ids=[];failures=[];stop_lines=None
    trees=dict(count=0,actor_ids=[],rejected={},skipped=True)
    if beautify != 'off':
        if beautify == 'simple':
            from network_simple_finish import build as build_simple
            scenery=build_simple(w,design)
        elif design['config'].get('standard_profile') == 'urban_cross3_v1':
            from runtime_finish import build as build_finish
            print('Adding ground and crossing paint', flush=True)
            scenery=build_finish(w,design,c)
        else:
            scenery=build(w,design,safety_rules)
        print('Adding signal masts', flush=True)
        lane_signals=build_lane_signals(w,design)
        ids,failures=decorate(w,design)
        stop_lines=draw_stop_lines(w,safety_rules) if beautify == 'complete' and safety_rules is not None and design['config'].get('standard_profile') != 'urban_cross3_v1' else None
        print('Adding roadside trees' if design['config']['rows']*design['config']['columns']==1 else 'Network standard: trees disabled', flush=True)
        trees=build_trees(w,design,c) if beautify == 'complete' and design['config']['rows']*design['config']['columns']==1 else dict(count=0,actor_ids=[],rejected={},skipped=True)
    center=design['junctions'][0]['center']
    w.get_spectator().set_transform(carla.Transform(carla.Location(x=center[0]-48,y=-center[1]-38,z=27),carla.Rotation(pitch=-24,yaw=38)))
    report=dict(beautify=beautify != 'off',appearance_mode=beautify,map_hash=design['opendrive_sha256'],signal_count=len(lights),signal_validity_passed=True,
                decoration_ids=ids,decoration_failures=failures,debug_overlays=False,scenery=scenery,trees=trees,lane_signals=lane_signals,stop_lines=stop_lines)
    if hidden_load:
        visible=w.get_settings()
        visible.synchronous_mode=original_settings.synchronous_mode
        visible.no_rendering_mode=False
        visible.fixed_delta_seconds=None
        visible.substepping=original_settings.substepping
        visible.max_substep_delta_time=original_settings.max_substep_delta_time
        visible.max_substeps=original_settings.max_substeps
        w.apply_settings(visible)
        w.wait_for_tick(15)
        report['visibility_transition']='hidden until validation and decoration completed'
    (folder/'load_report.json').write_text(json.dumps(report,indent=2))
    if photos:
        print('Capturing RGB previews', flush=True)
        cfg=design['config']
        overview_center=[(cfg['columns']-1)*cfg['spacing_m']/2,(cfg['rows']-1)*cfg['spacing_m']/2]
        wide = cfg['columns']>=3*cfg['rows']
        extent=cfg['arm_length_m']+cfg['junction_radius_m']+40
        span_x=(cfg['columns']-1)*cfg['spacing_m']+2*extent
        span_y=(cfg['rows']-1)*cfg['spacing_m']+2*extent
        height=100 if len(design['junctions'])==1 else 1.1*max(span_x/2,span_y*(3 if wide else 4/3)/2)
        capture(w,folder,center,height,overview_center,wide)
    print('Loaded:',folder,'lights:',len(lights),'roadside props:',len(ids),'failed props:',len(failures))
    return w,design


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('map_dir');p.add_argument('--host',default='localhost');p.add_argument('--port',type=int,default=2000);p.add_argument('--no-photos',action='store_true');p.add_argument('--show-during-load',action='store_true')
    p.add_argument('--beautify',choices=('auto','simple','complete','off'),default='auto',help='auto: complete for a single junction, simple for a network')
    a=p.parse_args();load(a.map_dir,a.host,a.port,not a.no_photos,not a.show_during_load,beautify=a.beautify)
