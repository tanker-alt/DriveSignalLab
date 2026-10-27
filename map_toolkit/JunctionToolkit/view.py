"""Load a validated map, decorate it and capture real RGB images (CARLA Python 3.7)."""
import argparse
import hashlib
import json
import queue
from pathlib import Path
import carla
from scenery import build
from lane_signals import build as build_lane_signals
from safety_rules import load_rules, draw_stop_lines


def capture(world, folder, center, radius, overview_center=None):
    bp=world.get_blueprint_library().find('sensor.camera.rgb')
    bp.set_attribute('image_size_x','1280');bp.set_attribute('image_size_y','960');bp.set_attribute('fov','90')
    overview_center=overview_center or center
    for name, transform in [
        ('overview',carla.Transform(carla.Location(x=overview_center[0],y=-overview_center[1],z=radius),carla.Rotation(pitch=-90,yaw=-90))),
        ('street',carla.Transform(carla.Location(x=center[0]-48,y=-center[1]-38,z=27),carla.Rotation(pitch=-24,yaw=38)))]:
        q=queue.Queue();camera=world.spawn_actor(bp,transform)
        try:
            camera.listen(q.put)
            for _ in range(6): frame=q.get(timeout=20)
            frame.save_to_disk(str(folder/(name+'.png')))
        finally:camera.stop();camera.destroy()


def decorate(world, design):
    # Keep sidewalks clear: no random furniture or roadside garden lamps.
    return [], []


def load(folder, host='localhost',port=2000,photos=True):
    folder=Path(folder);design=json.loads((folder/'manifest.json').read_text())
    if design['config'].get('type')=='roundabout':
        from roundabout_view import load as load_ring
        return load_ring(folder,host,port,photos)
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
    w=c.generate_opendrive_world(raw.decode('utf-8'),carla.OpendriveGenerationParameters(
        vertex_distance=.5 if design['config'].get('standard_profile') else 2.,max_road_length=50.,wall_height=0.,additional_width=0.,
        smooth_junctions=False,enable_mesh_visibility=True,enable_pedestrian_navigation=False))
    for _ in range(10):w.wait_for_tick(15)
    if w.get_map().to_opendrive()!=raw.decode('utf-8'):raise RuntimeError('Server loaded a stale OpenDriveMap cache')
    lights=list(w.get_actors().filter('traffic.traffic_light'))
    by_id={a.get_opendrive_id():a for a in lights}
    if set(by_id)!=set(design['signals']):raise RuntimeError('Actual signal IDs differ from map manifest')
    for sid,cfg in design['signals'].items():
        actual={(p.road_id,p.lane_id) for p in by_id[sid].get_affected_lane_waypoints()}
        if actual!={(cfg['road_id'],lane) for lane in cfg['incoming_lanes']}:raise RuntimeError('Signal lane validity mismatch: '+sid)
    w.set_weather(carla.WeatherParameters(cloudiness=25,precipitation=0,wetness=0,sun_altitude_angle=48,sun_azimuth_angle=110))
    scenery=build(w,design,safety_rules)
    lane_signals=build_lane_signals(w,design)
    ids,failures=decorate(w,design)
    stop_lines=draw_stop_lines(w,safety_rules) if safety_rules is not None else None
    center=design['junctions'][0]['center']
    w.get_spectator().set_transform(carla.Transform(carla.Location(x=center[0]-48,y=-center[1]-38,z=27),carla.Rotation(pitch=-24,yaw=38)))
    report=dict(map_hash=design['opendrive_sha256'],signal_count=len(lights),signal_validity_passed=True,
                decoration_ids=ids,decoration_failures=failures,debug_overlays=True,scenery=scenery,lane_signals=lane_signals,stop_lines=stop_lines)
    (folder/'load_report.json').write_text(json.dumps(report,indent=2))
    if photos:
        cfg=design['config']
        overview_center=[(cfg['columns']-1)*cfg['spacing_m']/2,(cfg['rows']-1)*cfg['spacing_m']/2]
        height=100 if len(design['junctions'])==1 else max(cfg['rows'],cfg['columns'])*cfg['spacing_m']*.85
        capture(w,folder,center,height,overview_center)
    print('Loaded:',folder,'lights:',len(lights),'roadside props:',len(ids),'failed props:',len(failures))
    return w,design


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('map_dir');p.add_argument('--host',default='localhost');p.add_argument('--port',type=int,default=2000);p.add_argument('--no-photos',action='store_true')
    a=p.parse_args();load(a.map_dir,a.host,a.port,not a.no_photos)
