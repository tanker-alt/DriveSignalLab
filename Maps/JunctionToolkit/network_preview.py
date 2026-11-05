"""Appearance-only preview; does not certify network connectivity (CARLA Python)."""
import argparse
import hashlib
import json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import xml.etree.ElementTree as ET
import carla
from runtime_finish import build as build_complete
from network_simple_finish import build as build_simple
from lane_signals import build as signals
from view import capture

def disable_collision_batch(args):
    # Separate clients/processes avoid serialization by the CARLA Python GIL.
    actor_ids,host,port=args
    client=carla.Client(host,port)
    client.set_timeout(120)
    actors=client.get_world().get_actors(actor_ids)
    if len(actors)!=len(actor_ids):
        raise RuntimeError('Decoration actor disappeared during finalization')
    for actor in actors:
        actor.set_collisions(False)
    return len(actors)

def main(folder,host='localhost',port=2000,photos=True,mode='simple'):
    folder=Path(folder)
    raw=(folder/'map.xodr').read_bytes().decode('utf-8')
    design=json.loads((folder/'manifest.json').read_text(encoding='utf-8'))
    assert hashlib.sha256(raw.encode()).hexdigest()==design['opendrive_sha256']
    validation_path=folder/'topology_validation.json'
    validation=json.loads(validation_path.read_text()) if validation_path.exists() else {}
    if not validation.get('passed') or validation.get('opendrive_sha256',design['opendrive_sha256'])!=design['opendrive_sha256']:
        raise RuntimeError('Map requires a current passed topology report')
    from network_appearance_plan import plan
    appearance_plan=plan(design,raw,mode)
    cfg=design['config']
    client=carla.Client(host,port);client.set_timeout(120)
    world=client.get_world()
    if world.get_settings().synchronous_mode or any(world.get_actors().filter(p) for p in ('vehicle.*','walker.*','sensor.*')):
        raise RuntimeError('Finish active simulation before preview')
    world=client.generate_opendrive_world(raw,carla.OpendriveGenerationParameters(vertex_distance=1.,max_road_length=50.,wall_height=0.,additional_width=0.,smooth_junctions=False,enable_mesh_visibility=True,enable_pedestrian_navigation=False))
    # Avoid rendering tens of thousands of partially initialized paint actors.
    hidden=world.get_settings()
    hidden.no_rendering_mode=True
    world.apply_settings(hidden)
    assert world.get_map().to_opendrive()==raw
    world.set_weather(carla.WeatherParameters(cloudiness=25,precipitation=0,wetness=0,sun_altitude_angle=48,sun_azimuth_angle=110))
    lights=list(world.get_actors().filter('traffic.traffic_light'))
    ids=[l.get_opendrive_id() for l in lights]
    if len(set(ids))!=len(ids) or set(ids)!=set(design['signals']):
        raise RuntimeError('Native signal IDs do not match manifest')
    for light in lights:
        spec=design['signals'][light.get_opendrive_id()]
        actual={(w.road_id,w.lane_id) for w in light.get_affected_lane_waypoints()}
        expected={(spec['road_id'],lane) for lane in spec['incoming_lanes']}
        if actual!=expected:
            raise RuntimeError('Signal lane validity mismatch: '+light.get_opendrive_id())
    signal_report=None;finish=dict(actor_ids=[],enabled=False,collision_finalized=True)
    if mode not in ('simple','complete','off'):
        raise ValueError('Network mode must be simple, complete or off')
    if mode != 'off':
        print('Applying signal masts',flush=True)
        signal_report=signals(world,design)
        if mode == 'complete':
            print('Loaded; applying complete network finish',flush=True)
            finish=build_complete(world,design,client)
        else:
            print('Loaded; applying simple stop-line finish',flush=True)
            finish=build_simple(world,design)
        if appearance_plan['ring_perimeter']:
            from ring_sidewalk_finish import build as finish_perimeter
            finish['ring_sidewalks']=finish_perimeter(client,world,design)
            finish['actor_ids'].extend(finish['ring_sidewalks']['actor_ids'])
    actor_ids=finish['actor_ids']
    batches=[(actor_ids[i:i+500],host,port) for i in range(0,len(actor_ids),500)]
    done=0
    with ProcessPoolExecutor(max_workers=16) as pool:
        for count in pool.map(disable_collision_batch,batches):
            done+=count
            if done%5000==0 or done==len(actor_ids):
                print('Collision disabled:',done,'/',len(actor_ids),flush=True)
    finish['collision_finalized']=True
    visible=world.get_settings()
    visible.no_rendering_mode=False
    world.apply_settings(visible)
    world.wait_for_tick(30)
    center=[(cfg['columns']-1)*cfg['spacing_m']/2,(cfg['rows']-1)*cfg['spacing_m']/2]
    wide=cfg['columns']>=3*cfg['rows']
    extent=cfg['arm_length_m']+cfg.get('junction_radius_m',22.5)+40
    span_x=(cfg['columns']-1)*cfg['spacing_m']+2*extent
    span_y=(cfg['rows']-1)*cfg['spacing_m']+2*extent
    height=1.1*max(span_x/2,span_y*(3 if wide else 4/3)/2)
    world.get_spectator().set_transform(carla.Transform(carla.Location(center[0],-center[1],height),carla.Rotation(pitch=-90,yaw=-90)))
    if photos: print('Capturing RGB',flush=True)
    street_center=design['junctions'][len(design['junctions'])//2]['center']
    if photos: capture(world,folder,street_center,height,center,wide=wide)
    validation_path=folder/'topology_validation.json'
    validation=json.loads(validation_path.read_text()) if validation_path.exists() else {}
    checked=validation.get('passed',False) and validation.get('opendrive_sha256')==design['opendrive_sha256']
    report=dict(beautify=mode != 'off',appearance_mode=mode,scope='appearance_preview',topology_validated=checked,map_hash=design['opendrive_sha256'],scenery=finish,lane_signals=signal_report,trees=False,grass_mesh=False,roadside_props=False,debug_overlays=False,appearance_plan=appearance_plan)
    (folder/'appearance_preview_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    (folder/'load_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print('Saved overview.png and street.png' if photos else 'Loaded network: '+mode,flush=True)
    return world,design

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('map_dir');p.add_argument('--mode',choices=('simple','complete','off'),default='simple');p.add_argument('--no-photos',action='store_true');a=p.parse_args();main(a.map_dir,photos=not a.no_photos,mode=a.mode)
