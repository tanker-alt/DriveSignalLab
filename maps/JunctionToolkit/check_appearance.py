"""Check loaded-map identity, native sidewalks, lamps and captured RGB artifacts."""
import argparse
import hashlib
import json
from pathlib import Path
import carla
from appearance_standard import settings


def run(folder):
    folder=Path(folder)
    design=json.loads((folder/'manifest.json').read_text())
    report=json.loads((folder/'load_report.json').read_text())
    client=carla.Client('localhost',2000);client.set_timeout(15)
    world=client.get_world();road_map=world.get_map();cfg=design['config']
    identity=hashlib.sha256(road_map.to_opendrive().encode()).hexdigest()==design['opendrive_sha256']
    samples=[]
    for j in design['junctions']:
        for rid,length in zip(j['roads'],j['lengths']):
            wp=road_map.get_waypoint_xodr(rid,-1,length/2)
            right=wp.transform.get_right_vector()
            half=cfg['incoming_lanes']*cfg['lane_width_m']
            for sign in (-1,1):
                lateral=sign*(half+cfg.get('shoulder_width_m',0)+cfg['curb_width_m']+cfg['sidewalk_width_m']/2)-cfg['lane_width_m']/2
                p=wp.transform.location+right*lateral
                hits=world.cast_ray(carla.Location(p.x,p.y,2),carla.Location(p.x,p.y,-.5))
                z=max([h.location.z for h in hits],default=None)
                samples.append(dict(road_id=rid,side=sign,height=z,passed=z is not None and abs(z-.15)<.04))
    images={name:hashlib.sha256((folder/name).read_bytes()).hexdigest() for name in ('overview.png','street.png')}
    lights=list(world.get_actors().filter('traffic.traffic_light'))
    signal_ok={a.get_opendrive_id() for a in lights}==set(design['signals'])
    tree_report=report['trees']
    result=dict(passed=identity and signal_ok and all(s['passed'] for s in samples),
                opendrive_sha256=design['opendrive_sha256'],map_identity=identity,
                signal_count=len(lights),signal_ids_match=signal_ok,sidewalk_samples=samples,
                tree_count=tree_report['count'],tree_rejections=tree_report['rejected'],
                image_sha256=images,appearance_template=settings(cfg),
                paint_top_m=report['scenery']['paint_top_m'],
                limits='Native sidewalk height and lamp IDs checked; image appearance reviewed separately, no traffic-flow certification.')
    (folder/'appearance_validation.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('Appearance checks:',result['passed'],'sidewalk samples:',len(samples),'signals:',len(lights),'trees:',tree_report['count'])
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('map_dir');args=p.parse_args()
    if not run(args.map_dir)['passed']:raise SystemExit(1)
