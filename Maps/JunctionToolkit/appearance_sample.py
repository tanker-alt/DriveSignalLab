"""Small runtime-only paint test. No debug lines or external asset import."""
import argparse
import json
import math
from pathlib import Path
import carla
from runtime_config import map_output

ROLE = 'appearance_probe'


def build(world):
    for actor in world.get_actors():
        if actor.attributes.get('role_name') == ROLE:
            actor.destroy()
    owned = []
    def spawn(asset, x, y, z, scale=1):
        bp = world.get_blueprint_library().find('static.prop.mesh')
        bp.set_attribute('mesh_path', '/Game/Carla/Static/'+asset+'.'+asset.split('/')[-1])
        bp.set_attribute('scale', str(scale))
        bp.set_attribute('role_name', ROLE)
        a = world.spawn_actor(bp, carla.Transform(carla.Location(x,y,z)))
        a.set_simulate_physics(False)
        a.set_collisions(False)
        owned.append(a.id)
        return a
    # Measured cube dimensions at scale=1, metres. Sink the volume below
    # road z=0, leaving only its upper face 1 mm above the pavement.
    def rectangle(x, y, length, width):
        size=.124086
        scale=width/size
        count=int(math.ceil(length/width))
        for i in range(count):
            center=x+width/2+i*(length-width)/max(1,count-1)
            spawn('RoadLine/SM_BassicTrafficLineCube_trafficlineCube',
                  center-.061836*scale,y,.001-.003803*scale,scale)
    # One stop line, then a zebra crossing toward the junction.
    for k in range(21):
        rectangle(-36, -10.25+k, 4, .5)
    # A transverse stop line tiled in the same plane.
    for k in range(21):
        rectangle(-39, -10.25+k*.5, .5, .5)
    for x in (-95,-86,-77,-68,-59,-50):
        spawn('RoadLine/St_Road_TileRoad_LaneMarkingBroken',x,0,.001-.029153)
    return dict(actor_ids=owned,paint_top_m=.001,debug_lines=False,
                status='experimental visual sample; not enabled for production maps')


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',required=True)
    a=p.parse_args();out=map_output(a.out);out.mkdir(parents=True,exist_ok=True)
    client=carla.Client('localhost',2000);client.set_timeout(60)
    world=client.get_world()
    if world.get_settings().synchronous_mode:
        raise RuntimeError('Finish synchronous experiment first')
    report=build(world)
    world.get_spectator().set_transform(carla.Transform(carla.Location(-60,-22,24),carla.Rotation(pitch=-40,yaw=40)))
    from view import capture
    capture(world,out,[-42,0],38)
    (out/'sample_report.json').write_text(json.dumps(report,indent=2))
    print('Sample actors:',len(report['actor_ids']))
