"""CARLA runtime preview of standard markings; native XODR supplies pavement/curbs.

OpenDRIVE objects also retain stop/crosswalk footprints for RoadRunner transfer.
Debug paint is a preview, not a baked material or semantic segmentation asset.
"""
import math
import carla
from road_standard import stop_s


def build(world, design):
    cfg = design['config']
    road_map = world.get_map()
    ids, failures = [], []
    span = (max(cfg['rows'], cfg['columns'])-1)*cfg['spacing_m']+2*(cfg['arm_length_m']+cfg['junction_radius_m'])+100
    bp = world.get_blueprint_library().find('static.prop.mesh')
    bp.set_attribute('mesh_path', '/Game/Carla/Static/Ground/GroundCube.GroundCube')
    bp.set_attribute('scale', str(span))
    bp.set_attribute('role_name', 'standard_road_ground')
    actor = world.try_spawn_actor(bp, carla.Transform(carla.Location(
        x=(cfg['columns']-1)*cfg['spacing_m']/2,
        y=-(cfg['rows']-1)*cfg['spacing_m']/2, z=-span/2-.2)))
    if actor:
        actor.set_simulate_physics(False)
        actor.set_collisions(False)
        ids.append(actor.id)
    else:
        failures.append('GroundCube')
    count = 0
    for junction in design['junctions']:
        for rid, length in zip(junction['roads'], junction['lengths']):
            def point(s, lateral):
                wp = road_map.get_waypoint_xodr(rid, -1, s)
                p, right = wp.transform.location, wp.transform.get_right_vector()
                return carla.Location(p.x+right.x*(lateral-1.75),
                                      p.y+right.y*(lateral-1.75), p.z+.035)

            def paint(s0, t0, s1, t1, thickness=.15, yellow=False):
                nonlocal count
                world.debug.draw_line(point(s0, t0), point(s1, t1), thickness=thickness,
                                      color=carla.Color(3, 2, 0) if yellow else carla.Color(2, 2, 2),
                                      life_time=0, persistent_lines=True)
                count += 1

            for t in (-.15, .15):
                paint(.01, t, length-6, t, yellow=True)
            for t in (-7., -3.5, 3.5, 7.):
                end = stop_s(length)-.15 if t > 0 else length-6
                solid = stop_s(length)-25 if t > 0 else end
                s = .01
                while s < solid:
                    paint(s, t, min(s+3, solid), t)
                    s += 9
                if solid < end:
                    paint(solid, t, end, t)
            for t in (-10.5, 10.5):
                paint(.01, t, stop_s(length)-.15 if t > 0 else length-6, t)
            for k in range(21):
                t = -10.25+k
                paint(length-6, t, length-2, t, .5)
    return dict(actor_ids=ids, failed_assets=failures, preview_paint_segments=count,
                road_material='Native CARLA OpenDRIVE mesh; no square junction tile overlay',
                curb_geometry='Native raised straight/arc sidewalk lanes, 0.20m curb + 2m sidewalk',
                collision_policy='Native OpenDRIVE road/sidewalk physics; ground decoration has no collision',
                markings='XODR roadMark + crosswalk/stop objects; debug preview requires reload through view')
