"""Minimal multi-junction finish: stop bars only.

OpenDRIVE supplies the road surface, lane markings and controllable heads.
This module adds only visible stop bars; signal heads and poles are handled by
lane_signals. It deliberately does not add sidewalks, grass, trees or props.
"""
import math
import carla

ASSET = '/Game/Carla/Static/RoadLine/SM_BassicTrafficLineCube_trafficlineCube.SM_BassicTrafficLineCube_trafficlineCube'


def build(world, design):
    library = world.get_blueprint_library()
    ids = []
    width = float(design['config']['lane_width_m'])
    lanes = int(design['config']['incoming_lanes'])

    def bar(location, yaw, length):
        bp = library.find('static.prop.mesh')
        bp.set_attribute('mesh_path', ASSET)
        bp.set_attribute('scale', str(length / .124086))
        bp.set_attribute('role_name', 'network_simple_stop_line')
        actor = world.spawn_actor(bp, carla.Transform(location, carla.Rotation(yaw=yaw)))
        actor.set_simulate_physics(False)
        actor.set_collisions(False)
        ids.append(actor.id)

    reports = []
    road_map = world.get_map()
    for junction in design['junctions']:
        for rid, length in zip(junction['roads'], junction['lengths']):
            stations = {float(signal['reference_s']) for signal in design['signals'].values()
                        if signal['road_id'] == rid}
            if len(stations) != 1:
                raise ValueError('Expected a common stop station for road {}'.format(rid))
            s = stations.pop()
            wp = road_map.get_waypoint_xodr(rid, -1, s)
            if wp is None:
                raise RuntimeError('Missing stop-line waypoint for road {}'.format(rid))
            right = wp.transform.get_right_vector()
            base = wp.transform.location
            # The incoming carriageway occupies t=[0, lanes*width] relative
            # to lane -1, matching the complete network finish geometry.
            start = carla.Location(base.x + right.x * (-width / 2),
                                   base.y + right.y * (-width / 2),
                                   base.z + .006)
            yaw = math.degrees(math.atan2(right.y, right.x))
            angle = math.radians(yaw)
            count = int(math.ceil(lanes * width / .5))
            tile = lanes * width / count
            scale = tile / .124086
            for index in range(count):
                along = (index + .5) * tile
                center = carla.Location(
                    start.x + right.x * along - .061836 * scale * math.cos(angle),
                    start.y + right.y * along - .061836 * scale * math.sin(angle),
                    start.z - .003803 * scale)
                bar(center, yaw, tile)
            reports.append(dict(road_id=rid, s=s, lane_count=lanes, width_m=lanes * width))
    return dict(actor_ids=ids, stop_lines=reports, mode='network_simple',
                sidewalks=False, grass=False, trees=False, roadside_props=False,
                collision_policy='visual stop bars only; native road collision retained')


if __name__ == '__main__':
    import argparse
    from network_preview import main
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map_dir')
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--no-photos', action='store_true')
    args = parser.parse_args()
    main(args.map_dir, args.host, args.port, photos=not args.no_photos, mode='simple')
