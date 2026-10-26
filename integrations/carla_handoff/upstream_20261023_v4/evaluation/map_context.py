"""Read the loaded map; no map loading, ticking, or signal control."""

import hashlib


def junctions(world):
    """Return junction objects indexed by ID (including unsignalized junctions)."""
    result = {}
    for waypoint in world.get_map().generate_waypoints(2.0):
        if waypoint.is_junction:
            junction = waypoint.get_junction()
            result[junction.id] = junction
    return result


def waypoint_record(waypoint):
    transform = waypoint.transform
    return dict(road_id=waypoint.road_id, section_id=waypoint.section_id,
                lane_id=waypoint.lane_id, s=waypoint.s, lane_width_m=waypoint.lane_width,
                location=dict(x=transform.location.x, y=transform.location.y,
                              z=transform.location.z), yaw=transform.rotation.yaw)


def read_inventory(world, junction_id):
    road_map = world.get_map()
    junction = junctions(world).get(junction_id)
    if junction is None:
        raise ValueError("Junction {} not found on {}".format(junction_id, road_map.name))
    center = junction.bounding_box.location
    lights = []
    for light in world.get_traffic_lights_in_junction(junction_id):
        lights.append(dict(opendrive_id=str(light.get_opendrive_id()),
                           stop_waypoints=[waypoint_record(wp) for wp in light.get_stop_waypoints()]))
    return dict(schema_version="1.1", map_name=road_map.name, junction_id=junction_id,
                opendrive_sha256=hashlib.sha256(road_map.to_opendrive().encode("utf-8")).hexdigest(),
                source="loaded_world", center_xy=[center.x, center.y], traffic_lights=lights)
