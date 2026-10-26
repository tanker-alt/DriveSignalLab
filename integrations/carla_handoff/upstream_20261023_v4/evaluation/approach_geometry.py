"""Serializable upstream lane geometry, shared by online and replay metrics."""
import argparse
import hashlib
import json
import math
from pathlib import Path

DEFAULT_APPROACH_CONFIG = Path(__file__).parent / "configs" / "approach_metrics.json"


def resolve_stop_line(road_map, stop, line, junction_id):
    """Resolve an explicit surveyed line without guessing a junction movement.

    The selected waypoint must trace uniquely back to the original approach.
    A short junction prefix is permitted; another junction or a fork is not.
    """
    ref=line['reference_waypoint']
    center=line['center']
    normal=line['normal_xy']
    values=[center[k] for k in ('x','y','z')]+list(normal)+[ref['s'],line['estimated_error_m']]
    if len(normal)!=2 or any(not math.isfinite(v) for v in values) or ref['s']<0 or line['estimated_error_m']<0:
        raise ValueError('Invalid surveyed stop-line numbers')
    if abs(math.hypot(*normal)-1)>1e-4:
        raise ValueError('Stop-line normal must be a unit downstream vector')
    if line.get('convention')!='approach_side_edge' or not line.get('evidence'):
        raise ValueError('Survey requires approach-side edge convention and evidence')
    wp=road_map.get_waypoint_xodr(ref['road_id'],ref['lane_id'],ref['s'])
    if wp is None or (wp.road_id,wp.section_id,wp.lane_id)!=(ref['road_id'],ref['section_id'],ref['lane_id']):
        raise ValueError('Survey waypoint cannot be resolved')
    p=wp.transform.location
    if math.hypot(p.x-center['x'],p.y-center['y'])>.25 or abs(p.z-center['z'])>.25:
        raise ValueError('Survey center is too far from the selected lane center')
    angle=math.radians(wp.transform.rotation.yaw)
    if math.cos(angle)*normal[0]+math.sin(angle)*normal[1]<math.cos(math.radians(15)):
        raise ValueError('Survey normal disagrees with downstream travel')
    anchor=stop['location']
    current=wp
    for _ in range(121):
        if current.is_junction and current.get_junction().id!=int(junction_id):
            raise ValueError('Survey crosses another junction')
        p=current.transform.location
        if ((current.road_id,current.section_id,current.lane_id)==(stop['road_id'],stop['section_id'],stop['lane_id'])
                and math.hypot(p.x-anchor['x'],p.y-anchor['y'])<=.35):
            return wp
        previous=current.previous(.25)
        if len(previous)!=1:
            raise ValueError('Survey upstream connection is missing or ambiguous')
        current=previous[0]
    raise ValueError('Survey does not connect to the original stop reference within 30 m')


def build(world, inventory, config, stop_reference_config=None):
    road_map = world.get_map()
    digest = hashlib.sha256(road_map.to_opendrive().encode("utf-8")).hexdigest()
    if road_map.name.split("/")[-1] != inventory["map_name"].split("/")[-1]:
        raise ValueError("Geometry map mismatch")
    if inventory.get("opendrive_sha256") not in (None, digest):
        raise ValueError("OpenDRIVE changed since collection")
    overrides={}
    if stop_reference_config is not None:
        if (stop_reference_config["map_name"].split("/")[-1]!=road_map.name.split("/")[-1] or
                int(stop_reference_config["junction_id"])!=int(inventory["junction_id"]) or
                stop_reference_config["opendrive_sha256"]!=digest):
            raise ValueError("Stop-reference calibration map/junction/fingerprint mismatch")
        overrides=stop_reference_config["approaches"]
    length, step = config["detection_length_m"], config["sample_step_m"]
    if not math.isfinite(length) or not math.isfinite(step) or not 0 < step <= length:
        raise ValueError("Invalid approach length/step")
    lanes = {}
    for light in inventory["traffic_lights"]:
        for stop in light["stop_waypoints"]:
            key = "{}_{}_{}".format(stop["road_id"], stop["section_id"], stop["lane_id"])
            if key in lanes:
                lanes[key]["signal_ids"].append(str(light["opendrive_id"]))
                continue
            override=overrides.get(key)
            if key in overrides and (not isinstance(override,dict) or (('s' in override)==('stop_line' in override))):
                raise ValueError("Stop-reference override requires exactly one of s or stop_line")
            line=override.get('stop_line') if override else None
            if override is not None and 'stop_line' in override and (not isinstance(line,dict) or not line):
                raise ValueError('stop_line must be a nonempty survey record')
            reference_s=override["s"] if override is not None and line is None else stop["s"]
            if not math.isfinite(reference_s) or reference_s<0:
                raise ValueError("Invalid stop reference s")
            if override is not None and not str(override.get("calibration_id","")).strip():
                raise ValueError("Stop-reference override requires calibration_id")
            wp = (resolve_stop_line(road_map,stop,line,inventory['junction_id']) if line else
                  road_map.get_waypoint_xodr(stop["road_id"], stop["lane_id"], reference_s))
            if wp is None or (not line and (wp.road_id,wp.section_id,wp.lane_id) != (stop["road_id"],stop["section_id"],stop["lane_id"])):
                raise ValueError("Stop reference cannot be resolved")
            samples, station, reason = [], 0., "complete"
            junction_prefix=bool(line and wp.is_junction)
            while True:
                p = wp.transform.location
                position=line['center'] if line and not samples else dict(x=p.x,y=p.y,z=p.z)
                samples.append(dict(**position,s=station,width=wp.lane_width,
                    road_id=wp.road_id,section_id=wp.section_id,lane_id=wp.lane_id))
                if station >= length - 1e-5:
                    break
                previous = wp.previous(min(step,length-station))
                if len(previous) != 1 or (previous[0].is_junction and
                        not (junction_prefix and previous[0].get_junction().id==int(inventory['junction_id']))):
                    reason = "upstream_end_or_branch"
                    break
                if not previous[0].is_junction:
                    junction_prefix=False
                q = previous[0].transform.location
                increment = math.hypot(q.x-position['x'],q.y-position['y'])
                if increment < 1e-5:
                    reason = "degenerate_geometry"
                    break
                station += increment
                wp = previous[0]
            lanes[key] = dict(id=key,signal_ids=[str(light["opendrive_id"])],samples=samples,
                geometry_status=reason,reference_kind="surveyed_stop_line" if line else "user_supplied_stop_waypoint" if override else "carla_stop_waypoint",
                stop_line=line,
                calibration_id=override["calibration_id"] if override else None)
    if set(overrides)-set(lanes):
        raise ValueError("Stop-reference overrides contain unknown approaches")
    return dict(schema_version="1.1", map_name=road_map.name,opendrive_sha256=digest,
                junction_id=inventory["junction_id"],config=config,lanes=list(lanes.values()),
                distance_reference="per_lane_reference_kind",stop_reference_config=stop_reference_config)


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--host",default="localhost")
    parser.add_argument("--port",type=int,default=2000)
    parser.add_argument("--stop-reference-config",type=Path)
    args=parser.parse_args()
    import carla
    client=carla.Client(args.host,args.port); client.set_timeout(10)
    inventory=json.loads((args.run/"metadata.json").read_text(encoding="utf-8"))["inventory"]
    config=json.loads(DEFAULT_APPROACH_CONFIG.read_text(encoding="utf-8"))
    overrides=json.loads(args.stop_reference_config.read_text(encoding="utf-8-sig")) if args.stop_reference_config else None
    geometry=build(client.get_world(),inventory,config,overrides)
    with args.output.open("x",encoding="utf-8") as handle:
        json.dump(geometry,handle,indent=2)
    print(args.output)
