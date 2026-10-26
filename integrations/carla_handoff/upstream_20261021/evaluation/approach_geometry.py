"""Serializable upstream lane geometry, shared by online and replay metrics."""
import argparse
import hashlib
import json
import math
from pathlib import Path

DEFAULT_APPROACH_CONFIG = Path(__file__).parent / "configs" / "approach_metrics.json"


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
            reference_s=override["s"] if override else stop["s"]
            if not math.isfinite(reference_s) or reference_s<0:
                raise ValueError("Invalid stop reference s")
            if override and not str(override.get("calibration_id","")).strip():
                raise ValueError("Stop-reference override requires calibration_id")
            wp = road_map.get_waypoint_xodr(stop["road_id"], stop["lane_id"], reference_s)
            if wp is None or wp.section_id != stop["section_id"]:
                raise ValueError("Stop reference cannot be resolved")
            samples, station, reason = [], 0., "complete"
            while True:
                p = wp.transform.location
                samples.append(dict(x=p.x,y=p.y,z=p.z,s=station,width=wp.lane_width,
                    road_id=wp.road_id,section_id=wp.section_id,lane_id=wp.lane_id))
                if station >= length - 1e-5:
                    break
                previous = wp.previous(min(step,length-station))
                if len(previous) != 1 or previous[0].is_junction:
                    reason = "upstream_end_or_branch"
                    break
                q = previous[0].transform.location
                increment = math.hypot(q.x-p.x,q.y-p.y)
                if increment < 1e-5:
                    reason = "degenerate_geometry"
                    break
                station += increment
                wp = previous[0]
            lanes[key] = dict(id=key,signal_ids=[str(light["opendrive_id"])],samples=samples,
                geometry_status=reason,reference_kind="user_supplied_stop_waypoint" if override else "carla_stop_waypoint",
                calibration_id=override["calibration_id"] if override else None)
    if set(overrides)-set(lanes):
        raise ValueError("Stop-reference overrides contain unknown approaches")
    return dict(schema_version="1.0", map_name=road_map.name,opendrive_sha256=digest,
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
