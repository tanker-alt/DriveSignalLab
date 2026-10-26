"""Inspect the loaded map, list junctions, export road information, optionally draw labels."""
import argparse
import hashlib
import json
import math
from pathlib import Path
from evaluation.map_context import junctions, read_inventory, waypoint_record


def inspect(world, junction_id=None):
    import carla
    road_map=world.get_map()
    found=junctions(world)
    if junction_id is not None and junction_id not in found:
        raise ValueError("Junction {} is not on {}".format(junction_id,road_map.name))
    records=[]
    for jid,junction in sorted(found.items()):
        if junction_id is not None and jid!=junction_id:
            continue
        center=junction.bounding_box.location
        movements=junction.get_waypoints(carla.LaneType.Driving)
        lights=world.get_traffic_lights_in_junction(jid)
        item=dict(junction_id=jid,center=dict(x=center.x,y=center.y,z=center.z),
            road_ids=sorted({wp.road_id for pair in movements for wp in pair}),
            movement_count=len(movements),signal_ids=sorted(str(l.get_opendrive_id()) for l in lights))
        if junction_id is not None:
            item["movements"]=[dict(entry=waypoint_record(a),exit=waypoint_record(b)) for a,b in movements]
            item["inventory"]=read_inventory(world,jid)
        records.append(item)
    return dict(schema_version="map_tools.1",map_name=road_map.name,
        opendrive_sha256=hashlib.sha256(road_map.to_opendrive().encode("utf-8")).hexdigest(),
        junctions=records)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host",default="localhost")
    parser.add_argument("--port",type=int,default=2000)
    parser.add_argument("--junction-id",type=int,help="Omit to list all junctions")
    parser.add_argument("--output",type=Path,help="New JSON file, never overwrite existing output")
    parser.add_argument("--draw",action="store_true",help="Draw junction IDs and entry/exit reference labels")
    parser.add_argument("--focus",action="store_true",help="Move spectator above selected junction")
    parser.add_argument("--life-seconds",type=float,default=60)
    args=parser.parse_args()
    if not math.isfinite(args.life_seconds) or args.life_seconds<=0:
        parser.error("life-seconds must be finite and positive")
    if args.focus and args.junction_id is None:
        parser.error("--focus requires --junction-id")
    if args.output and args.output.exists():
        parser.error("output already exists")
    import carla
    client=carla.Client(args.host,args.port);client.set_timeout(10)
    world=client.get_world();data=inspect(world,args.junction_id)
    print(json.dumps(data,ensure_ascii=False,indent=2))
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        with args.output.open("x",encoding="utf-8") as handle:
            json.dump(data,handle,ensure_ascii=False,indent=2)
    for item in data["junctions"]:
        center=carla.Location(**item["center"])
        if args.draw:
            world.debug.draw_string(center,"J{}".format(item["junction_id"]),
                draw_shadow=True,color=carla.Color(255,220,0),life_time=args.life_seconds)
            for movement in item.get("movements",[]):
                for kind in ("entry","exit"):
                    wp=movement[kind]
                    world.debug.draw_string(carla.Location(**wp["location"]),
                        "{} {}_{}_{}".format(kind,wp["road_id"],wp["section_id"],wp["lane_id"]),
                        draw_shadow=True,color=carla.Color(40,220,255),life_time=args.life_seconds)
        if args.focus:
            center.z+=70
            world.get_spectator().set_transform(carla.Transform(center,carla.Rotation(pitch=-90)))


if __name__=="__main__":
    main()
