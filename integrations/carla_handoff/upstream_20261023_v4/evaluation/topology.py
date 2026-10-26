"""从实时地图标定交通计数线，并输出俯视图与 CARLA 调试绘制。

到达线：默认对齐进口检测区实际上游边界；允许显式指定上游距离。
出口线：沿 junction 出口 waypoint 向下游 5 m。
这两类线是人为定义的流量断面，不需要与路面停止标线重合；
本工具不将未经人工核验的停止参考点升级为违规判据。
"""
import argparse
import json
import math
from pathlib import Path


def approach_arrivals(geometry):
    """Counting sections at the actual upstream ends, pointing downstream."""
    lines=[]
    for lane in geometry["lanes"]:
        samples=lane["samples"]
        if len(samples)<2 or samples[-1]["s"]<=0:
            raise ValueError("Approach has no usable upstream boundary: "+lane["id"])
        end,inner=samples[-1],samples[-2]
        dx,dy=inner["x"]-end["x"],inner["y"]-end["y"]
        length=math.hypot(dx,dy)
        if length<=1e-6:
            raise ValueError("Degenerate approach boundary")
        for old in lines:
            if (old["road_id"],old["section_id"],old["lane_id"]) == (end["road_id"],end["section_id"],end["lane_id"]):
                raise ValueError("Shared upstream lane requires explicit detection configuration")
        lines.append(dict(id="arrival_"+lane["id"],kind="arrival",junction_id=geometry["junction_id"],
            center_xy=[end["x"],end["y"]],z=end["z"],normal_xy=[dx/length,dy/length],
            half_width_m=end["width"]/2,road_id=end["road_id"],section_id=end["section_id"],
            lane_id=end["lane_id"],enabled=True,source="approach_upstream_boundary",
            approach_id=lane["id"],actual_detection_length_m=end["s"],
            geometry_status=lane["geometry_status"]))
    return lines


def build(world, junction_id=1427, arrival_distance_m=None, exit_distance_m=5.0, approach_geometry=None):
    import carla
    road_map = world.get_map()
    if (arrival_distance_m is not None and arrival_distance_m <= 0) or exit_distance_m <= 0:
        raise ValueError("Counting section distances must be positive")
    junction = next((w.get_junction() for w in road_map.generate_waypoints(2)
                     if w.is_junction and w.get_junction().id == junction_id), None)
    if junction is None:
        raise ValueError("Target junction not found")
    lines = {}

    def add(wp, kind, source):
        if wp.is_junction:
            raise ValueError("Counting line must lie outside junction interior")
        key = (kind, wp.road_id, wp.section_id, wp.lane_id)
        location = wp.transform.location
        forward = wp.transform.get_forward_vector()
        candidate = dict(id="%s_%s_%s_%s" % key, kind=kind, junction_id=junction_id,
            center_xy=[location.x, location.y], z=location.z,
            normal_xy=[forward.x, forward.y], half_width_m=wp.lane_width / 2,
            road_id=wp.road_id, section_id=wp.section_id, lane_id=wp.lane_id,
            s=wp.s, enabled=True, source=source)
        # 多个 junction movement 可能共享出口车道，只保留同一个物理断面。
        if key in lines:
            old = lines[key]["center_xy"]
            if ((old[0]-location.x)**2+(old[1]-location.y)**2)**.5 > 1:
                raise ValueError("Conflicting exit locations for same lane; manual calibration required")
        else:
            lines[key] = candidate

    movements = junction.get_waypoints(carla.LaneType.Driving)
    stops = [stop for light in world.get_traffic_lights_in_junction(junction_id)
             for stop in light.get_stop_waypoints()]
    # Unsignalized junctions have no signal stop references; use movement entries.
    aligned=arrival_distance_m is None
    if aligned and approach_geometry is None:
        try:
            from .approach_geometry import build as build_approaches, DEFAULT_APPROACH_CONFIG
            from .map_context import read_inventory
        except ImportError:
            from approach_geometry import build as build_approaches, DEFAULT_APPROACH_CONFIG
            from map_context import read_inventory
        approach_geometry=build_approaches(world,read_inventory(world,junction_id),
            json.loads(DEFAULT_APPROACH_CONFIG.read_text(encoding="utf-8")))
    if aligned and (not approach_geometry or not approach_geometry["lanes"]):
        raise ValueError("Automatic arrival alignment needs approach geometry; supply detection_config or explicit arrival_distance_m")
    for stop in ([] if aligned else (stops or [entry for entry, _ in movements])):
        upstream = stop.previous(arrival_distance_m)
        if len(upstream) != 1:
            raise ValueError("Ambiguous arrival lane; manual calibration required")
        add(upstream[0], "arrival", "map_entry.previous({}m)".format(arrival_distance_m))
    for entry, exit_wp in movements:
        downstream = exit_wp.next(exit_distance_m)
        if len(downstream) != 1:
            raise ValueError("Ambiguous exit lane; manual calibration required")
        add(downstream[0], "exit", "junction_exit.next({}m)".format(exit_distance_m))
    return dict(schema_version="2.0", map_name=road_map.name, junction_id=junction_id,
                calibration="CARLA lane center, width and tangent; traffic-counting sections only",
                arrival_policy="approach_upstream_boundary" if aligned else "explicit_distance",
                lines=(approach_arrivals(approach_geometry) if aligned else [])+list(lines.values()))


def visualize(world, config, output):
    import carla
    import os
    # 缓存放在工程中，不要求向用户主目录写字体缓存。
    os.environ.setdefault("MPLCONFIGDIR",str(Path(__file__).parent / ".mplcache"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import math
    fig, ax = plt.subplots(figsize=(10, 10))
    centers = [line["center_xy"] for line in config["lines"]]
    cx, cy = [sum(p[i] for p in centers)/len(centers) for i in (0, 1)]
    for wp in world.get_map().generate_waypoints(1):
        p = wp.transform.location
        if abs(p.x-cx)<65 and abs(p.y-cy)<65:
            ax.plot(p.x, p.y, ".", color="0.8", markersize=2)
    for line in config["lines"]:
        x,y = line["center_xy"]; nx,ny = line["normal_xy"]
        n=math.hypot(nx,ny); nx,ny=nx/n,ny/n
        width=line["half_width_m"]
        a,b=(x-ny*width,y+nx*width),(x+ny*width,y-nx*width)
        color="tab:blue" if line["kind"]=="arrival" else "tab:orange"
        ax.plot([a[0],b[0]],[a[1],b[1]],color=color,linewidth=3)
        ax.arrow(x,y,nx*4,ny*4,color=color,head_width=.7,length_includes_head=True)
        # 并排车道标签错开，避免上下方向进口的两个名称叠在一起。
        offset = 13 if abs(line["lane_id"]) % 2 else -15
        ax.annotate(line["id"], (x,y), xytext=(8,offset), textcoords="offset points",fontsize=6)
        z=line["z"]+.3
        world.debug.draw_line(carla.Location(a[0],a[1],z),carla.Location(b[0],b[1],z),
            thickness=.15,color=carla.Color(40,120,255) if line["kind"]=="arrival" else carla.Color(255,150,20),life_time=300)
    ax.set_aspect("equal"); ax.invert_yaxis(); ax.set_xlabel("CARLA X (m)"); ax.set_ylabel("CARLA Y (m)")
    ax.set_title("{} junction {} | blue: arrival, orange: exit".format(config["map_name"], config["junction_id"]))
    fig.tight_layout(); fig.savefig(output,dpi=180); plt.close(fig)


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host",default="localhost"); parser.add_argument("--port",type=int,default=2000)
    parser.add_argument("--junction-id", type=int, required=True)
    parser.add_argument("--arrival-distance-m",type=float,help="Override upstream alignment with an explicit distance")
    parser.add_argument("--output", type=Path, required=True, help="New calibration JSON (must not exist)")
    args=parser.parse_args()
    import carla
    client=carla.Client(args.host,args.port); client.set_timeout(10)
    world=client.get_world(); config=build(world, args.junction_id,args.arrival_distance_m)
    path=args.output
    with path.open("x", encoding="utf-8") as handle:
        json.dump(config, handle, indent=2)
    visualize(world,config,path.with_suffix(".png"))
    print("Calibrated lines:",len(config["lines"])); print(path)
