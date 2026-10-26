#!/usr/bin/env python3
"""独立监听入口：适合调试；严格逐帧实验请使用控制器的 --collect。"""

import argparse
import json
import math
from pathlib import Path

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from evaluation.collector import DEFAULT_OUTPUT, FrameCollector
from evaluation.map_context import junctions



def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--junction-id", type=int)
    parser.add_argument("--list-junctions", action="store_true")
    parser.add_argument("--inventory", default=None, help="Optional legacy inventory")
    parser.add_argument("--detection-config", help="Optional calibrated sections JSON")
    parser.add_argument("--stop-reference-config",help="User-verified stop waypoint references JSON")
    parser.add_argument('--safety-config', help='Safety thresholds JSON override')
    parser.add_argument('--safety-map-dir', help='Explicit map directory containing safety_rules.json')
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--radius", type=float, default=120.0)
    parser.add_argument("--frames", type=int, default=0, help="0 means until Ctrl+C")
    parser.add_argument('--collisions',action='store_true')
    parser.add_argument('--collision-merge-gap-seconds',type=float,default=1.0)
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()
    if not math.isfinite(args.collision_merge_gap_seconds) or args.collision_merge_gap_seconds<0:
        parser.error('collision merge gap must be finite and nonnegative')
    if args.frames < 0 or args.timeout <= 0 or args.radius <= 0:
        parser.error("frames must be nonnegative; timeout and radius must be positive")
    # 延迟导入，未安装 CARLA 的 Python 环境也能查看 --help。
    import carla
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    if args.list_junctions:
        print(json.dumps(dict(map_name=world.get_map().name, junction_ids=sorted(junctions(world))), indent=2))
        return
    if args.junction_id is None:
        parser.error("--junction-id is required; use --list-junctions to inspect the loaded map")
    detection_config = (json.loads(Path(args.detection_config).read_text(encoding="utf-8-sig"))
                        if args.detection_config else None)
    collector = FrameCollector(world, args.junction_id, args.inventory, args.output, args.radius,
        collisions=args.collisions,collision_config=dict(merge_gap_seconds=args.collision_merge_gap_seconds),
        detection_config=detection_config,stop_reference_config=json.loads(Path(args.stop_reference_config).read_text(encoding="utf-8-sig"))
            if args.stop_reference_config else None,
        safety_config=args.safety_config, safety_map_dir=args.safety_map_dir)
    print(f"Output: {collector.output}")
    print("Observer only: does not tick. In synchronous mode start a separate tick owner.")
    print("For guaranteed per-frame collection use control_junction_phases.py --collect.")
    status = "failed"
    try:
        while args.frames == 0 or collector.frames < args.frames:
            collector.sample(world.wait_for_tick(args.timeout))
        status = "completed"
    except KeyboardInterrupt:
        status = "interrupted"
    finally:
        collector.close(status)


if __name__ == "__main__":
    main()
