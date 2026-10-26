#!/usr/bin/env python3
"""在目标路口各信号灯关联的进口车道生成 Traffic Manager 测试车辆。

本脚本负责生成、托管和清理测试车，不负责推进同步仿真。
正式逐帧数据由 control_junction_phases.py --collect 保存；这里保留的
每秒遥测仅用于调试，不能用来判断精确越线时刻或计算逐帧指标。
"""

import argparse
import csv
import json
import math
import random
import signal
import time
from pathlib import Path
from uuid import uuid4

import carla

from traffic_scenario import LanePath, actual_counts, fleet_plan, measure_box, fit_first_distance

DEFAULT_TRAFFIC_CONFIG = Path(__file__).parent / "evaluation" / "configs" / "traffic.json"


def destroy_owned_vehicles(client, world, vehicles):
    """批量销毁并核验，只操作本次生成的 actor；不要求控制器再 tick。

    原实现将停用 autopilot 与 destroy 放在同一 try 中，前者失败会跳过销毁。
    DestroyActor 自身会移除 actor，无需先逐辆发停用 RPC。
    """
    remaining = {vehicle.id for vehicle in vehicles}
    for attempt in range(3):
        if not remaining:
            break
        try:
            # False：绝不从车辆进程推进同步世界。
            replies = client.apply_batch_sync([carla.command.DestroyActor(i) for i in sorted(remaining)], False)
            for reply in replies:
                if reply.has_error():
                    print(f"Destroy response: {reply.error}")
            remaining &= {actor.id for actor in world.get_actors()}
        except RuntimeError as exc:
            print(f"Cleanup attempt {attempt + 1} failed: {exc}")
        if remaining and attempt < 2:
            time.sleep(0.2)
    if remaining:
        print(f"WARNING: cleanup unverified; remaining actor IDs: {sorted(remaining)}")
    else:
        print(f"Cleanup verified: all {len(vehicles)} owned vehicles removed.")
    return remaining


def advance(world):
    # 同步模式下由相位控制器独占 world.tick()，本进程只等待新帧。
    # 控制器停止推进后，这里可能等待超时；finally 仍会尝试清理车辆。
    return world.wait_for_tick(5.0)


def main():
    # 距离单位为米。vehicles-per-approach 实际按每个信号灯分配车辆，
    # 并非每条车道各生成这么多辆；一个信号灯可关联多个停止线车道。
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--junction-id", type=int)
    parser.add_argument('--profile',help='Map profile JSON; otherwise active profile is used when no junction is supplied')
    parser.add_argument("--host")
    parser.add_argument("--port", type=int)
    parser.add_argument("--tm-port", type=int, default=8000)
    parser.add_argument("--vehicles-per-approach", type=int, default=None)
    parser.add_argument("--first-distance", type=float, default=None, help="Preferred first distance before CARLA stop reference; shortened if needed to fit the full fleet")
    parser.add_argument("--traffic-config", type=Path, default=DEFAULT_TRAFFIC_CONFIG)
    parser.add_argument("--spawn-gap", type=float, default=None,
                        help="Initial bumper gap in metres; overrides traffic config (default 2)")
    parser.add_argument("--seed", type=int, default=17, help="Fleet/Python seed only; TM seed belongs to controller --tm-seed")
    parser.add_argument("--output", default="", help="CSV telemetry path; default is beside this script")
    parser.add_argument("--verbose", action="store_true", help="Print per-vehicle debug samples")
    parser.add_argument("--duration-seconds", type=float, default=0,
                        help="Simulation seconds after spawning; 0 runs until Ctrl+C")
    parser.add_argument("--vehicle-blueprint", default="random",
                        help="random selects a mixed fleet; optionally supply an exact blueprint ID")
    parser.add_argument("--route", choices=["Straight", "Left", "Right", "random"], default="random",
                        help="Requested movement at next junction; validate lane compatibility in CARLA")
    args = parser.parse_args()
    from evaluation.map_profile import configure_arguments, validate_world, adapt_world
    profile=configure_arguments(parser,args)
    generated=bool(profile and profile.get('_design'))
    if args.vehicles_per_approach is None:args.vehicles_per_approach=3 if generated else 2
    if args.first_distance is None:args.first_distance=65.0 if generated else 18.0
    config = json.loads(args.traffic_config.read_text(encoding="utf-8-sig"))
    gap = config["spawn_gap_m"] if args.spawn_gap is None else args.spawn_gap
    if not math.isfinite(gap) or gap < 0 or not math.isfinite(args.first_distance) or args.first_distance <= 0:
        parser.error("spawn gap must be finite and nonnegative; first distance must be finite and positive")
    if not math.isfinite(args.duration_seconds) or args.duration_seconds < 0:
        parser.error("duration-seconds must be nonnegative")
    if args.vehicles_per_approach < 1:
        raise SystemExit("--vehicles-per-approach must be at least 1")

    # Only seed local fleet planning here. CARLA TM seeding resets ALL traffic lights.
    random.seed(args.seed)
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
    world = adapt_world(world,profile)
    validate_world(world,profile)
    map_name = world.get_map().name
    tm = client.get_trafficmanager(args.tm_port)
    # The world owner seeds TM before taking control of signals. Never reseed mid-run.
    lights = world.get_traffic_lights_in_junction(args.junction_id)
    if not lights:
        raise SystemExit(f"No traffic lights for junction {args.junction_id} on {world.get_map().name}.")
    lights = sorted(lights, key=lambda light: light.get_pole_index())

    approaches = [(light, sorted(light.get_stop_waypoints(),
                  key=lambda wp: (wp.road_id, wp.section_id, wp.lane_id))) for light in lights]
    approaches = [(light, stops) for light, stops in approaches if stops]
    lane_lights = {(wp.road_id, wp.section_id, wp.lane_id): light
                   for light, stops in approaches for wp in stops}
    if getattr(world, 'design', {}).get('signal_policy') == 'one_signal_per_incoming_lane':
        # Preserve --vehicles-per-approach as a physical direction count.
        grouped = {}
        for light, stops in approaches:
            for wp in stops:
                grouped.setdefault(wp.road_id, []).append(wp)
        approaches = [(lane_lights[(stops[0].road_id, stops[0].section_id, stops[0].lane_id)],
                       sorted(stops, key=lambda wp: wp.lane_id))
                      for _, stops in sorted(grouped.items())]
    if not approaches:
        raise SystemExit("No usable signal stop waypoints")
    plan, requested_counts = fleet_plan(world.get_blueprint_library().filter("vehicle.*"),
        len(approaches) * args.vehicles_per_approach, config, random, args.vehicle_blueprint)

    output = Path(args.output) if args.output else (Path(__file__).parent / "evaluation/outputs/run_logs" /
        f"junction_{args.junction_id}_vehicle_telemetry.csv"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    # vehicles 只保存本脚本生成的车辆，退出时不会销毁用户另行创建的 VLA 车。
    # metadata 是生成时的进口道归属，不随车辆驶入路口或驶出路口而更新。
    vehicles = []
    metadata = {}
    spawn_rows = []
    plan_rows = [dict(index=i, category=category, vehicle_blueprint=bp.id, status="pending")
                 for i, (category, bp) in enumerate(plan)]
    manifest = output.with_name("spawn_manifest_" + uuid4().hex[:10] + ".json")
    stop_requested = False
    failure_reason = None

    def request_stop(signum, frame):
        # 不在 spawn RPC 返回与登记车辆之间抛异常，避免刚生成的车失去所有权记录。
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)

    def check_stop():
        if stop_requested:
            raise KeyboardInterrupt

    def save_manifest(status):
        temporary = manifest.with_suffix(".tmp")
        temporary.write_text(json.dumps(dict(schema_version="2.0", map_name=map_name,
            junction_id=args.junction_id, seed=args.seed, seed_scope="fleet_python_only",
            tm_seed_source="external_world_owner", requested_route=args.route,
            traffic_config=config, requested_counts=requested_counts, actual_counts=actual_counts(plan_rows),
            spawn_gap_m=gap, gap_tolerance_m=.05, preferred_first_distance_m=args.first_distance,
            first_distance_policy="fit_full_lane_fleet_toward_stop_reference", status=status, failure_reason=failure_reason,
            plan=plan_rows, vehicles=spawn_rows,
            gap_definition="bbox_projection_onto_sampled_lane_centerline_before_release"),
            indent=2), encoding="utf-8")
        temporary.replace(manifest)
    print(f"Map={map_name}; junction={args.junction_id}; approaches={len(approaches)}; signals={len(lights)}")

    status = "failed"
    try:
        save_manifest("spawning")
        boxes, paths, leaders = {}, {}, {}
        lane_boxes, first_distances = {}, {}
        index = 0
        # Measure and fit all lanes before placing any final vehicle.
        for light, stop_wps in approaches:
            for i in range(args.vehicles_per_approach):
                check_stop()
                stop = stop_wps[i % len(stop_wps)]
                key = (stop.road_id, stop.section_id, stop.lane_id)
                if key not in paths:
                    paths[key] = LanePath(stop)
                blueprint = plan[index][1]
                index += 1
                if blueprint.id not in boxes:
                    boxes[blueprint.id] = measure_box(world, blueprint, paths[key],
                        min(args.first_distance, paths[key].stations[-1]/2), vehicles, check_stop)
                lane_boxes.setdefault(key, []).append(boxes[blueprint.id])
        for key, planned_boxes in lane_boxes.items():
            try:
                first_distances[key] = fit_first_distance(paths[key], planned_boxes, args.first_distance, gap)
            except ValueError as exc:
                raise ValueError('Lane {}: {}'.format(key, exc)) from exc
            if first_distances[key] != args.first_distance:
                print('Lane {}: first distance adjusted {:.2f} -> {:.2f} m to fit full fleet '
                      '(upstream length {:.2f} m)'.format(key, args.first_distance,
                          first_distances[key], paths[key].stations[-1]))
        plan_index = 0
        for light, stop_wps in approaches:
            for i in range(args.vehicles_per_approach):
                check_stop()
                stop_wp = stop_wps[i % len(stop_wps)]
                key = (stop_wp.road_id, stop_wp.section_id, stop_wp.lane_id)
                light = lane_lights[key]
                if key not in paths:
                    paths[key] = LanePath(stop_wp)
                path = paths[key]
                category, blueprint = plan[plan_index]
                task = plan_rows[plan_index]
                plan_index += 1
                task["status"] = "attempting"
                save_manifest("spawning")
                if blueprint.id not in boxes:
                    boxes[blueprint.id] = measure_box(world, blueprint, path,
                        args.first_distance, vehicles, check_stop)
                box = boxes[blueprint.id]
                leader = leaders.get(key)
                leader_rear = path.interval(leader.bounding_box, leader.get_transform())[1] if leader else None
                distance = (path.following_distance(leader_rear, box, gap)
                            if leader else first_distances[key])
                blueprint.set_attribute("role_name", f"junction_test_p{light.get_pole_index()}_{i}")
                if blueprint.has_attribute("color"):
                    colors = blueprint.get_attribute("color").recommended_values
                    if colors:
                        blueprint.set_attribute("color", random.choice(colors))
                spawned = None
                used_distance = distance
                for retry in range(4):
                    # 首车可向上游寻找空位；后车不能靠扩大净间距冒充满足配置。
                    used_distance = distance + (retry * 4.0 if leader is None else 0.0)
                    transform = path.transform(used_distance)
                    spawned = world.try_spawn_actor(blueprint, transform)
                    if spawned:
                        vehicles.append(spawned)
                        spawned.set_simulate_physics(False)
                        check_stop()
                        break
                if spawned is None:
                    task["status"] = "spawn_failed"
                    raise RuntimeError("Planned vehicle could not spawn; fleet aborted without substitution")
                snapshot = advance(world)
                state = snapshot.find(spawned.id)
                if state is None or (leader and snapshot.find(leader.id) is None):
                    raise RuntimeError("Placed fleet missing from the server snapshot")
                front, rear = path.interval(spawned.bounding_box, state.get_transform())
                if leader:
                    leader_rear = path.interval(leader.bounding_box, snapshot.find(leader.id).get_transform())[1]
                measured_gap = front - leader_rear if leader else None
                if measured_gap is not None and abs(measured_gap-gap) > .05:
                    task["status"] = "gap_failed"
                    raise RuntimeError("Measured initial gap differs from configured gap")
                leaders[key] = spawned
                metadata[spawned.id] = {
                    "pole_index": light.get_pole_index(),
                    "signal_actor": light.id,
                    "road_id": stop_wp.road_id,
                    "lane_id": stop_wp.lane_id,
                }
                row = {
                    "vehicle_id": spawned.id, "pole_index": light.get_pole_index(),
                    "signal_actor": light.id, "road_id": stop_wp.road_id,
                    "lane_id": stop_wp.lane_id, "spawn_distance_m": used_distance,
                    "vehicle_blueprint": spawned.type_id, "requested_route": args.route,
                    "category": category, "section_id": stop_wp.section_id,
                    "leader_id": leader.id if leader else None, "measured_gap_m": measured_gap,
                    "front_station_m": front, "rear_station_m": rear,
                    "placement_frame": snapshot.frame,
                }
                spawn_rows.append(row)
                task.update(status="spawned", vehicle_id=spawned.id)
                save_manifest("spawning")
                print(f"Spawned {row}")

        # 构建车队期间冻结本次车辆的物理，避免前车在计算后车间距时驶离。
        # 只在所有计划车辆成功后释放；运行中的跟车距离由 TM 自行决定。
        for vehicle, row in zip(vehicles, spawn_rows):
            check_stop()
            vehicle.set_simulate_physics(True)
            vehicle.set_autopilot(True, args.tm_port)
            tm.ignore_lights_percentage(vehicle, 0)
            tm.auto_lane_change(vehicle, False)
            if args.route != "random":
                tm.set_route(vehicle, [args.route])
            row["release_frame"] = world.get_snapshot().frame
        save_manifest("running")
        print(f"Spawned {len(vehicles)} vehicles. Keep this process open during the run.")
        print("Start this script after the phase controller. Ctrl+C destroys these test vehicles and writes telemetry.")
        # 以下是旧的低频调试日志：以仿真时间每秒采一次，在退出时统一写出。
        # 数据积存在内存，长时间实验请以 evaluation 的持续写出结果为准。
        rows = []
        last_sample = -1.0
        start_time = world.get_snapshot().timestamp.elapsed_seconds
        while True:
            check_stop()
            snapshot = advance(world)
            check_stop()
            now = snapshot.timestamp.elapsed_seconds
            # 自动结束仍走 finally，复用批量销毁和残留核验。
            if args.duration_seconds and now - start_time >= args.duration_seconds:
                break
            if now - last_sample < 1.0:
                continue
            last_sample = now
            for vehicle in vehicles:
                if not vehicle.is_alive:
                    continue
                info = metadata[vehicle.id]
                # 直接读取 actor 缓存，独立进程下不保证所有字段严格属于同一帧。
                # evaluation 采集器则使用 ActorSnapshot 获取同帧运动学数据。
                transform = vehicle.get_transform()
                velocity = vehicle.get_velocity()
                # 只有进入相应影响区域时才可能返回信号；None 不表示不存在受控信号。
                traffic_light = vehicle.get_traffic_light()
                row = {
                    "sim_time": round(now, 3), "vehicle_id": vehicle.id,
                    "pole_index": info["pole_index"], "mapped_signal_actor": info["signal_actor"],
                    # 这里的 road/lane 是初始停止线车道，不是车辆当前车道。
                    "road_id": info["road_id"], "lane_id": info["lane_id"],
                    "x": round(transform.location.x, 3), "y": round(transform.location.y, 3),
                    "speed_kmh": round(velocity.length() * 3.6, 3),
                    "reported_signal_actor": traffic_light.id if traffic_light else "",
                    "reported_signal_state": str(vehicle.get_traffic_light_state()),
                }
                rows.append(row)
                if args.verbose:
                    print(row)
        status = "completed"
    except KeyboardInterrupt:
        status = "interrupted"
        print("\nVehicle experiment stopped by user.")
    except Exception as exc:
        failure_reason = str(exc)
        raise
    finally:
        # 二次 Ctrl+C 之前会打断 finally，导致只销毁前几辆；清理阶段屏蔽它。
        # 每个 RPC 设置有限超时，服务器断开时会报告未核验 ID，不假报成功。
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        client.set_timeout(5.0)
        remaining = destroy_owned_vehicles(client, world, vehicles)
        save_manifest(status if not remaining else "cleanup_unverified")
        if 'rows' in locals():
            # 同名调试 CSV 会覆盖；正式采集的 outputs/<run_id> 不受影响。
            with output.open("w", newline="", encoding="utf-8") as f:
                fields = list(rows[0].keys()) if rows else [
                    "sim_time", "vehicle_id", "pole_index", "mapped_signal_actor",
                    "road_id", "lane_id", "x", "y", "speed_kmh",
                    "reported_signal_actor", "reported_signal_state",
                ]
                writer = csv.DictWriter(f, fieldnames=fields)
                writer.writeheader()
                writer.writerows(rows)
            print(f"Vehicle telemetry saved: {output}")
        if spawn_rows:
            print(f"Spawn summary: {len(spawn_rows)} vehicles were initially created.")
            print(f"Spawn manifest: {manifest}")


if __name__ == "__main__":
    main()
