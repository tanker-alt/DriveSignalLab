#!/usr/bin/env python3
"""Conservative one-approach-at-a-time signal controller for a CARLA junction."""

import argparse
import csv
import json
import math
import signal
from pathlib import Path

import carla


def hold_target_lights(lights, saved):
    """只延长目标灯的自动计时，不调用可能影响全图的 freeze API。

    手动 set_state 仍负责相位切换。十亿秒远大于单次实验长度；退出时恢复原配时。
    saved 在每次修改前填入，保证部分初始化失败时仍能恢复已修改的灯。
    """
    for light in lights:
        saved.append((light, light.get_state(), light.get_green_time(),
                      light.get_yellow_time(), light.get_red_time()))
        light.set_green_time(1e9)
        light.set_yellow_time(1e9)
        light.set_red_time(1e9)


def restore_target_lights(saved):
    for light, state, green, yellow, red in saved:
        try:
            light.set_green_time(green)
            light.set_yellow_time(yellow)
            light.set_red_time(red)
            light.set_state(state)
        except RuntimeError as exc:
            print(f"Warning: failed to restore signal {light.id}: {exc}")


# 同步模式由本控制器主动推进一帧；异步模式等待服务器产生下一帧。
# 返回帧快照，供后续使用仿真时间计时。
def advance(world, synchronous):
    if synchronous:
        world.tick()
        return world.get_snapshot()
    return world.wait_for_tick(5.0)


def signal_consistency(lights, phase):
    expected={str(light.get_opendrive_id()):
        (phase["stage"].title() if str(light.get_opendrive_id())==phase["active_signal_opendrive_id"]
         and phase["stage"] in ("GREEN","YELLOW") else "Red") for light in lights}
    actual={str(light.get_opendrive_id()):str(light.get_state()) for light in lights}
    mismatches=sorted(key for key in expected if expected[key]!=actual[key])
    return dict(expected=expected,actual=actual,mismatch_ids=mismatches)


def run_interval(world, synchronous, duration, sample_callback=None, phase=None, verify_callback=None):
    # 持续时间以仿真秒数计算，不使用现实时间；结束时可能超过目标不到一帧。
    start = world.get_snapshot().timestamp.elapsed_seconds
    current = start
    while current - start < duration:
        snapshot = advance(world, synchronous)
        current = snapshot.timestamp.elapsed_seconds
        check=verify_callback(snapshot,phase) if verify_callback else None
        sample_phase=dict(phase) if phase is not None else None
        if check is not None:
            sample_phase.update(signal_consistency="mismatch" if check["mismatch_ids"] else "matched",
                                signal_mismatch_ids="|".join(check["mismatch_ids"]))
        if sample_callback:
            # 传完整快照，采集器从该帧读取车辆状态，避免混用不同帧的位置/速度。
            if phase is None:
                sample_callback(snapshot)
            else:
                sample_callback(snapshot,dict(sample_phase,remaining_seconds=max(0.,start+duration-current),
                    scheduled_start_time=start,scheduled_end_time=start+duration))
        if check and check["mismatch_ids"]:
            raise RuntimeError("Signal schedule mismatch at frame {}: {}".format(snapshot.frame,check))


def main():
    # 配置目标路口、服务端连接，以及绿灯、黄灯和全红清空阶段的时长。
    # cycles=0 表示持续运行；一个周期包含所有灯杆各放行一次。
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--junction-id", type=int)
    parser.add_argument('--profile',help='Map profile JSON; otherwise active profile is used when no junction is supplied')
    parser.add_argument("--host")
    parser.add_argument("--port", type=int)
    parser.add_argument("--tm-port", type=int, default=8000)
    parser.add_argument("--tm-seed", type=int, default=17, help="Initialize TM before signal takeover; CARLA resets all lights when seeding")
    parser.add_argument("--green-seconds", type=float, default=12.0)
    parser.add_argument("--yellow-seconds", type=float, default=3.0)
    parser.add_argument("--all-red-seconds", type=float, default=2.0)
    parser.add_argument("--cycles", type=int, default=0, help="0 means run until Ctrl+C")
    parser.add_argument("--output", default="", help="CSV event log; default is beside this script")
    parser.add_argument("--collect", action="store_true", help="Collect every frame; enables synchronous mode")
    parser.add_argument("--collisions", action="store_true", help="Attach collision sensors; requires --collect")
    parser.add_argument('--collision-merge-gap-seconds',type=float,default=1.0,help='Adjacent callback gap per unordered actor pair')
    parser.add_argument("--fixed-delta-seconds", type=float, default=0.05)
    parser.add_argument("--collection-radius", type=float, default=120.0)
    parser.add_argument("--collection-output", default="", help="Collection output root directory")
    parser.add_argument("--inventory", default="", help="Previously exported signal inventory JSON")
    parser.add_argument("--detection-config", default="", help="User-calibrated counting sections JSON")
    parser.add_argument("--stop-reference-config",default="",help="User-verified stop waypoint references JSON")
    parser.add_argument("--reset-all-signals", action="store_true",
                        help="One-time recovery from an old globally frozen run; resets all signals")
    args = parser.parse_args()
    from evaluation.map_profile import configure_arguments, validate_world, adapt_world
    profile=configure_arguments(parser,args)
    if not math.isfinite(args.collision_merge_gap_seconds) or args.collision_merge_gap_seconds<0:
        parser.error('collision merge gap must be finite and nonnegative')
    if args.collisions and not args.collect:
        parser.error("--collisions requires --collect")
    if args.fixed_delta_seconds <= 0 or args.collection_radius <= 0:
        parser.error("fixed delta and collection radius must be positive")

    # 沿用世界当前的同步设置；同步模式下同时启用 Traffic Manager 同步。
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
    world = adapt_world(world,profile)
    validate_world(world,profile)
    original_settings = world.get_settings()
    synchronous = original_settings.synchronous_mode or args.collect
    tm = client.get_trafficmanager(args.tm_port)
    tm_sync_enabled = False

    # 只轮流控制指定路口的信号灯，并按灯杆索引、actor ID 固定放行顺序。
    lights = world.get_traffic_lights_in_junction(args.junction_id)
    if not lights:
        raise SystemExit(f"No traffic lights for junction {args.junction_id} on {world.get_map().name}.")
    lights = sorted(lights, key=lambda light: (light.get_pole_index(), light.id))
    poles = [light.get_pole_index() for light in lights]
    # 日志以 pole_index 标识灯杆，因此拒绝存在重复索引的路口。
    if len(set(poles)) != len(poles):
        raise SystemExit(f"Duplicate pole_index values in junction: {poles}")

    # 未指定输出文件时，将 CSV 集中保存到运行日志目录。
    # 先在内存中累计相位切换事件，退出控制循环时统一写入文件。
    output = Path(args.output) if args.output else (Path(__file__).parent / "evaluation/outputs/run_logs" /
        f"junction_{args.junction_id}_phase_log.csv"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    events = []

    def apply_states(stage, active_light=None, cycle=0):
        # 仅当前灯杆可以为绿灯或黄灯，其余灯杆始终设为红灯。
        # 全红阶段不传 active_light，此时路口内所有信号灯均为红灯。
        for light in lights:
            light.set_state(
                carla.TrafficLightState.Green if light.id == getattr(active_light, "id", None)
                and stage == "GREEN" else
                carla.TrafficLightState.Yellow if light.id == getattr(active_light, "id", None)
                and stage == "YELLOW" else
                carla.TrafficLightState.Red
            )
        # set_state is asynchronous. Wait for the server RPC queue before the next tick.
        # This does not advance simulation and is not a substitute for post-tick verification.
        client.apply_batch_sync([], False)
        # 每次相位切换记录一次：仿真时间、周期、阶段及各灯杆的状态。
        states = {light.get_pole_index(): str(light.get_state()) for light in lights}
        now = world.get_snapshot().timestamp.elapsed_seconds
        row = {
            "sim_time": round(now, 3), "cycle": cycle, "stage": stage,
            "active_pole": active_light.get_pole_index() if active_light else "ALL_RED",
            "active_actor": active_light.id if active_light else "",
            "pole_states_before_tick": str(states),
        }
        events.append(row)
        print(row)

    print(f"Map={world.get_map().name}; junction={args.junction_id}; poles={poles}")
    print("Conservative plan: one pole green at a time; yellow then all-red clearance between poles.")
    print("TM is seeded before takeover (CARLA resets all lights); during the run only target lights are controlled.")
    print("In synchronous mode this process ticks the world; start vehicle spawning in another process afterward.")

    saved_signals = []
    signal_checks = None
    collector = None
    settings_changed = False
    collection_status = "failed"
    try:
        if args.reset_all_signals:
            # 仅显式要求时修复旧版脚本遗留的全图冻结；正常实验不会重置其他路口。
            world.freeze_all_traffic_lights(False)
            world.reset_all_traffic_lights()
        elif any(light.is_frozen() for light in lights):
            raise RuntimeError("Signals are already frozen. Restart with --reset-all-signals to recover the old run.")
        # TM seed resets the whole light network; do this only before schedule ownership/collection.
        tm.set_random_device_seed(args.tm_seed)
        client.apply_batch_sync([], False)
        if args.collect:
            from evaluation.collector import FrameCollector, DEFAULT_INVENTORY, DEFAULT_OUTPUT
            settings = world.get_settings()
            # 物理子步必须能覆盖一个固定仿真步，否则运动学数据不可靠。
            if settings.substepping and args.fixed_delta_seconds > settings.max_substep_delta_time * settings.max_substeps:
                raise ValueError("fixed delta exceeds physics substep capacity")
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = args.fixed_delta_seconds
            world.apply_settings(settings)
            settings_changed = True
            collector = FrameCollector(world, args.junction_id,
                inventory=args.inventory or None,
                output_root=args.collection_output or DEFAULT_OUTPUT,
                radius=args.collection_radius, mode="controller_owned_tick", collisions=args.collisions,
                collision_config=dict(merge_gap_seconds=args.collision_merge_gap_seconds),
                detection_config=json.loads(Path(args.detection_config).read_text(encoding="utf-8-sig"))
                    if args.detection_config else None,
                stop_reference_config=json.loads(Path(args.stop_reference_config).read_text(encoding="utf-8-sig"))
                    if args.stop_reference_config else None)
            print(f"Frame collection output: {collector.output}")
        if synchronous:
            # 本进程必须是唯一 tick 所有者。采集完成后才允许推进下一帧。
            tm.set_synchronous_mode(True)
            tm_sync_enabled = True
        sample = collector.sample if collector else None
        check_path=(collector.output/"signal_checks.jsonl" if collector else output.with_suffix(".signal_checks.jsonl"))
        signal_checks=check_path.open("w",encoding="utf-8")
        if collector:
            (collector.output/"controller_config.json").write_text(json.dumps(dict(
                tm_seed=args.tm_seed,tm_port=args.tm_port,green_seconds=args.green_seconds,
                yellow_seconds=args.yellow_seconds,all_red_seconds=args.all_red_seconds,
                signal_write_barrier="apply_batch_sync_empty_no_tick",mismatch_policy="record_then_fail"),indent=2),encoding="utf-8")
        def verify(snapshot,phase):
            check=signal_consistency(lights,phase)
            signal_checks.write(json.dumps(dict(frame=snapshot.frame,sim_time=snapshot.timestamp.elapsed_seconds,
                phase_id=phase["phase_id"],**check))+"\n")
            signal_checks.flush()
            return check
        def interval(stage,duration,active=None,cycle=0,next_stage=None,next_active=None):
            apply_states(stage,active,cycle)
            signal_id=str(active.get_opendrive_id()) if active else ""
            next_id=str(next_active.get_opendrive_id()) if next_active else ""
            phase=dict(source="fixed_schedule",phase_id=signal_id+":"+stage,stage=stage,cycle=cycle,
                active_signal_opendrive_id=signal_id,
                next_phase_id=next_id+":"+next_stage if next_stage else None)
            run_interval(world,synchronous,duration,sample,phase,verify)
        hold_target_lights(lights, saved_signals)
        # 首次放行前先保持全红，为路口内已有车辆留出清空时间。
        interval("ALL_RED",args.all_red_seconds,next_stage="GREEN",next_active=lights[0])

        cycle = 0
        while args.cycles == 0 or cycle < args.cycles:
            cycle += 1
            for index,light in enumerate(lights):
                # 每个灯杆依次经历“绿灯 → 黄灯 → 全红”，然后轮到下一灯杆。
                interval("GREEN",args.green_seconds,light,cycle,"YELLOW",light)
                interval("YELLOW",args.yellow_seconds,light,cycle,"ALL_RED")
                finishing=args.cycles and cycle==args.cycles and index==len(lights)-1
                interval("ALL_RED",args.all_red_seconds,cycle=cycle,
                    next_stage=None if finishing else "GREEN",
                    next_active=None if finishing else lights[(index+1)%len(lights)])
        collection_status = "completed"
    except KeyboardInterrupt:
        collection_status = "interrupted"
        print("\nController stopped by user.")
    finally:
        # 清理过程中重复 Ctrl+C 不应中断配时恢复或采集文件关闭。
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        restore_target_lights(saved_signals)
        if tm_sync_enabled:
            # 仅当本脚本启用了 Traffic Manager 同步时，在此将其关闭。
            try:
                tm.set_synchronous_mode(False)
            except RuntimeError:
                pass
        if settings_changed:
            # 恢复启动前的世界设置，避免退出后服务器停在无人 tick 的同步模式。
            try:
                world.apply_settings(original_settings)
            except RuntimeError as exc:
                print(f"Warning: could not restore world settings: {exc}")
        if collector:
            collector.close(collection_status)
        if signal_checks:
            signal_checks.close()
        if events:
            # CSV 列顺序与事件字典一致；同名输出文件会被覆盖。
            with output.open("w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=list(events[0].keys()))
                writer.writeheader()
                writer.writerows(events)
            print(f"Phase log saved: {output}")


if __name__ == "__main__":
    main()
