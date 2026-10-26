"""按帧保存评估真值。车辆运动学来自同一个 WorldSnapshot。

注意：CARLA 的 ActorSnapshot 不包含信号灯颜色；灯色仍需向 actor 查询。
因此严格采集应由唯一 tick 所有者在两次 tick 之间调用 sample()，且期间
不能由其他进程切换信号。独立监听模式只能记录观测值，不能保证不漏帧。
"""

import csv
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4



DEFAULT_INVENTORY = Path(__file__).resolve().parents[2] / "Toolkits" / "MapTools" / "junction_signal_inventory.json"
DEFAULT_OUTPUT = Path(__file__).resolve().parent / "outputs"


def lane_key(wp):
    """lane_id 在不同道路上会重复，不能单独用作车道主键。"""
    return (int(wp.road_id), int(wp.section_id), int(wp.lane_id))


def norm(vector):
    return math.sqrt(vector.x ** 2 + vector.y ** 2 + vector.z ** 2)


class FrameCollector:
    """持久化一场实验；不生成车辆、不控制信号，也不调用 world.tick()。"""

    def __init__(self, world, junction_id=1427, inventory=None,
                 output_root=DEFAULT_OUTPUT, radius=120.0, mode="observer", collisions=False,
                 detection_config=None, arrival_distance_m=None, exit_distance_m=5.0, stop_reference_config=None,
                 collision_config=None, safety_config=None, safety_map_dir=None):
        if radius <= 0:
            raise ValueError("radius must be positive")
        if collisions:
            collision_config=dict(collision_config or {})
            if set(collision_config)-{'merge_gap_seconds'}:
                raise ValueError('Unknown collision configuration field')
            gap=collision_config.get('merge_gap_seconds',1.0)
            if not math.isfinite(gap) or gap<0:
                raise ValueError('Collision merge gap must be finite and nonnegative')
        if collisions:
            collision_config=dict(collision_config or {})
            if set(collision_config)-{'merge_gap_seconds'}:
                raise ValueError('Unknown collision configuration field')
            gap=collision_config.get('merge_gap_seconds',1.0)
            if not math.isfinite(gap) or gap<0:
                raise ValueError('Collision merge gap must be finite and nonnegative')
        self.world = world
        self.map = world.get_map()
        self.junction_id = junction_id
        self.radius = radius
        if inventory is None:
            try:
                from .map_context import read_inventory
                from .topology import build
            except ImportError:
                from map_context import read_inventory
                from topology import build
            data = read_inventory(world, junction_id)
        else:
            data = json.loads(Path(inventory).read_text(encoding="utf-8-sig"))
        try:
            from .approach_geometry import build as build_approaches, DEFAULT_APPROACH_CONFIG
        except ImportError:
            from approach_geometry import build as build_approaches, DEFAULT_APPROACH_CONFIG
        approach_config=json.loads(DEFAULT_APPROACH_CONFIG.read_text(encoding="utf-8"))
        approach_geometry=(build_approaches(world,data,approach_config,stop_reference_config)
                           if hasattr(self.map,"to_opendrive") else None)
        if stop_reference_config is not None and approach_geometry is None:
            raise ValueError("Stop reference calibration requires live map geometry")
        if inventory is None and detection_config is None:
            detection_config = build(world, junction_id, arrival_distance_m, exit_distance_m, approach_geometry)
        # Explicit legacy inventory requires explicit counting sections, never implicit Town05 lines.
        if detection_config is None:
            detection_config = dict(map_name=self.map.name, junction_id=junction_id, lines=[])
        if (detection_config["map_name"].split("/")[-1] != self.map.name.split("/")[-1]
                or int(detection_config["junction_id"]) != junction_id
                or any(int(line["junction_id"]) != junction_id for line in detection_config["lines"])):
            raise ValueError("Detection configuration does not match loaded map/junction")
        try:
            from .detection_lines import DetectionLines
        except ImportError:
            from detection_lines import DetectionLines
        DetectionLines(detection_config["lines"])
        # 使用地图名和 junction_id 校验，拒绝把其他地图的拓扑静默套用过来。
        if data["map_name"].split("/")[-1] != self.map.name.split("/")[-1]:
            raise ValueError("Inventory map does not match current CARLA map")
        if int(data["junction_id"]) != junction_id:
            raise ValueError("Inventory junction_id does not match requested junction")
        live = list(world.get_traffic_lights_in_junction(junction_id))
        self.lights = {}
        for light in live:
            key = str(light.get_opendrive_id())
            if key in self.lights:
                raise ValueError("Duplicate live signal OpenDRIVE ID: " + key)
            self.lights[key] = light
        expected = {str(item["opendrive_id"]) for item in data["traffic_lights"]}
        if set(self.lights) != expected:
            raise ValueError("Live signals differ from inventory; verify topology first")

        # 以已经导出的停止线参考点为准，不使用历史 actor_id。
        # 沿 previous() 向上游追溯车道，因此跨 road/section 后仍能建立关联。
        # 若同一车道关联多个信号，输出 ambiguous，不擅自猜测车辆转向意图。
        self.lane_signals = {}
        self.stops = []
        for item in data["traffic_lights"]:
            signal_id = str(item["opendrive_id"])
            for stop in item["stop_waypoints"]:
                wp = self.map.get_waypoint_xodr(stop["road_id"], stop["lane_id"], stop["s"])
                if wp is None or wp.section_id != stop["section_id"]:
                    raise ValueError("Cannot resolve inventory stop waypoint")
                self.stops.append((signal_id, stop))
                pending = [(wp, 0.0)]
                seen = set()
                while pending:
                    current, distance = pending.pop()
                    marker = (lane_key(current), round(current.s, 1))
                    if marker in seen:
                        continue
                    seen.add(marker)
                    self.lane_signals.setdefault(lane_key(current), set()).add(signal_id)
                    if distance < radius:
                        for previous in current.previous(2.0):
                            # 不穿过上一个路口继续追踪，否则会错误绑定相邻路口。
                            if not previous.is_junction:
                                pending.append((previous, distance + 2.0))
        if self.stops:
            self.center = tuple(sum(s["location"][axis] for _, s in self.stops) / len(self.stops)
                                for axis in ("x", "y"))
        elif "center_xy" in data:
            self.center = tuple(data["center_xy"])
        else:
            raise ValueError("No stop references or junction center in inventory")
        self.detection_config = detection_config
        self.approach_geometry = approach_geometry
        self.run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ_") + uuid4().hex[:8]
        self.output = Path(output_root) / self.run_id
        self.output.mkdir(parents=True, exist_ok=False)
        # Geometry/configuration snapshots are raw provenance, not metric output.
        (self.output/'detection_lines_used.json').write_text(json.dumps(detection_config,indent=2),encoding='utf-8')
        if approach_geometry is not None:
            (self.output/'approach_geometry_used.json').write_text(json.dumps(approach_geometry,indent=2),encoding='utf-8')
        self.handles = []
        self.last_frame = None
        self.frames = self.vehicle_rows = self.missing_frames = 0
        self.closed = False
        base = ["run_id", "frame", "sim_time", "junction_id"]
        self.writers = {}
        fields = {
            "frames": base + ["delta_seconds", "frame_gap", "vehicle_count", "signal_count", "end_frame", "read_overlapped_tick"],
            "signals": base + ["signal_actor_id", "signal_opendrive_id", "pole_index", "state", "is_frozen"],
            "phases": base + ["source","phase_id","stage","cycle","active_signal_opendrive_id",
                              "next_phase_id","remaining_seconds","scheduled_start_time","scheduled_end_time",
                              "signal_consistency","signal_mismatch_ids"],
            "vehicles": base + ["vehicle_id", "role_name", "type_id", "x", "y", "z", "pitch", "yaw", "roll",
                                "vx", "vy", "vz", "speed_mps", "ax", "ay", "az", "acceleration_mps2",
                                "road_id", "section_id", "lane_id", "lane_s", "is_junction", "current_junction_id",
                                "signal_match", "signal_candidates", "signal_actor_id", "signal_opendrive_id", "signal_state",
                                "stop_plane_front_distance_m", "lane_width_m", "bbox_vertices_world_json",
                                "legal_direction_status", "legal_forward_x", "legal_forward_y", "legal_forward_z",
                                "lk_target_lane", "lk_target_source", "lk_lane_change_allowed",
                                "longitudinal_accel_mps2","lateral_accel_mps2",
                                "throttle","brake","steer","reverse","hand_brake","control_source"],
        }
        try:
            for name, columns in fields.items():
                handle = (self.output / (name + ".csv")).open("w", newline="", encoding="utf-8-sig")
                self.handles.append(handle)
                writer = csv.DictWriter(handle, fieldnames=columns)
                writer.writeheader()
                self.writers[name] = writer
            settings = world.get_settings()
            self.metadata = {
                "schema_version": "1.2", "run_id": self.run_id, "map": self.map.name,
                "junction_id": junction_id, "mode": mode, "radius_m": radius,
                "region_center_xy": self.center, "synchronous_mode": settings.synchronous_mode,
                "fixed_delta_seconds": settings.fixed_delta_seconds,
                "inventory": data, "status": "running",
                "notes": "Blank means unavailable. Distance is a local stop-plane projection, NOT route distance."
            }
            self._write_metadata()
            self.flush()
            self.collision_recorder = None
            if collisions:
                try:
                    from .collision_recorder import CollisionRecorder
                except ImportError:
                    from collision_recorder import CollisionRecorder
                self.collision_recorder = CollisionRecorder(world, self.output,**(collision_config or {}))
            self.metadata["collision_recording"] = collisions
            self.metadata['collision_event_config']=(dict(merge_gap_seconds=self.collision_recorder.merge_gap_seconds)
                                                       if self.collision_recorder else None)
            self._write_metadata()
            try:
                from .observations import ObservationWriter
            except ImportError:
                from observations import ObservationWriter
            self.observations = ObservationWriter(self.output)
            self.last_observation = None
            self.metadata['data_products'] = dict(
                agent_observations='observations/agent_observations.jsonl',
                legacy_files='Root CSV/JSON remain audit and replay sources; do not expose the whole run to agents')
            self._write_metadata()
        except BaseException:
            for handle in self.handles:
                handle.close()
            raise

    def _write_metadata(self):
        # 临时文件替换，减少进程中断造成 metadata JSON 半截写入的可能性。
        path = self.output / "metadata.json"
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps(self.metadata, ensure_ascii=False, indent=2), encoding="utf-8")
        temporary.replace(path)

    def sample(self, snapshot, phase=None, planner_targets=None):
        """每个 snapshot 只采一次；显式记录帧号跳变，不补造丢失帧。"""
        if self.closed:
            raise RuntimeError("Collector is closed")
        if self.last_frame is not None and snapshot.frame <= self.last_frame:
            raise RuntimeError("Non-increasing frame; world may have been reloaded")
        gap = 0 if self.last_frame is None else snapshot.frame - self.last_frame - 1
        self.missing_frames += gap
        base = dict(run_id=self.run_id, frame=snapshot.frame,
                    sim_time=snapshot.timestamp.elapsed_seconds, junction_id=self.junction_id)
        phase = dict(phase) if phase is not None else dict(source="unavailable")
        self.writers["phases"].writerow(dict(base,**phase))
        states = {}
        for key, light in self.lights.items():
            states[key] = str(light.get_state())
            self.writers["signals"].writerow(dict(base, signal_actor_id=light.id,
                signal_opendrive_id=key, pole_index=light.get_pole_index(),
                state=states[key], is_frozen=light.is_frozen()))

        count = 0
        vehicle_rows = []
        for actor in self.world.get_actors().filter("vehicle.*"):
            # Reserved setup-only role: dimensions are measured before final fleet placement.
            if actor.attributes.get("role_name") == "junction_geometry_probe":
                continue
            state = snapshot.find(actor.id)
            # 车辆可能刚生成/销毁，只接受本帧快照里确实存在的 actor。
            if state is None:
                continue
            transform = state.get_transform()
            loc = transform.location
            if math.hypot(loc.x - self.center[0], loc.y - self.center[1]) > self.radius:
                continue
            if self.collision_recorder:
                self.collision_recorder.attach(actor, snapshot.frame)
            velocity, acceleration = state.get_velocity(), state.get_acceleration()
            # 不把路外车辆强行投影到最近车道；未匹配时车道字段留空。
            wp = self.map.get_waypoint(loc, project_to_road=False)
            candidates = sorted(self.lane_signals.get(lane_key(wp), set())) if wp else []
            signal_id = candidates[0] if len(candidates) == 1 else None
            row = dict(base, vehicle_id=actor.id, role_name=actor.attributes.get("role_name", ""),
                       type_id=actor.type_id, x=loc.x, y=loc.y, z=loc.z,
                       pitch=transform.rotation.pitch, yaw=transform.rotation.yaw, roll=transform.rotation.roll,
                       vx=velocity.x, vy=velocity.y, vz=velocity.z, speed_mps=norm(velocity),
                       ax=acceleration.x, ay=acceleration.y, az=acceleration.z,
                       acceleration_mps2=norm(acceleration), signal_candidates="|".join(candidates),
                       signal_match="unique" if signal_id else ("ambiguous" if candidates else "unmapped"))
            vertices = actor.bounding_box.get_world_vertices(transform)
            row["bbox_vertices_world_json"] = json.dumps([[v.x, v.y, v.z] for v in vertices])
            angle=math.radians(transform.rotation.yaw)
            row["longitudinal_accel_mps2"]=acceleration.x*math.cos(angle)+acceleration.y*math.sin(angle)
            row["lateral_accel_mps2"]=-acceleration.x*math.sin(angle)+acceleration.y*math.cos(angle)
            if hasattr(actor,"get_control"):
                control=actor.get_control()
                row.update({key:getattr(control,key) for key in ("throttle","brake","steer","reverse","hand_brake")})
                row["control_source"]="actor_cache_between_ticks"
            try:
                from .safety_config import legal_direction_fields
            except ImportError:
                from safety_config import legal_direction_fields
            row.update(legal_direction_fields(wp))
            target = (planner_targets or {}).get(actor.id)
            if target is not None:
                if target.get('frame') != snapshot.frame or type(target.get('lane_change_allowed')) is not bool:
                    raise ValueError('Planner LK target must have current frame and explicit lane_change_allowed boolean')
                row.update(lk_target_lane='{}:{}:{}'.format(target['road_id'],target['section_id'],target['lane_id']),
                           lk_target_source='planner',lk_lane_change_allowed=target['lane_change_allowed'])
            if wp:
                row.update(road_id=wp.road_id, section_id=wp.section_id, lane_id=wp.lane_id,
                           lane_width_m=getattr(wp, "lane_width", ""),
                           lane_s=wp.s, is_junction=wp.is_junction,
                           current_junction_id=wp.get_junction().id if wp.is_junction else "")
            if signal_id:
                row.update(signal_actor_id=self.lights[signal_id].id,
                           signal_opendrive_id=signal_id, signal_state=states[signal_id])
                # 仅在停止线所在 road/section/lane 输出局部平面距离。
                # 用包围盒世界坐标顶点求最前端，考虑车长、车身朝向和 bbox 偏移。
                # 弯路上的沿路径距离需另行计算，不能把这里的投影冒充为它。
                stops = [s for key, s in self.stops if key == signal_id and
                         (s["road_id"], s["section_id"], s["lane_id"]) == lane_key(wp)]
                if len(stops) == 1:
                    stop = stops[0]
                    angle = math.radians(stop["yaw"])
                    ux, uy = math.cos(angle), math.sin(angle)
                    anchor = stop["location"]
                    projections = [(v.x - anchor["x"]) * ux + (v.y - anchor["y"]) * uy
                                   for v in vertices]
                    row["stop_plane_front_distance_m"] = -max(projections)
            self.writers["vehicles"].writerow(row)
            vehicle_rows.append(row)
            count += 1
        end_frame = self.world.get_snapshot().frame
        self.writers["frames"].writerow(dict(base, delta_seconds=snapshot.timestamp.delta_seconds,
            frame_gap=gap, vehicle_count=count, signal_count=len(self.lights),
            end_frame=end_frame, read_overlapped_tick=end_frame != snapshot.frame))
        self.last_frame = snapshot.frame
        self.frames += 1
        self.vehicle_rows += count
        if self.collision_recorder:
            self.collision_recorder.observe(snapshot,[row['vehicle_id'] for row in vehicle_rows])
        contexts = []
        try:
            from .observations import observation_packet
        except ImportError:
            from observations import observation_packet
        self.last_observation = observation_packet(snapshot.frame, base['sim_time'], vehicle_rows, states, contexts,
            run_id=self.run_id, junction_id=self.junction_id, frame_gap=gap,
            read_overlapped_tick=end_frame != snapshot.frame)
        self.observations.write(self.last_observation)
        # 每帧 flush，避免只在 Ctrl+C 时写出；这是用户态缓冲刷新，不等于磁盘 fsync。
        self.flush()
        if gap:
            print(f"WARNING: missed {gap} frames before frame {snapshot.frame}")
        # Return the same-frame truth to adapters; callers must not treat it as sensor observation.
        try:
            from .world_snapshot import collect_world
        except ImportError:
            from world_snapshot import collect_world
        world_packet = collect_world(self.world, snapshot)
        self.last_packet = dict(world_snapshot=world_packet, run_id=self.run_id, frame=snapshot.frame, sim_time=base["sim_time"],
                    junction_id=self.junction_id, vehicles=vehicle_rows, signals=states,
                    frame_gap=gap, read_overlapped_tick=end_frame != snapshot.frame,phase=phase,vehicle_context=contexts)
        return self.last_packet

    def flush(self):
        for handle in self.handles:
            handle.flush()

    def sample_observation(self, snapshot, planner_targets=None):
        """Agent-facing current observation only; call instead of sample, once per tick."""
        self.sample(snapshot, planner_targets=planner_targets)
        return self.last_observation

    def close(self, status="completed"):
        """正常结束、Ctrl+C 和异常退出都保留已经写出的帧。"""
        if self.closed:
            return
        self.closed = True
        if self.collision_recorder:
            self.collision_recorder.close()
        for handle in self.handles:
            handle.close()
        self.observations.close()
        self.metadata.update(status=status, frames=self.frames, vehicle_rows=self.vehicle_rows,
                             missing_frames=self.missing_frames, last_frame=self.last_frame)
        self._write_metadata()
