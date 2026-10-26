"""与 CARLA 解耦的事件状态机；在线采集和 CSV 回放共用同一套规则。

等待仅统计 signal_match=unique 的目标进口道，且在进入目标路口之前。
使用水平速度和双阈值滞回；不是队列识别，也不判断红灯违规。
所有时间为仿真时间。跨缺帧、消失、实验结束均截断，不补算未知区间。
"""

import math


class VehicleTracker:
    def __init__(self, junction_id=1427, stop_speed_mps=0.1, resume_speed_mps=0.3,
                 observation_warmup_seconds=1.0):
        if not 0 <= stop_speed_mps < resume_speed_mps or observation_warmup_seconds < 0:
            raise ValueError("Invalid waiting thresholds")
        self.junction_id = str(junction_id)
        self.stop_speed = stop_speed_mps
        self.resume_speed = resume_speed_mps
        self.warmup = observation_warmup_seconds
        self.active = {}
        self.completed = []
        self.events = []
        self.frame = None
        self.now = None
        self.sequence = 0

    def emit(self, state, kind, **extra):
        self.events.append(dict(frame=self.frame, sim_time=self.now,
            vehicle_id=state["vehicle_id"], visit_id=state["visit_id"], event=kind, **extra))

    def end_wait(self, state, reason):
        if state["waiting"]:
            self.emit(state, "wait_end", reason=reason,
                      observed_end_time=state["last_time"], accumulated_wait_s=state["wait_s"])
        state["waiting"] = False

    def finish(self, vid, reason):
        state = self.active.pop(vid)
        self.end_wait(state, reason)
        self.emit(state, "visit_end", reason=reason)
        self.completed.append(dict(vehicle_id=vid, visit_id=state["visit_id"],
            first_observed_time=state["first_time"], last_observed_time=state["last_time"],
            wait_s=state["wait_s"], stop_count=state["stop_count"],
            junction_enter_time=state["enter_time"], junction_exit_time=state["exit_time"],
            entered_junction=state["entered"], completed=state["passed"], reason=reason))

    def update(self, frame, now, rows):
        """rows 必须是该帧完整的采集范围内车辆列表，空场景也要调用。"""
        frame, now = int(frame), float(now)
        if self.frame is not None and (frame <= self.frame or now <= self.now):
            raise ValueError("Frames and simulation time must increase")
        gap = self.frame is not None and frame != self.frame + 1
        self.frame, self.now = frame, now
        self.events = []
        present = {str(r["vehicle_id"]) for r in rows}
        if len(present) != len(rows):
            raise ValueError("Duplicate vehicle in frame")
        for vid in list(self.active):
            if gap or vid not in present:
                self.finish(vid, "frame_gap" if gap else "observation_lost")
        for row in rows:
            vid = str(row["vehicle_id"])
            approach = row.get("signal_match") == "unique" and str(row.get("current_junction_id", "")) != self.junction_id
            inside = str(row.get("current_junction_id", "")) == self.junction_id
            state = self.active.get(vid)
            observation_start = state["observation_start"] if state else now
            observation_wait = state["observation_wait_s"] if state else 0.0
            # 已经完成通行的车仍在区域内停车时，不再统计其等待。
            # 若以后重新回到目标进口道，建立新的 visit，而不是复用上一趟状态。
            if state and state["passed"] and approach:
                self.finish(vid, "reapproach")
                state = None
            if state is None:
                self.sequence += 1
                state = dict(vehicle_id=vid, visit_id=self.sequence, first_time=now,
                    observation_start=observation_start, observation_wait_s=observation_wait,
                    last_time=now, last_row=None, waiting=False, wait_s=0.0,
                    stop_count=0, entered=False, passed=False, approach_seen=False,
                    enter_time=None, exit_time=None)
                self.active[vid] = state
                self.emit(state, "vehicle_observed")
            previous = state["last_row"]
            eligible = approach and not state["entered"] and not state["passed"]
            if eligible and not state["approach_seen"]:
                state["approach_seen"] = True
                self.emit(state, "approach_observed", signal_opendrive_id=row.get("signal_opendrive_id"))
            speed = math.hypot(float(row["vx"]), float(row["vy"]))
            ready = now - state["observation_start"] >= self.warmup
            # 区间两端均满足等待状态才累计，离开进口道或恢复运动的边界区间不计。
            # 因而误差至多约一个采样间隔/边界；不会将离场后的静止累计进来。
            if state["waiting"]:
                if eligible and ready and speed < self.resume_speed:
                    state["wait_s"] += now - state["last_time"]
                    state["observation_wait_s"] += now - state["last_time"]
                else:
                    self.end_wait(state, "moving" if eligible else "left_approach")
            elif eligible and ready and speed <= self.stop_speed:
                state["waiting"] = True
                state["stop_count"] += 1
                self.emit(state, "wait_start")
            # 仅同车道、同信号、连续观测之间报告参考平面过线。
            # 此事件用于校验几何，绝不命名为 red_light_violation。
            if previous and eligible:
                same = all(str(previous.get(k, "")) == str(row.get(k, "")) for k in
                           ("road_id", "section_id", "lane_id", "signal_opendrive_id"))
                before, after = previous.get("stop_plane_front_distance_m"), row.get("stop_plane_front_distance_m")
                if same and before not in (None, "") and after not in (None, "") and float(before) > 0 >= float(after):
                    self.emit(state, "reference_plane_crossing", signal_state=row.get("signal_state"),
                              previous_signal_state=previous.get("signal_state"), is_violation=None)
            if inside and not state["entered"]:
                state["entered"] = True
                state["enter_time"] = now
                self.emit(state, "junction_enter", left_censored=not state["approach_seen"])
            # 本阶段出口定义：actor 原点从目标 junction 车道转入普通道路车道。
            # 不是车尾完全驶离，也不是预先标定的出口检测线；缺失车道不算完成。
            if previous and str(previous.get("current_junction_id", "")) == self.junction_id and not inside and row.get("road_id") not in (None, "") and str(row.get("is_junction")).lower() == "false":
                state["passed"] = True
                state["exit_time"] = now
                self.emit(state, "junction_exit", completion_definition="actor_origin_leaves_junction")
            state.update(last_time=now, last_row=dict(row))
        return list(self.events)

    def close(self):
        self.events = []
        for vid in list(self.active):
            self.finish(vid, "experiment_end")
        return list(self.events)
