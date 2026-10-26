"""当前进口车道的停车统计，不将停车数冒充为带空间连续性的队列长度。"""

import math
from collections import deque


class FlowMetrics:
    """按检测线统计累计与滚动窗口次数；同车绕行再次正向过线再次计数。"""
    def __init__(self, lines, window_seconds=60.0):
        if window_seconds <= 0:
            raise ValueError("Flow window must be positive")
        self.window = window_seconds
        self.lines = {line["id"]: line for line in lines if line.get("enabled")}
        self.times = {key: deque() for key in self.lines}
        self.totals = {key: 0 for key in self.lines}
        self.unique = {key: set() for key in self.lines}

    def update(self, now, events):
        now = float(now)
        for event in events:
            key = event["line_id"]
            self.times[key].append(event["estimated_crossing_time"])
            self.totals[key] += 1
            self.unique[key].add(str(event["vehicle_id"]))
        rows = []
        for key, line in self.lines.items():
            # 窗口 (t-window,t]，包括零流量车道。未观察到的过线不补计。
            times = self.times[key]
            # 同帧车辆遍历次序未必按插值时刻排列，排序后再从队首过期。
            if events:
                times = self.times[key] = deque(sorted(times))
            while times and times[0] <= now-self.window:
                times.popleft()
            rows.append(dict(line_id=key, kind=line["kind"], junction_id=line["junction_id"],
                cumulative_count=self.totals[key], unique_vehicle_count=len(self.unique[key]),
                window_seconds=self.window, window_count=len(times),
                nominal_rate_veh_per_hour=len(times)*3600/self.window))
        return rows


def lane_metrics(rows, tracker):
    groups = {}
    for row in rows:
        state = tracker.active[str(row["vehicle_id"])]
        if row.get("signal_match") != "unique" or state["entered"] or state["passed"]:
            continue
        key = tuple(row.get(k, "") for k in ("road_id", "section_id", "lane_id", "signal_opendrive_id"))
        groups.setdefault(key, []).append((row, state))
    result = []
    for key, members in groups.items():
        result.append(dict(zip(("road_id", "section_id", "lane_id", "signal_opendrive_id"), key),
            vehicle_count=len(members), stopped_vehicle_count=sum(s["waiting"] for _, s in members),
            mean_speed_mps=sum(math.hypot(float(r["vx"]), float(r["vy"])) for r, _ in members) / len(members),
            mean_accumulated_wait_s=sum(s["wait_s"] for _, s in members) / len(members)))
    return result
