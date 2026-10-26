"""有限长度检测线的有方向过线检测：前后帧 actor 原点线段与检测线求交。

坐标采用 CARLA 世界坐标，normal 指向允许计数的行驶方向。
只处理连续帧，出生在线后方不补造到达事件；缺帧/消失不跨区间连接。
"""

import math


class DetectionLines:
    def __init__(self, lines):
        self.lines = lines
        ids = [line["id"] for line in lines]
        if len(set(ids)) != len(ids):
            raise ValueError("Detection line IDs must be unique")
        for line in lines:
            if line["half_width_m"] <= 0 or math.hypot(*line["normal_xy"]) == 0:
                raise ValueError("Invalid detection line width/normal")
        self.previous = {}
        self.frame = None

    def update(self, frame, now, rows):
        frame = int(frame)
        events = []
        previous = self.previous if self.frame is not None and frame == self.frame + 1 else {}
        current = {str(r["vehicle_id"]): r for r in rows}
        for vid, row in current.items():
            old = previous.get(vid)
            if old is None:
                continue
            for line in self.lines:
                # 未经校验的候选线不会用于正式事件输出。
                if not line.get("enabled", False):
                    continue
                # 几何求交之外校验车道，避免相邻车道边界或对向轨迹误计。
                if "road_id" in line:
                    keys=("road_id", "section_id", "lane_id")
                    if not any(all(str(point.get(k,""))==str(line[k]) for k in keys)
                               for point in (old,row)):
                        continue
                nx, ny = line["normal_xy"]
                length = math.hypot(nx, ny)
                nx, ny = nx / length, ny / length
                cx, cy = line["center_xy"]
                x0, y0 = float(old["x"])-cx, float(old["y"])-cy
                x1, y1 = float(row["x"])-cx, float(row["y"])-cy
                d0, d1 = x0*nx+y0*ny, x1*nx+y1*ny
                if not d0 < 0 <= d1:
                    continue
                fraction = -d0 / (d1-d0)
                lateral = -(x0+fraction*(x1-x0))*ny + (y0+fraction*(y1-y0))*nx
                if abs(lateral) <= line["half_width_m"]:
                    events.append(dict(event="detection_line_crossing", frame=frame,
                        sim_time=float(now), vehicle_id=vid, line_id=line["id"],
                        kind=line["kind"], junction_id=line["junction_id"],
                        reference_point="actor_origin", estimated_crossing_time=
                        float(old["sim_time"])+fraction*(float(now)-float(old["sim_time"]))))
        self.previous, self.frame = current, frame
        return events
