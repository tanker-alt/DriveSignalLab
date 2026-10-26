"""在线和离线共用的派生输出记录器；不更改原始车辆与信号 CSV。"""

import csv
import json
from pathlib import Path

try:
    from .vehicle_tracker import VehicleTracker
    from .metrics import lane_metrics, FlowMetrics
    from .detection_lines import DetectionLines
    from .approach_metrics import ApproachMetrics
    from .trip_tracker import TripTracker
    from .signal_association import SignalAssociation
except ImportError:  # 支持直接 python evaluation/run_collection.py
    from vehicle_tracker import VehicleTracker
    from metrics import lane_metrics, FlowMetrics
    from detection_lines import DetectionLines
    from approach_metrics import ApproachMetrics
    from trip_tracker import TripTracker
    from signal_association import SignalAssociation


DEFAULT_CONFIG = Path(__file__).parent / "configs" / "metrics.json"


class EvaluationRecorder:
    def __init__(self, output, config=None, line_config=None, approach_geometry=None):
        self.output = Path(output)
        self.config = config or json.loads(DEFAULT_CONFIG.read_text(encoding="utf-8"))
        if "junction_id" not in self.config:
            raise ValueError("Evaluation config requires junction_id")
        self.tracker = VehicleTracker(**self.config)
        self.trips=TripTracker(self.config["junction_id"])
        self.associations=SignalAssociation(self.config["junction_id"])
        self.trip_file=(self.output/"trips.jsonl").open("w",encoding="utf-8")
        self.context_file=(self.output/"vehicle_context.csv").open("w",newline="",encoding="utf-8-sig")
        self.context_writer=csv.DictWriter(self.context_file,fieldnames=["frame","sim_time","junction_id","vehicle_id",
            "geometry_match","approach_id","origin_reference_distance_m","front_reference_distance_m",
            "rear_reference_distance_m","reference_kind","calibration_id","approach_signal_id",
            "signal_association_source","approach_signal_state"])
        self.context_writer.writeheader()
        if line_config is None:
            line_config = {"lines": []}
        self.line_config = line_config
        self.detectors = DetectionLines([line for line in line_config["lines"]
            if int(line["junction_id"]) == self.config["junction_id"]])
        self.flow = FlowMetrics(self.detectors.lines)
        self.approaches = ApproachMetrics(approach_geometry) if approach_geometry else None
        self.approach_file = None
        if approach_geometry:
            (self.output / "approach_geometry_used.json").write_text(json.dumps(approach_geometry,indent=2),encoding="utf-8")
            self.approach_file = (self.output / "approaches.csv").open("w",newline="",encoding="utf-8-sig")
            head_fields=[prefix+suffix for prefix in ("lane_head","queue_head")
                for suffix in ("_id","_distance_m","_speed_mps","_longitudinal_accel_mps2")]
            self.approach_writer=csv.DictWriter(self.approach_file,fieldnames=["frame","sim_time","junction_id",
                "approach_id","signal_candidates","detection_length_m","geometry_status","metric_status",
                "primary_vehicle_count","queue_vehicle_count","queue_tail_id","queue_length_m","spatial_occupancy"]+head_fields)
            self.approach_writer.writeheader()
        self.junction_file=(self.output / "junctions.csv").open("w",newline="",encoding="utf-8-sig")
        self.junction_writer=csv.DictWriter(self.junction_file,fieldnames=["frame","sim_time","junction_id",
            "observed_vehicle_count","inside_vehicle_count","exit_crossing_count","exit_window_count",
            "current_approach_vehicle_count","mean_current_approach_speed_mps","mean_current_approach_wait_s",
            "paired_trip_count","open_trip_count","censored_trip_count","wait_sample_count",
            "wait_excluded_paired_count","mean_paired_approach_wait_s","mean_paired_travel_time_s"])
        self.junction_writer.writeheader()
        # 初始化即保存快照；强制退出也可知道此实验使用的是哪一版检测线。
        (self.output / "detection_lines_used.json").write_text(json.dumps(line_config,indent=2),encoding="utf-8")
        self.flow_file = (self.output / "flows.csv").open("w",newline="",encoding="utf-8-sig")
        self.flow_writer = csv.DictWriter(self.flow_file, fieldnames=["frame","sim_time","line_id","kind",
            "junction_id","cumulative_count","unique_vehicle_count","window_seconds","window_count",
            "nominal_rate_veh_per_hour"])
        self.flow_writer.writeheader()
        self.events = (self.output / "events.jsonl").open("w", encoding="utf-8")
        self.lanes = (self.output / "lanes.csv").open("w", newline="", encoding="utf-8-sig")
        self.writer = csv.DictWriter(self.lanes, fieldnames=["frame", "sim_time", "road_id",
            "section_id", "lane_id", "signal_opendrive_id", "vehicle_count",
            "stopped_vehicle_count", "mean_speed_mps", "mean_accumulated_wait_s"])
        self.writer.writeheader()
        self.closed = False

    def write_events(self, events):
        for event in events:
            self.events.write(json.dumps(event, ensure_ascii=False) + "\n")
        self.events.flush()

    def sample(self, frame, now, rows, signals=None):
        self.write_events(self.tracker.update(frame, now, rows))
        crossings = self.detectors.update(frame, now, rows)
        self.write_events(crossings)
        for trip in self.trips.update(frame,now,rows,crossings,self.tracker):
            self.trip_file.write(json.dumps(trip,ensure_ascii=False)+"\n")
        self.trip_file.flush()
        flows=self.flow.update(now,crossings)
        for flow in flows:
            self.flow_writer.writerow(dict(frame=frame,sim_time=now,**flow))
        self.flow_file.flush()
        current_lanes=lane_metrics(rows, self.tracker)
        for row in current_lanes:
            self.writer.writerow(dict(frame=frame, sim_time=now, **row))
        self.lanes.flush()
        exits=[flow for flow in flows if flow["kind"]=="exit"]
        approach_count=sum(lane["vehicle_count"] for lane in current_lanes)
        trip_metrics=self.trips.summary()
        trip_metrics.pop("definition")
        self.junction_writer.writerow(dict(frame=frame,sim_time=now,junction_id=self.config["junction_id"],
            observed_vehicle_count=len(rows),inside_vehicle_count=sum(str(row.get("current_junction_id",""))==str(self.config["junction_id"]) for row in rows),
            exit_crossing_count=sum(flow["cumulative_count"] for flow in exits) if exits else None,
            exit_window_count=sum(flow["window_count"] for flow in exits) if exits else None,
            current_approach_vehicle_count=approach_count,
            mean_current_approach_speed_mps=sum(lane["vehicle_count"]*lane["mean_speed_mps"] for lane in current_lanes)/approach_count if approach_count else None,
            mean_current_approach_wait_s=sum(lane["vehicle_count"]*lane["mean_accumulated_wait_s"] for lane in current_lanes)/approach_count if approach_count else None,
            **trip_metrics))
        self.junction_file.flush()
        if self.approaches:
            for result in self.approaches.sample(rows):
                self.approach_writer.writerow(dict(frame=frame,sim_time=now,junction_id=self.config["junction_id"],**result))
            self.approach_file.flush()
        links=self.associations.update(frame,rows,signals or {})
        contexts=[]
        for row in rows:
            vid=str(row["vehicle_id"])
            detail=self.approaches.vehicle_details[vid] if self.approaches else dict(geometry_match="unavailable")
            context=dict(frame=frame,sim_time=now,junction_id=self.config["junction_id"],vehicle_id=vid,**detail,**links[vid])
            self.context_writer.writerow(context)
            contexts.append(context)
        self.context_file.flush()
        return contexts

    def close(self, status="completed"):
        if self.closed:
            return
        self.closed = True
        for trip in self.trips.close():
            self.trip_file.write(json.dumps(trip,ensure_ascii=False)+"\n")
        self.trip_file.close()
        self.context_file.close()
        self.write_events(self.tracker.close())
        self.events.close()
        self.lanes.close()
        self.flow_file.close()
        self.junction_file.close()
        if self.approach_file:
            self.approach_file.close()
        visits = self.tracker.completed
        summary = dict(schema_version="1.0", status=status, config=self.config,
            detection_line_config=self.line_config,
            flow_totals=self.flow.totals,
            trip_metrics=self.trips.summary(),
            completion_definition="actor_origin_leaves_junction", visits=visits,
            completed_visits=sum(v["completed"] for v in visits),
            total_observed_approach_wait_s=sum(v["wait_s"] for v in visits),
            notes="Waiting excludes downstream stops. Empty lanes omitted. No red-light violation judgement.")
        (self.output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
