"""Evaluation consumer. Does not tick, control actors or acquire observations."""
import json
from pathlib import Path
from .recorder import EvaluationRecorder, DEFAULT_CONFIG
from .mdrive_metrics import MDriveMetrics


class MetricEvaluator:
    def __init__(self, output, junction_id, detection_config=None, approach_geometry=None,
                 safety_config=None, safety_geometry=None, vehicle_ids=(), road_map=None,
                 routes=None, collision_merge_gap_seconds=1.0, collision_accident_config=None):
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        config = json.loads(DEFAULT_CONFIG.read_text(encoding='utf-8'))
        config['junction_id'] = junction_id
        if safety_geometry is None and road_map is not None:
            from .safety_config import load_config, map_geometry
            safety_geometry = map_geometry(road_map, load_config(safety_config))
        self.recorder = EvaluationRecorder(self.output, config, detection_config,
                                           approach_geometry, safety_config, safety_geometry)
        self.mdrive = MDriveMetrics(vehicle_ids, road_map, routes)
        self.gap = collision_merge_gap_seconds
        self.collision_accident_config = collision_accident_config
        self.junction_id = junction_id
        self.last_frame = self.last_time = None
        self.closed = False
        self.report = None
        # Numeric acceleration statistics and our event/exposure measures are
        # supplementary to MDrive's binary history_comfort, not alternative scores.
        from .comfort import AccelerationMetrics
        from .comfort_events import ComfortEvents
        self.recorder.assessment.acceleration = AccelerationMetrics(vehicle_ids)
        self.recorder.assessment.comfort_events = ComfortEvents(vehicle_ids)
        self.raw = (self.output/'assessment'/'input_frames.jsonl').open('w', encoding='utf-8')

    def sample(self, packet):
        if self.closed:
            raise RuntimeError('Evaluator is closed')
        if packet['junction_id'] != self.junction_id:
            raise ValueError('Packet junction differs from evaluator')
        if packet.get('read_overlapped_tick'):
            raise ValueError('Cannot evaluate mixed-frame observations')
        frame, now = packet['frame'], packet['sim_time']
        if self.last_frame is not None and (frame <= self.last_frame or now <= self.last_time):
            raise ValueError('Evaluation frame/time must increase')
        self.mdrive.sample(packet)
        self.raw.write(json.dumps(packet, ensure_ascii=False, allow_nan=False)+'\n')
        self.raw.flush()
        self.recorder.sample(frame, now, packet['vehicles'], packet.get('signals', {}), packet.get('task_starts', ()))
        self.last_frame, self.last_time = frame, now

    def close(self, status='completed'):
        if self.closed:
            return self.report
        self.closed = True
        self.raw.close()
        try:
            self.recorder.assessment.mdrive_result = self.mdrive.close()
        except Exception as exc:
            # A failed upstream calculation must not erase local audit data.
            self.recorder.assessment.mdrive_result = dict(available=False, value=None,
                                                          reason='MDrive calculation failed: '+str(exc))
            status = 'failed'
        collision_path = self.output/'collisions.jsonl'
        if collision_path.exists():
            from .collision_events import summarize
            summarize(collision_path, self.output, self.gap, self.collision_accident_config)
        self.recorder.close(status)
        self.report = json.loads((self.output/'assessment/evaluation_report.json').read_text(encoding='utf-8'))
        return self.report
