"""Current-frame information packets only; no metric calculation.

No CARLA dependency. Unknown metrics are null, never zero or a passing score.
"""
import json
import math
from pathlib import Path


OBSERVATION_FIELDS = (
    'vehicle_id', 'type_id', 'role_name', 'x', 'y', 'z', 'yaw', 'pitch', 'roll',
    'vx', 'vy', 'vz', 'speed_mps', 'ax', 'ay', 'az',
    'longitudinal_accel_mps2', 'lateral_accel_mps2',
    'road_id', 'section_id', 'lane_id', 'lane_s', 'lane_width_m',
    'is_junction', 'current_junction_id', 'bbox_vertices_world_json',
    'signal_match', 'signal_candidates', 'signal_opendrive_id', 'signal_state',
    'stop_plane_front_distance_m')
CONTEXT_FIELDS = ('vehicle_id', 'geometry_match', 'approach_id', 'origin_reference_distance_m',
    'front_reference_distance_m', 'rear_reference_distance_m', 'reference_kind', 'calibration_id',
    'approach_signal_id', 'stop_line_front_distance_m', 'stop_line_rear_distance_m',
    'signal_association_source', 'approach_signal_state')


def observation_packet(frame, now, rows, signals, contexts=(), **identity):
    """Explicit whitelist: no future schedule, rewards, trip result or metrics."""
    return dict(schema_version='agent_observations.1', source='carla_simulator_truth',
                input_policy='candidate_truth_fields_not_sensor_or_visibility_filtered',
                frame=int(frame), sim_time=float(now),
                **{k: v for k, v in identity.items() if k in ('run_id', 'junction_id', 'frame_gap', 'read_overlapped_tick')},
                vehicles=[{key: row[key] for key in OBSERVATION_FIELDS if key in row} for row in rows],
                signals=dict(signals), vehicle_context=[{k: r[k] for k in CONTEXT_FIELDS if k in r} for r in contexts])


class ObservationWriter:
    def __init__(self, output):
        folder = Path(output) / 'observations'
        folder.mkdir(parents=True, exist_ok=True)
        self.file = (folder / 'agent_observations.jsonl').open('w', encoding='utf-8')

    def write(self, packet):
        self.file.write(json.dumps(packet, ensure_ascii=False, allow_nan=False) + '\n')
        self.file.flush()

    def close(self):
        self.file.close()

