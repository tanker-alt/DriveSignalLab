"""Custom retrospective Safety/Comfort/Efficiency outputs; observations live separately.

No CARLA dependency. Unknown metrics are null, never zero or a passing score.
"""
import json
import math
from pathlib import Path



def number(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


PENDING = {
    'Safety': {
        'speeding': 'Per-frame speed limit and tolerance/duration rule required'},
    'Comfort': {
        'jerk': 'Incomplete: raw body-component differences can be calculated, but filtering is unresolved; diagnostic only',
        'harsh_braking': 'Acceleration threshold, duration and event merge rules pending confirmation',
        'harsh_acceleration': 'Acceleration threshold, duration and event merge rules pending confirmation',
        'threshold_exceedance_duration': 'Risk/violation thresholds and duration rules pending confirmation'},
    'Efficiency': {
        'task_completion': 'Task start, goal, eligible population and timeout rules pending confirmation',
        'delay_seconds': 'Ideal/free-flow travel-time baseline pending confirmation'}}


def pending(category):
    return {key: dict(value=None, status='pending_definition_or_data', reason=reason)
            for key, reason in PENDING[category].items()}


class AssessmentWriter:
    """Derived records only. Gaps break derivatives and queue integration."""
    def __init__(self, output, safety_config=None, safety_geometry=None, comfort_vehicle_ids=None, comfort_config=None, efficiency_config=None):
        self.mdrive_result = dict(available=False, reason='MDrive trajectory evaluation not attached')
        self.root = Path(output)
        self.folder = self.root / 'assessment'
        self.folder.mkdir(parents=True, exist_ok=True)
        try:
            from .task_efficiency import TaskEfficiency
        except ImportError:
            from task_efficiency import TaskEfficiency
        self.task_efficiency = TaskEfficiency(efficiency_config)
        (self.root / 'efficiency_config_used.json').write_text(
            json.dumps(self.task_efficiency.config, indent=2), encoding='utf-8')
        try:
            from .safety_config import load_config
            from .safety import SafetyEvaluator
        except ImportError:
            from safety_config import load_config
            from safety import SafetyEvaluator
        try:
            from .comfort import AccelerationMetrics
            from .comfort_events import ComfortEvents
        except ImportError:
            from comfort import AccelerationMetrics
            from comfort_events import ComfortEvents
        comfort_vehicle_ids = list(comfort_vehicle_ids) if comfort_vehicle_ids is not None else None
        self.acceleration = AccelerationMetrics(comfort_vehicle_ids)
        self.comfort_events = ComfortEvents(comfort_vehicle_ids, comfort_config)
        (self.root / 'comfort_config_used.json').write_text(
            json.dumps(self.comfort_events.config, indent=2), encoding='utf-8')
        self.safety_config = load_config(safety_config)
        self.safety = SafetyEvaluator(self.safety_config, safety_geometry)
        self.file = (self.folder / 'evaluation_metrics.jsonl').open('w', encoding='utf-8')
        self.safety_events = (self.folder / 'safety_events.jsonl').open('w', encoding='utf-8')
        self.safety_event_offset = 0
        (self.root / 'safety_config_used.json').write_text(json.dumps(self.safety_config, indent=2), encoding='utf-8')
        if safety_geometry is not None:
            (self.root / 'safety_geometry_used.json').write_text(json.dumps(safety_geometry), encoding='utf-8')
        self.previous = {}
        self.previous_queues = {}
        self.queue_integrals = {}
        self.statistics = {}
        self.last_frame = self.last_time = self.start_time = None
        self.frames = self.gaps = 0
        self.last_efficiency = {}
        self.closed = False

    def flush_safety_events(self):
        for event in self.safety.events[self.safety_event_offset:]:
            self.safety_events.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + '\n')
        self.safety_event_offset = len(self.safety.events)
        self.safety_events.flush()

    def sample(self, frame, now, rows, approaches, flows, trips, signals=None, task_starts=()):
        frame, now = int(frame), float(now)
        if self.last_frame is not None and (frame <= self.last_frame or now <= self.last_time):
            raise ValueError('Assessment frame/time must increase')
        contiguous = self.last_frame is not None and frame == self.last_frame + 1
        if self.last_frame is not None:
            self.gaps += frame - self.last_frame - 1
        if self.start_time is None:
            self.start_time = now
        comfort, current = [], {}
        for row in rows:
            vid = str(row['vehicle_id'])
            values = {axis + '_accel_mps2': number(row.get(axis + '_accel_mps2'))
                      for axis in ('longitudinal', 'lateral')}
            old = self.previous.get(vid)
            for axis in ('longitudinal', 'lateral'):
                key = axis + '_accel_mps2'
                values[axis + '_jerk_mps3'] = ((values[key] - old[key]) / (now - self.last_time)
                    if contiguous and old and values[key] is not None and old[key] is not None else None)
            current[vid] = values
            comfort.append(dict(vehicle_id=vid, **values))
            for key, value in values.items():
                if value is None:
                    continue
                stats = self.statistics.setdefault(key, dict(count=0, sum=0., sum_sq=0., peak_abs=0.))
                stats['count'] += 1
                stats['sum'] += value
                stats['sum_sq'] += value * value
                stats['peak_abs'] = max(stats['peak_abs'], abs(value))
        queues = {}
        for approach in approaches:
            key = approach['approach_id']
            value = number(approach.get('queue_length_m')) if approach.get('metric_status') == 'valid' else None
            if value is not None:
                queues[key] = value
                integral = self.queue_integrals.setdefault(key, dict(metre_seconds=0., covered_seconds=0.))
                if contiguous and key in self.previous_queues:
                    dt = now - self.last_time
                    integral['metre_seconds'] += .5 * (self.previous_queues[key] + value) * dt
                    integral['covered_seconds'] += dt
        exits = [f for f in flows if f['kind'] == 'exit']
        efficiency = dict(trip_metrics=dict(trips), queue_length_m_by_approach=queues,
                          exit_crossing_count=sum(f['cumulative_count'] for f in exits) if exits else None,
                          exit_window_count=sum(f['window_count'] for f in exits) if exits else None)
        safety = pending('Safety')
        safety.update(self.safety.sample(frame, now, rows, signals))
        safety.update(self.safety.summary())
        safety['collision'] = dict(value=None, status='pending_final_audit',
            reason='Asynchronous collision callbacks are aggregated at close; see final report')
        self.flush_safety_events()
        vla_acceleration = self.acceleration.sample(frame, now, rows)
        comfort_metrics = pending('Comfort')
        comfort_metrics.update(self.comfort_events.sample(frame, now, rows))
        task_metrics = self.task_efficiency.sample(frame, now, rows, task_starts)
        self.file.write(json.dumps(dict(schema_version='EvaluationMetrics.1', frame=frame, sim_time=now,
            MDrive=self.mdrive_result,
            Safety=safety,
            Comfort=dict(vehicles=comfort, legacy_scope='all_observed_vehicles_diagnostic_only',
                vla_acceleration=vla_acceleration, filter='none', **comfort_metrics),
            Efficiency=dict(**efficiency, **task_metrics)), ensure_ascii=False, allow_nan=False) + '\n')
        self.file.flush()
        self.previous, self.previous_queues = current, queues
        self.last_frame, self.last_time = frame, now
        self.last_efficiency = efficiency
        self.frames += 1

    def close(self, status, trip_metrics, collision_summary=None):
        if self.closed:
            return
        self.closed = True
        self.file.close()
        comfort = {key: dict(sample_count=s['count'], mean=s['sum']/s['count'],
                   rms=math.sqrt(s['sum_sq']/s['count']), peak_abs=s['peak_abs'])
                   for key, s in self.statistics.items()}
        queues = {key: dict(mean_queue_length_m=(s['metre_seconds']/s['covered_seconds'] if s['covered_seconds'] else None),
                           **s) for key, s in self.queue_integrals.items()}
        coverage = sum(s['covered_seconds'] for s in self.queue_integrals.values())
        safety = pending('Safety')
        safety.update(self.safety.close())
        self.flush_safety_events()
        self.safety_events.close()
        safety['collision'] = (dict(status=collision_summary.get('accidents', {}).get('status', 'unavailable'),
                                   value=collision_summary.get('accidents', {}).get('accident_count'), unit='observed_accidents',
                                   accident_audit=collision_summary.get('accidents'),
                                   raw_callback_count=collision_summary['raw_callback_count'],
                                   pair_categories=collision_summary.get('categories', {}),
                                   coverage=collision_summary['coverage'],
                                   definition=collision_summary.get('accidents', {}).get('definition'), audit=collision_summary)
                               if collision_summary else dict(status='unavailable', value=None,
                                   reason='No aggregated collision audit for this evaluation scope'))
        comfort_metrics = pending('Comfort')
        comfort_metrics.update(self.comfort_events.close())
        (self.folder / 'comfort_events.jsonl').write_text(''.join(
            json.dumps(event, allow_nan=False) + '\n' for event in self.comfort_events.events), encoding='utf-8')
        report = dict(schema_version='evaluation_report.1', status=status,
            frames=self.frames, missing_frames=self.gaps, start_sim_time=self.start_time,
            end_sim_time=self.last_time,
            MDrive=self.mdrive_result,
            Safety=safety,
            Comfort=dict(statistics=comfort, aggregation='vehicle-sample weighted; no smoothing',
                legacy_scope='all_observed_vehicles_diagnostic_only_not_vla_comfort',
                acceleration=self.acceleration.summary(),
                jerk_definition='backward difference of body-yaw projected acceleration / sim dt; consecutive observed frames only',
                **comfort_metrics),
            Efficiency=dict(**self.task_efficiency.close(), trip_metrics=trip_metrics,
                traffic_metrics=getattr(self,'traffic_efficiency_result',None),
                mean_travel_time_s=trip_metrics.get('mean_paired_travel_time_s'),
                travel_time_definition='arrival-to-exit actor-origin crossings, valid paired trips only; not full task time',
                queue_by_approach=queues,
                mean_queue_length_m=(sum(s['metre_seconds'] for s in self.queue_integrals.values())/coverage if coverage else None),
                queue_definition='trapezoidal time-weighted lane mean; valid consecutive frames only, includes observed empty queues',
                queue_covered_lane_seconds=coverage,
                throughput_crossings=self.last_efficiency.get('exit_crossing_count'),
                throughput_definition='observed exit crossings, repeated passes counted; not demand-normalized capacity'),
            safety_config=self.safety_config,
            notes=['Pending values are not zero or passing scores.',
                   'All-vehicle Safety uses approved map geometry proxies; MDrive vehicle scores remain separate.',
                   'Source preparation period must be excluded explicitly for formal comparisons.'])
        (self.folder / 'evaluation_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
        return report
