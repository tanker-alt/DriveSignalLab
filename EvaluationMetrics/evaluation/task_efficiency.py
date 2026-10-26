"""Planner task assessment, independent of Safety and network trip metrics."""
import copy
import json
import math
from pathlib import Path
try:
    from .comfort import finite
except ImportError:
    from comfort import finite


class TaskEfficiency:
    def __init__(self, config=None):
        path = Path(config) if config else Path(__file__).parent/'configs/efficiency.json'
        self.config = json.loads(path.read_text(encoding='utf-8'))
        for key in ('stop_speed_mps', 'stop_duration_s'):
            v = self.config[key]
            if isinstance(v, bool) or finite(v) is None or float(v) < 0:
                raise ValueError('Invalid efficiency setting: ' + key)
            self.config[key] = float(v)
        self.tasks = {}
        self.frame = self.time = None

    def activate(self, task, frame, now):
        """Call on the frame where task activation AND VLA control ownership hold."""
        task = copy.deepcopy(task)
        tid = str(task['task_id'])
        vid = str(task['vehicle_id'])
        if tid in self.tasks:
            raise ValueError('Task IDs must be unique; active reference routes are immutable')
        if any(t['vehicle_id'] == vid and t['status'] == 'active' for t in self.tasks.values()):
            raise ValueError('Vehicle already has an active task')
        for point in ('start', 'goal'):
            for axis in ('x', 'y'):
                if finite(task[point].get(axis)) is None:
                    raise ValueError('Invalid task position')
        for key in ('arrival_radius_m', 'timeout_s'):
            if isinstance(task[key], bool) or finite(task[key]) is None or float(task[key]) <= 0:
                raise ValueError('Task requires positive ' + key)
        if not isinstance(task['require_stop'], bool):
            raise ValueError('require_stop must be boolean')
        ideal, reason = None, 'missing_reference_segments'
        segments = task.get('reference_segments')
        if isinstance(segments, list) and segments:
            terms = []
            for segment in segments:
                length = finite(segment.get('length_m')) if isinstance(segment, dict) else None
                speed = finite(segment.get('reference_speed_mps')) if isinstance(segment, dict) else None
                if length is None or speed is None or length < 0 or speed <= 0:
                    reason = 'invalid_reference_segments'
                    break
                terms.append(length/speed)
            else:
                total = sum(terms)
                if math.isfinite(total) and total > 0:
                    ideal, reason = total, None
                else:
                    reason = 'invalid_reference_total'
        self.tasks[tid] = dict(task_id=tid, vehicle_id=vid, definition=task,
            status='active', start_frame=frame, start_sim_time=now, end_sim_time=None,
            elapsed_s=0., actual_duration_s=None, ideal_duration_s=ideal,
            delay_seconds=None, delay_unavailable_reason=reason or 'task_not_completed',
            stop_since=None, reason=None)

    def finish(self, t, status, now, reason):
        t.update(status=status, end_sim_time=now, elapsed_s=now-t['start_sim_time'], reason=reason)
        if status == 'completed':
            t['actual_duration_s'] = t['elapsed_s']
            if t['ideal_duration_s'] is not None:
                t['delay_seconds'] = t['actual_duration_s']-t['ideal_duration_s']
                t['delay_unavailable_reason'] = None

    def sample(self, frame, now, rows, task_starts=()):
        frame, now = int(frame), float(now)
        if not math.isfinite(now) or (self.frame is not None and (frame <= self.frame or now <= self.time)):
            raise ValueError('Task frame/time must increase and be finite')
        gap = self.frame is not None and frame != self.frame+1
        for t in self.tasks.values():
            if t['status'] == 'active' and gap:
                self.finish(t, 'censored', self.time, 'frame_gap')
        for task in task_starts:
            self.activate(task, frame, now)
        actors = {}
        for row in rows:
            vid = str(row['vehicle_id'])
            if vid in actors:
                raise ValueError('Duplicate vehicle observation')
            actors[vid] = row
        for t in self.tasks.values():
            if t['status'] != 'active':
                continue
            definition = t['definition']
            row = actors.get(t['vehicle_id'], {})
            x, y = finite(row.get('x')), finite(row.get('y'))
            speed = finite(row.get('speed_mps'))
            if x is None or y is None or (definition['require_stop'] and (speed is None or speed < 0)):
                self.finish(t, 'censored', max(t['start_sim_time'], self.time if self.time is not None else now), 'missing_observation')
                continue
            deadline = t['start_sim_time']+float(definition['timeout_s'])
            if now > deadline:
                self.finish(t, 'timed_out', deadline, 'deadline_exceeded')
                continue
            t['elapsed_s'] = now-t['start_sim_time']
            goal = definition['goal']
            inside = math.hypot(x-float(goal['x']), y-float(goal['y'])) <= float(definition['arrival_radius_m'])
            arrived = inside
            if definition['require_stop']:
                if inside and speed <= self.config['stop_speed_mps']:
                    if t['stop_since'] is None:
                        t['stop_since'] = now
                    arrived = now-t['stop_since']+1e-9 >= self.config['stop_duration_s']
                else:
                    t['stop_since'] = None
                    arrived = False
            if arrived:
                self.finish(t, 'completed', now, 'goal_verified')
            elif now >= deadline:
                self.finish(t, 'timed_out', deadline, 'deadline_exceeded')
        self.frame, self.time = frame, now
        return self.summary()

    def summary(self):
        tasks = [{k: copy.deepcopy(v) for k, v in t.items() if k != 'stop_since'} for t in self.tasks.values()]
        counts = {s: sum(t['status'] == s for t in tasks) for s in ('completed', 'timed_out', 'censored', 'active')}
        delays = [t['delay_seconds'] for t in tasks if t['delay_seconds'] is not None]
        return dict(task_completion=dict(status='observed' if tasks else 'pending_task_input',
                    value=None, completion_rate_status='denominator_pending_confirmation', counts=counts, tasks=tasks),
                    delay_seconds=dict(status='available' if delays else 'unavailable',
                        value=sum(delays)/len(delays) if delays else None, valid_task_count=len(delays),
                        aggregation='mean of completed tasks with valid reference segments; signed, not clipped'))

    def close(self):
        for t in self.tasks.values():
            if t['status'] == 'active':
                self.finish(t, 'censored', self.time, 'evaluation_ended')
        return self.summary()
