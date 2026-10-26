"""Forward-only, unfiltered acceleration events on explicitly selected VLA actors."""
import json
import math
from pathlib import Path
try:
    from .comfort import finite
    from .comfort_exposure import ComfortExposure
except ImportError:
    from comfort import finite
    from comfort_exposure import ComfortExposure


class ComfortEvents:
    def __init__(self, vehicle_ids=None, config=None):
        path = Path(config) if config is not None else Path(__file__).parent/'configs/comfort.json'
        self.config = json.loads(path.read_text(encoding='utf-8'))
        for name, sign in (('harsh_braking', -1), ('harsh_acceleration', 1)):
            rule = self.config[name]
            for key in ('trigger_mps2', 'release_mps2', 'minimum_duration_s'):
                if isinstance(rule[key], bool) or finite(rule[key]) is None:
                    raise ValueError('Invalid Comfort configuration: ' + key)
                rule[key] = float(rule[key])
            if not (sign*rule['trigger_mps2'] > sign*rule['release_mps2'] >= 0 and rule['minimum_duration_s'] >= 0):
                raise ValueError('Invalid Comfort thresholds or duration')
        self.ids = {str(v) for v in vehicle_ids} if vehicle_ids is not None else set()
        self.exposure = ComfortExposure(self.ids, self.config)
        self.states, self.previous, self.events = {}, set(), []
        self.valid = {name: 0 for name in self.config}
        self.frame = self.time = None

    def finish(self, key, now, reason):
        state = self.states.pop(key)
        if state['confirmed']:
            self.events.append(dict(vehicle_id=key[0], metric=key[1],
                start_sim_time=state['start'], end_sim_time=now,
                status='completed' if reason == 'released' and not state['left_censored'] else 'censored',
                reason='left_censored' if reason == 'released' and state['left_censored'] else reason))

    def sample(self, frame, now, rows):
        if self.frame is not None and (frame <= self.frame or now <= self.time):
            raise ValueError('Comfort event frame/time must increase')
        if self.frame is not None and frame != self.frame + 1:
            for key in list(self.states):
                self.finish(key, self.time, 'frame_gap')
            self.previous.clear()
        present = {}
        for row in rows:
            vid = str(row['vehicle_id'])
            if vid not in self.ids:
                continue
            if vid in present:
                raise ValueError('Duplicate Comfort actor')
            present[vid] = row
        eligible = set()
        exposure_values = {}
        for vid in self.ids:
            row = present.get(vid, {})
            a = finite(row.get('longitudinal_accel_mps2'))
            vx, vy, yaw = (finite(row.get(k)) for k in ('vx', 'vy', 'yaw'))
            speed = (vx*math.cos(math.radians(yaw)) + vy*math.sin(math.radians(yaw))
                     if all(v is not None for v in (vx, vy, yaw)) else None)
            if a is None or speed is None or speed < 0:
                for name in self.config:
                    key = (vid, name)
                    if key in self.states:
                        self.finish(key, self.time, 'reverse_or_missing_data')
                continue
            # At rest only allow an existing event to close; do not initiate an event.
            if speed > 0:
                eligible.add(vid)
                exposure_values[vid] = a
            for name, sign in (('harsh_braking', -1), ('harsh_acceleration', 1)):
                rule, key = self.config[name], (vid, name)
                if speed > 0:
                    self.valid[name] += 1
                state = self.states.get(key)
                if state and sign*a <= sign*rule['release_mps2']:
                    self.finish(key, now, 'released')
                    continue
                if speed == 0:
                    if state:
                        self.finish(key, self.time, 'stopped_before_release')
                    continue
                over = sign*a > sign*rule['trigger_mps2']
                if state and not state['confirmed'] and not over:
                    del self.states[key]
                    state = None
                if state is None and over:
                    state = dict(start=now, confirmed=False, left_censored=vid not in self.previous)
                    self.states[key] = state
                if state and over and now-state['start'] + 1e-9 >= rule['minimum_duration_s']:
                    state['confirmed'] = True
        self.previous = eligible
        self.exposure.sample(frame, now, exposure_values)
        self.frame, self.time = frame, now
        return self.summary()

    def summary(self):
        result = {name: dict(value=sum(e['metric'] == name and e['status'] == 'completed' for e in self.events)
                          if self.valid[name] else None,
                          status='observed_unfiltered' if self.valid[name] else 'unavailable',
                          censored_count=sum(e['metric'] == name and e['status'] == 'censored' for e in self.events),
                          active_count=sum(k[1] == name and s['confirmed'] for k, s in self.states.items()),
                          eligible_sample_count=self.valid[name], scope='explicit_vla_forward_only', filter='none')
                for name in self.config}
        result['threshold_exceedance_duration'] = self.exposure.summary(self.events, self.states)
        return result

    def close(self):
        for key in list(self.states):
            self.finish(key, self.time, 'evaluation_ended')
        return self.summary()
