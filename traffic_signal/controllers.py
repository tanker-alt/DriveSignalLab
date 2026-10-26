"""Replaceable traffic-signal policies with an explicit safety transition layer.

The controller selects a *target green movement*.  Yellow and all-red are
inserted here, so neither a rule policy nor a learned policy can command two
conflicting greens directly.
"""
import json
from pathlib import Path


class FixedTimePolicy:
    """Two-movement fixed-time baseline using the same decision contract."""

    name = 'fixed'

    def __init__(self, green_seconds=10.0, yellow_seconds=3.0,
                 clearance_seconds=2.0, initial_red_seconds=2.0):
        self.initial_red_seconds = float(initial_red_seconds)
        self.stages = [
            ('movement_0_green', float(green_seconds), 0, 'Green'),
            ('movement_0_yellow', float(yellow_seconds), 0, 'Yellow'),
            ('all_red_after_0', float(clearance_seconds), None, 'Red'),
            ('movement_1_green', float(green_seconds), 1, 'Green'),
            ('movement_1_yellow', float(yellow_seconds), 1, 'Yellow'),
            ('all_red_after_1', float(clearance_seconds), None, 'Red'),
        ]
        if self.initial_red_seconds < 0 or any(stage[1] <= 0 for stage in self.stages):
            raise ValueError('Signal durations must be positive')

    def update(self, elapsed, observation):
        return False

    def decide(self, elapsed):
        elapsed = max(0.0, float(elapsed))
        if elapsed < self.initial_red_seconds:
            return self._decision('initial_all_red', None, 'Red',
                                  self.initial_red_seconds - elapsed)
        cycle = sum(stage[1] for stage in self.stages)
        offset = (elapsed - self.initial_red_seconds) % cycle
        for phase, duration, movement, state in self.stages:
            if offset < duration:
                return self._decision(phase, movement, state, duration - offset)
            offset -= duration
        raise RuntimeError('Fixed-time phase calculation failed')

    def _decision(self, phase, movement, state, remaining):
        return {'phase': phase, 'active_movement': movement, 'state': state,
                'remaining_s': remaining, 'controller': self.name,
                'target_action': movement, 'forced_by_max_green': False}


class MaxPressureSelector:
    name = 'max_pressure'

    def select(self, observation, current_action=None):
        pressures = observation['pressure']
        if not pressures:
            raise ValueError('MaxPressure requires at least one movement')
        best = max(pressures)
        candidates = [i for i, value in enumerate(pressures) if value == best]
        # Keeping the current phase on a tie avoids needless switching.
        return current_action if current_action in candidates else candidates[0]


class PressLightSelector:
    """Inference adapter for a two-layer PressLight-compatible Q network.

    Checkpoints are NPZ files accompanied by JSON metadata.  This intentionally
    refuses missing or mismatched weights: an untrained/random network must not
    be reported as PressLight.
    """
    name = 'presslight'
    schema = 'carla_two_movement_presslight_v1'

    def __init__(self, checkpoint):
        self.path = Path(checkpoint)
        metadata_path = self.path.with_suffix('.json')
        if not self.path.is_file() or not metadata_path.is_file():
            raise FileNotFoundError('PressLight requires both %s and %s' %
                                    (self.path, metadata_path))
        import numpy as np
        self.np = np
        self.metadata = json.loads(metadata_path.read_text())
        self.movements = int(self.metadata.get('action_dim', 0))
        if self.movements < 2:
            raise ValueError('At least two signal movements required')
        n = self.movements
        features = ['phase_%d' % i for i in range(n)]
        features += [key + '_%d' % i for i in range(n) for key in ('incoming', 'outgoing')]
        expected = {'schema': (self.schema if n == 2 else 'carla_n_movement_presslight_v1'),
                    'state_dim': 3*n, 'action_dim': n, 'features': features}
        for key, value in expected.items():
            if self.metadata.get(key) != value:
                raise ValueError('PressLight checkpoint %s mismatch: expected %r, got %r' %
                                 (key, value, self.metadata.get(key)))
        arrays = np.load(str(self.path), allow_pickle=False)
        required = {'dense_0_kernel': (3*self.movements, 20), 'dense_0_bias': (20,),
                    'dense_1_kernel': (20, 20), 'dense_1_bias': (20,),
                    'q_values_kernel': (20, self.movements), 'q_values_bias': (self.movements,)}
        self.weights = {}
        for name, shape in required.items():
            if name not in arrays or arrays[name].shape != shape:
                raise ValueError('PressLight weight %s must have shape %r' % (name, shape))
            self.weights[name] = arrays[name].astype('float32')

    @staticmethod
    def state_vector(observation, current_action):
        n = len(observation['incoming'])
        if len(observation['outgoing']) != n:
            raise ValueError('Incoming/outgoing movement mismatch')
        phase = [0.0] * n
        if current_action is not None:
            if not 0 <= current_action < n:
                raise ValueError('Current phase outside movement range')
            phase[current_action] = 1.0
        return phase + [float(values[i]) for i in range(n)
                        for values in (observation['incoming'], observation['outgoing'])]

    def select(self, observation, current_action=None):
        if len(observation['incoming']) != self.movements:
            raise ValueError('Checkpoint does not match junction movement count')
        x = self.np.asarray(self.state_vector(observation, current_action), dtype='float32')
        w = self.weights
        x = self.np.maximum(0, x @ w['dense_0_kernel'] + w['dense_0_bias'])
        x = self.np.maximum(0, x @ w['dense_1_kernel'] + w['dense_1_bias'])
        q_values = x @ w['q_values_kernel'] + w['q_values_bias']
        self.last_q_values = q_values.tolist()
        return int(q_values.argmax())


class SafeAdaptivePolicy:
    """Turn target-phase choices into mutually exclusive CARLA light states."""
    def __init__(self, selector, movements=2, initial_red_seconds=2.0,
                 min_green_seconds=5.0, decision_interval_seconds=2.0,
                 yellow_seconds=3.0, clearance_seconds=2.0,
                 max_green_seconds=15.0):
        self.selector = selector
        self.movements = int(movements)
        self.initial_red_seconds = float(initial_red_seconds)
        self.min_green_seconds = float(min_green_seconds)
        self.decision_interval_seconds = float(decision_interval_seconds)
        self.yellow_seconds = float(yellow_seconds)
        self.clearance_seconds = float(clearance_seconds)
        self.max_green_seconds = float(max_green_seconds)
        values = (self.initial_red_seconds, self.min_green_seconds,
                  self.decision_interval_seconds, self.yellow_seconds,
                  self.clearance_seconds, self.max_green_seconds)
        if self.movements < 2 or any(value <= 0 for value in values):
            raise ValueError('Adaptive policy requires at least two movements and positive durations')
        if self.max_green_seconds < self.min_green_seconds:
            raise ValueError('Maximum green must be at least minimum green')
        timing = getattr(selector, 'metadata', {}).get('signal_timing_seconds')
        if timing:
            actual = {'decision_interval': self.decision_interval_seconds,
                      'yellow': self.yellow_seconds,
                      'all_red_clearance': self.clearance_seconds,
                      'minimum_green_after_switch': self.min_green_seconds,
                      'maximum_green': self.max_green_seconds}
            mismatches = {key: (timing.get(key), value) for key, value in actual.items()
                          if float(timing.get(key, -1)) != value}
            if mismatches:
                raise ValueError('PressLight checkpoint/safety-layer timing mismatch: %r' %
                                 mismatches)
        if getattr(selector, 'movements', self.movements) != self.movements:
            raise ValueError('Checkpoint/action-space mismatch')
        self.mode = 'initial_all_red'
        self.mode_start = 0.0
        self.active = None
        self.pending = None
        self.last_decision = None
        self.last_observation = None
        self.last_forced_switch = False

    def update(self, elapsed, observation):
        elapsed = max(0.0, float(elapsed))
        self.last_observation = observation
        changed = False
        while True:
            age = elapsed - self.mode_start
            if self.mode == 'initial_all_red':
                if age < self.initial_red_seconds:
                    break
                self.active = self.selector.select(observation, None)
                self._validate_action(self.active)
                self.mode, self.mode_start, self.last_decision = 'green', elapsed, elapsed
                changed = True
            elif self.mode == 'green':
                others = [(self.active + i) % self.movements for i in range(1, self.movements)]
                waiting = [i for i in others if observation['incoming'][i] > 0]
                forced = age >= self.max_green_seconds and bool(waiting)
                due = forced or (age >= self.min_green_seconds and
                                 elapsed - self.last_decision >= self.decision_interval_seconds)
                if not due:
                    break
                target = waiting[0] if forced else self.selector.select(observation, self.active)
                self._validate_action(target)
                self.last_decision = elapsed
                self.last_forced_switch = forced
                if target == self.active:
                    break
                self.pending = target
                self.mode, self.mode_start = 'yellow', elapsed
                changed = True
            elif self.mode == 'yellow':
                if age < self.yellow_seconds:
                    break
                self.mode, self.mode_start = 'all_red_clearance', elapsed
                changed = True
            elif self.mode == 'all_red_clearance':
                if age < self.clearance_seconds:
                    break
                self.active, self.pending = self.pending, None
                self.mode, self.mode_start, self.last_decision = 'green', elapsed, elapsed
                changed = True
            break
        return changed

    def _validate_action(self, action):
        if not isinstance(action, int) or not 0 <= action < self.movements:
            raise ValueError('Illegal target phase %r' % (action,))

    def decide(self, elapsed):
        elapsed = max(0.0, float(elapsed))
        age = elapsed - self.mode_start
        if self.mode == 'initial_all_red':
            remaining = max(0.0, self.initial_red_seconds-age)
            return self._decision('initial_all_red', None, 'Red', remaining)
        if self.mode == 'green':
            # Time to the next eligible policy decision, not guaranteed phase end.
            remaining = max(0.0, max(self.min_green_seconds-age,
                                     self.decision_interval_seconds-(elapsed-self.last_decision)))
            return self._decision('movement_%d_green' % self.active, self.active,
                                  'Green', remaining)
        if self.mode == 'yellow':
            return self._decision('movement_%d_yellow' % self.active, self.active,
                                  'Yellow', max(0.0, self.yellow_seconds-age))
        return self._decision('all_red_clearance', None, 'Red',
                              max(0.0, self.clearance_seconds-age))

    def _decision(self, phase, active, state, remaining):
        result = {'phase': phase, 'active_movement': active, 'active_ego': active, 'state': state,
                  'remaining_s': remaining, 'controller': self.selector.name,
                  'target_action': self.pending if self.pending is not None else self.active,
                  'forced_by_max_green': self.last_forced_switch}
        if hasattr(self.selector, 'last_q_values'):
            result['q_values'] = self.selector.last_q_values
        return result


def build_signal_policy(name, checkpoint=None, **settings):
    if name == 'fixed':
        return FixedTimePolicy(**settings)
    if name == 'maxpressure':
        return SafeAdaptivePolicy(MaxPressureSelector(), **settings)
    if name == 'presslight':
        if not checkpoint:
            raise ValueError('PRESSLIGHT_CHECKPOINT is required for presslight mode')
        return SafeAdaptivePolicy(PressLightSelector(checkpoint), **settings)
    raise ValueError('Unknown adaptive signal controller %r' % name)
