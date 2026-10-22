import json
import tempfile
import unittest
from pathlib import Path

try:
    import numpy as np
except ImportError:
    np = None

from traffic_signal.controllers import (FixedTimePolicy, MaxPressureSelector,
                                        PressLightSelector, SafeAdaptivePolicy)


def observation(incoming=(0, 0), outgoing=(0, 0)):
    return {'incoming': list(incoming), 'outgoing': list(outgoing),
            'pressure': [a-b for a, b in zip(incoming, outgoing)]}


class TrafficSignalControlTests(unittest.TestCase):
    def test_fixed_time_uses_safe_transition_stages(self):
        policy = FixedTimePolicy(initial_red_seconds=2, green_seconds=10,
                                 yellow_seconds=3, clearance_seconds=2)
        self.assertEqual(policy.decide(0)['phase'], 'initial_all_red')
        self.assertEqual(policy.decide(2)['phase'], 'movement_0_green')
        self.assertEqual(policy.decide(12)['phase'], 'movement_0_yellow')
        self.assertEqual(policy.decide(15)['phase'], 'all_red_after_0')
        self.assertEqual(policy.decide(17)['phase'], 'movement_1_green')

    def test_max_pressure_prefers_demand_and_keeps_tie(self):
        selector = MaxPressureSelector()
        self.assertEqual(selector.select(observation((1, 3)), 0), 1)
        self.assertEqual(selector.select(observation((2, 2)), 1), 1)

    def test_safe_transition_inserts_yellow_and_clearance(self):
        policy = SafeAdaptivePolicy(MaxPressureSelector(), initial_red_seconds=2,
                                    min_green_seconds=5, decision_interval_seconds=2,
                                    yellow_seconds=3, clearance_seconds=2)
        policy.update(0, observation((2, 0)))
        self.assertEqual(policy.decide(0)['phase'], 'initial_all_red')
        policy.update(2, observation((2, 0)))
        self.assertEqual(policy.decide(2)['phase'], 'movement_0_green')
        policy.update(7, observation((0, 2)))
        self.assertEqual(policy.decide(7)['phase'], 'movement_0_yellow')
        policy.update(10, observation((0, 2)))
        self.assertEqual(policy.decide(10)['phase'], 'all_red_clearance')
        policy.update(12, observation((0, 2)))
        self.assertEqual(policy.decide(12)['phase'], 'movement_1_green')

    def test_maximum_green_forces_other_movement(self):
        policy = SafeAdaptivePolicy(MaxPressureSelector(), initial_red_seconds=2,
                                    min_green_seconds=5, decision_interval_seconds=2,
                                    yellow_seconds=3, clearance_seconds=2,
                                    max_green_seconds=15)
        policy.update(2, observation((5, 1)))
        self.assertEqual(policy.decide(2)['phase'], 'movement_0_green')
        policy.update(17, observation((5, 1)))
        decision = policy.decide(17)
        self.assertEqual(decision['phase'], 'movement_0_yellow')
        self.assertEqual(decision['target_action'], 1)
        self.assertTrue(decision['forced_by_max_green'])

    @unittest.skipIf(np is None, 'numpy is only installed in the LMDrive runtime')
    def test_presslight_checkpoint_contract_and_inference(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'model.npz'
            arrays = {'dense_0_kernel': np.zeros((6, 20), dtype='float32'),
                      'dense_0_bias': np.ones(20, dtype='float32'),
                      'dense_1_kernel': np.zeros((20, 20), dtype='float32'),
                      'dense_1_bias': np.ones(20, dtype='float32'),
                      'q_values_kernel': np.zeros((20, 2), dtype='float32'),
                      'q_values_bias': np.array([0, 1], dtype='float32')}
            np.savez(path, **arrays)
            path.with_suffix('.json').write_text(json.dumps({
                'schema':'carla_two_movement_presslight_v1','state_dim':6,'action_dim':2,
                'features':['phase_0','phase_1','incoming_0','outgoing_0','incoming_1','outgoing_1']}))
            selector = PressLightSelector(path)
            self.assertEqual(selector.select(observation((1, 2)), 0), 1)
            self.assertEqual(len(selector.last_q_values), 2)

    def test_presslight_refuses_missing_checkpoint(self):
        with self.assertRaises(FileNotFoundError):
            PressLightSelector('/tmp/definitely_missing_presslight.npz')

    @unittest.skipIf(np is None, 'numpy is only installed in the LMDrive runtime')
    def test_presslight_refuses_training_runtime_timing_mismatch(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'model.npz'
            np.savez(path,
                     dense_0_kernel=np.zeros((6, 20), dtype='float32'),
                     dense_0_bias=np.zeros(20, dtype='float32'),
                     dense_1_kernel=np.zeros((20, 20), dtype='float32'),
                     dense_1_bias=np.zeros(20, dtype='float32'),
                     q_values_kernel=np.zeros((20, 2), dtype='float32'),
                     q_values_bias=np.zeros(2, dtype='float32'))
            path.with_suffix('.json').write_text(json.dumps({
                'schema':'carla_two_movement_presslight_v1','state_dim':6,'action_dim':2,
                'features':['phase_0','phase_1','incoming_0','outgoing_0',
                            'incoming_1','outgoing_1'],
                'signal_timing_seconds': {'decision_interval':2, 'yellow':3,
                    'all_red_clearance':2, 'minimum_green_after_switch':5,
                    'maximum_green':15}}))
            with self.assertRaisesRegex(ValueError, 'timing mismatch'):
                SafeAdaptivePolicy(PressLightSelector(path), yellow_seconds=4)


if __name__ == '__main__':
    unittest.main()
