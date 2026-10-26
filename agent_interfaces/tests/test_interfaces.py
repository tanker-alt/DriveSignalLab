import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from agent_interfaces import (load_plugin, load_driving_agent, validate_controls,
                              SignalAgent, SignalSelector, OfficialSignalCallback)
from agent_interfaces.signals import MaxPressureAgent
from traffic_signal.controllers import SafeAdaptivePolicy
from agent_interfaces.presslight import OfficialPressLightAgent


class FakeDrivingAgent:
    ego_vehicles_num = 2
    agent_name = 'test'
    sensor_interface = object()

    def sensors(self):
        return []

    def set_global_plan(self, gps, world):
        pass

    def run_step(self, data, timestamp):
        return []

    def __call__(self):
        return []

    def destroy(self):
        self.closed = True


class RecordingAgent(SignalAgent):
    def reset(self, context):
        self.context = context

    def act(self, observation):
        self.observation = observation
        return 2


class InterfaceTests(unittest.TestCase):
    def test_official_state_preserved_and_context_discriminated(self):
        agent = RecordingAgent()
        callback = OfficialSignalCallback(agent)
        state = [{'cur_phase': [1]*8, 'coming_vehicle': [0]*36}]
        self.assertEqual(callback(state, [0, 0, 2, 0], 1, 12.5), 2)
        self.assertIs(agent.observation['raw']['state'], state)
        self.assertEqual(agent.context['observation_schema'], 'cross3_official_80_v1')
        self.assertEqual(agent.observation['elapsed_seconds'], 12.5)

    def test_cross3_safety_layer_retained(self):
        selector = SignalSelector(MaxPressureAgent())
        policy = SafeAdaptivePolicy(selector, movements=4, max_green_seconds=20)
        obs = dict(pressure=[0, 1, 8, 0], incoming=[0, 1, 8, 0], elapsed=0)
        policy.update(0, obs)
        self.assertEqual(policy.decide(0)['state'], 'Red')
        policy.update(2, obs)
        self.assertEqual(policy.decide(2)['active_movement'], 2)
        self.assertEqual(policy.decide(2)['state'], 'Green')
        self.assertEqual(selector.select(obs, 2), 2)

    def test_invalid_signal_actions_rejected(self):
        agent = RecordingAgent()
        selector = SignalSelector(agent)
        for invalid in (-1, 4, True, 1.5, '1'):
            agent.act = lambda obs, action=invalid: action
            with self.assertRaises(ValueError):
                selector.select({'pressure': [0]*4})

    def test_plugin_loading_and_kind_validation(self):
        with TemporaryDirectory() as folder:
            path = Path(folder)/'plugin.json'
            path.write_text(json.dumps(dict(schema='drive_agent_plugin_v1', kind='signal',
                                            factory='agent_interfaces.signals:MaxPressureAgent')))
            self.assertIsInstance(load_plugin(path, 'signal'), MaxPressureAgent)
            with self.assertRaises(ValueError):
                load_plugin(path, 'driving')

    def test_plugin_expands_environment_in_kwargs(self):
        with TemporaryDirectory() as folder:
            path = Path(folder)/'plugin.json'
            path.write_text(json.dumps(dict(schema='drive_agent_plugin_v1', kind='signal',
                                            factory='agent_interfaces.signals:MaxPressureAgent',
                                            kwargs={'checkpoint': '${TEST_PLUGIN_MODEL}'})))
            with patch.dict('os.environ', TEST_PLUGIN_MODEL='/models/test'), patch(
                    'agent_interfaces.signals.MaxPressureAgent') as factory:
                load_plugin(path, 'signal')
                factory.assert_called_once_with(checkpoint='/models/test')

    def test_driving_validation_and_failure_cleanup(self):
        agent = SimpleNamespace(ego_vehicles_num=1, destroy=lambda: closed.append(True))
        closed = []
        with patch('agent_interfaces.plugins.load_plugin', return_value=agent):
            with self.assertRaises(TypeError):
                load_driving_agent('unused', 2)
        self.assertEqual(closed, [True])

    def test_driving_factory_and_ego_mismatch(self):
        with TemporaryDirectory() as folder:
            path = Path(folder)/'driving.json'
            path.write_text(json.dumps(dict(schema='drive_agent_plugin_v1', kind='driving',
                                            factory=__name__+':FakeDrivingAgent')))
            self.assertIsInstance(load_driving_agent(path, 2), FakeDrivingAgent)
        agent = FakeDrivingAgent()
        with patch('agent_interfaces.plugins.load_plugin', return_value=agent):
            with self.assertRaises(ValueError):
                load_driving_agent('unused', 1)
        self.assertTrue(agent.closed)

    def test_official_presslight_refuses_counts_without_model_runtime(self):
        agent = OfficialPressLightAgent.__new__(OfficialPressLightAgent)
        with self.assertRaisesRegex(ValueError, '80-D'):
            SignalSelector(agent)

    def test_pressure_dimension_rejected(self):
        with self.assertRaises(ValueError):
            SignalSelector(MaxPressureAgent()).select({'pressure': [1, 2]})

    def test_actuator_validation(self):
        control = SimpleNamespace(throttle=.5, brake=0., steer=-.5)
        self.assertEqual(validate_controls([control, control], 2), [control, control])
        with self.assertRaises(ValueError):
            validate_controls([control], 2)
        for invalid in (float('nan'), float('inf'), 1.1):
            control.throttle = invalid
            with self.assertRaises(ValueError):
                validate_controls([control, control], 2)


if __name__ == '__main__':
    unittest.main()
