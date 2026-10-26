"""清理重试及信号配时恢复的离线回归测试；不连接 CARLA。"""
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

import spawn_junction_traffic as spawn
import control_junction_phases as controller


class LifecycleTest(unittest.TestCase):
    def test_signal_mismatch_records_frame_then_stops(self):
        world=NS(get_snapshot=lambda:NS(timestamp=NS(elapsed_seconds=10.)))
        snapshot=NS(frame=21,timestamp=NS(elapsed_seconds=10.05))
        light=NS(get_opendrive_id=lambda:"A",get_state=lambda:"Red")
        phase=dict(stage="GREEN",active_signal_opendrive_id="A",phase_id="A:GREEN")
        received=[]
        with patch.object(controller,"advance",return_value=snapshot) as advance:
            with self.assertRaisesRegex(RuntimeError,"21"):
                controller.run_interval(world,True,1,lambda s,p:received.append(p),phase,
                    lambda s,p:controller.signal_consistency([light],p))
        self.assertEqual(advance.call_count,1)
        self.assertEqual(received[0]["signal_consistency"],"mismatch")
        self.assertEqual(received[0]["signal_mismatch_ids"],"A")

    def test_all_red_and_yellow_verify_actual_states(self):
        lights=[NS(get_opendrive_id=lambda:"A",get_state=lambda:"Yellow"),
                NS(get_opendrive_id=lambda:"B",get_state=lambda:"Red")]
        self.assertEqual(controller.signal_consistency(lights,dict(stage="YELLOW",active_signal_opendrive_id="A"))["mismatch_ids"],[])
        self.assertEqual(controller.signal_consistency(lights,dict(stage="ALL_RED",active_signal_opendrive_id=""))["mismatch_ids"],["A"])

    def test_phase_countdown_uses_simulation_schedule(self):
        world=NS(get_snapshot=lambda:NS(timestamp=NS(elapsed_seconds=10.)))
        snapshots=[NS(timestamp=NS(elapsed_seconds=t)) for t in (10.05,10.1)]
        received=[]
        with patch.object(controller,"advance",side_effect=snapshots):
            controller.run_interval(world,True,.099,lambda snapshot,phase:received.append(phase),
                dict(phase_id="A:GREEN",next_phase_id="A:YELLOW"))
        self.assertAlmostEqual(received[0]["remaining_seconds"],.049)
        self.assertEqual(received[1]["remaining_seconds"],0)
        self.assertEqual(received[1]["next_phase_id"],"A:YELLOW")

    def test_batch_cleanup_retries_and_does_not_tick(self):
        calls = []
        populations = iter([[NS(id=2), NS(id=999)], [NS(id=999)]])
        world = NS(get_actors=lambda: next(populations))
        def batch(commands, tick):
            calls.append((commands, tick))
            return [NS(has_error=lambda: False) for _ in commands]
        with patch.object(spawn.carla.command, "DestroyActor", side_effect=lambda i: i), patch.object(spawn.time, "sleep"):
            remaining = spawn.destroy_owned_vehicles(NS(apply_batch_sync=batch), world, [NS(id=1), NS(id=2)])
        self.assertEqual(remaining, set())
        self.assertEqual(calls, [([1, 2], False), ([2], False)])

    def test_target_timings_restored(self):
        class Light:
            id = 1
            def __init__(self): self.green, self.yellow, self.red, self.state = 12, 3, 20, "Red"
            def get_state(self): return self.state
            def get_green_time(self): return self.green
            def get_yellow_time(self): return self.yellow
            def get_red_time(self): return self.red
            def set_green_time(self, value): self.green = value
            def set_yellow_time(self, value): self.yellow = value
            def set_red_time(self, value): self.red = value
            def set_state(self, value): self.state = value
        target, unrelated = Light(), Light()
        saved = []
        controller.hold_target_lights([target], saved)
        self.assertEqual(target.green, 1e9)
        self.assertEqual(unrelated.green, 12)
        controller.restore_target_lights(saved)
        self.assertEqual((target.green, target.yellow, target.red, target.state), (12, 3, 20, "Red"))
