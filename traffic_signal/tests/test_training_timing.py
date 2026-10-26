#!/usr/bin/env python3
"""Unit checks for the timing-aware synthetic PressLight environment."""
import random
import unittest
from unittest.mock import patch

from traffic_signal.train_two_movement import advance


class PressLightTrainingTimingTests(unittest.TestCase):
    def test_keep_uses_decision_interval(self):
        queues = [8, 0]
        random.seed(1)
        with patch('traffic_signal.train_two_movement.random.random', return_value=1.0), \
             patch('traffic_signal.train_two_movement.random.randint', return_value=1):
            _, _, duration = advance(queues, [0, 0], 8, 1, 0, 0,
                                     decision_seconds=2, yellow_seconds=3,
                                     clearance_seconds=2, min_green_seconds=5,
                                     gamma=.8)
        self.assertEqual(duration, 2)
        self.assertEqual(queues, [6, 0])

    def test_switch_blocks_discharge_during_yellow_and_all_red(self):
        queues = [0, 8]
        random.seed(1)
        with patch('traffic_signal.train_two_movement.random.random', return_value=1.0), \
             patch('traffic_signal.train_two_movement.random.randint', return_value=1):
            _, _, duration = advance(queues, [0, 0], 8, 1, 0, 1,
                                     decision_seconds=2, yellow_seconds=3,
                                     clearance_seconds=2, min_green_seconds=5,
                                     gamma=.8)
        self.assertEqual(duration, 10)
        # Five vehicles discharge during minimum green; none discharge during
        # the preceding three yellow plus two all-red seconds.
        self.assertEqual(queues, [0, 3])


if __name__ == '__main__':
    unittest.main()
