import ast
import importlib
import sys
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from evaluation.tests import test_collector
from evaluation.collector import FrameCollector
from evaluation.evaluator import MetricEvaluator


class SplitTests(unittest.TestCase):
    def test_collector_does_not_create_or_invoke_evaluator(self):
        fixture = test_collector.CollectorTest()
        fixture.setUp()
        try:
            with patch('evaluation.recorder.EvaluationRecorder', side_effect=AssertionError('evaluation leaked')):
                c = FrameCollector(fixture.world, inventory=fixture.inventory, output_root=fixture.root)
                observation = c.sample_observation(fixture.snapshot(10))
                c.close()
            self.assertFalse((c.output/'assessment').exists())
            self.assertFalse(hasattr(c, 'evaluation'))
            self.assertNotIn('world_snapshot', observation)
            self.assertNotIn('MDrive', observation)
            self.assertEqual(observation, c.last_observation)
        finally:
            fixture.doCleanups()

    def test_public_modules_and_frame_contract(self):
        sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
        self.assertIs(importlib.import_module('InformationCollector').FrameCollector, FrameCollector)
        self.assertIs(importlib.import_module('EvaluationMetrics').MetricEvaluator, MetricEvaluator)
        with tempfile.TemporaryDirectory() as tmp:
            e = MetricEvaluator(tmp, 10)
            packet = dict(frame=1, sim_time=.1, junction_id=10, vehicles=[], signals={})
            e.sample(packet)
            with self.assertRaises(ValueError): e.sample(packet)
            with self.assertRaises(ValueError): e.sample(dict(packet,frame=2,sim_time=.2,junction_id=11))
            report = e.close()
            self.assertFalse(report['MDrive']['available'])
            self.assertIn('No explicit', report['MDrive']['reason'])
            self.assertNotIn('ttc', report['Safety'])
            self.assertNotIn('lane_keeping', report['Safety'])
            self.assertIn('speeding', report['Safety'])
            self.assertIs(e.close(), report)
