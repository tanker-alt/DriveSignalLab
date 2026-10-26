import math
import tempfile
import unittest

from evaluation.comfort import AccelerationMetrics
from evaluation.data_products import AssessmentWriter


class AccelerationTest(unittest.TestCase):
    def test_vla_scope_statistics_and_coverage(self):
        metrics = AccelerationMetrics([7, 8])
        def sample(frame, now, acceleration):
            return metrics.sample(frame, now, [
                dict(vehicle_id=7, longitudinal_accel_mps2=acceleration, lateral_accel_mps2=0),
                dict(vehicle_id=9, longitudinal_accel_mps2=1000)])
        self.assertEqual(len(sample(1, 0., -3)), 1)
        sample(2, .5, 4)
        sample(4, 2., float('nan'))
        sample(5, 3., 0)
        report = metrics.summary()
        stats = report['vehicles']['7']['longitudinal_accel_mps2']
        self.assertEqual(stats['sample_count'], 3)
        self.assertAlmostEqual(stats['rms'], math.sqrt(25/3))
        self.assertAlmostEqual(stats['p95_abs'], 3.9)
        self.assertEqual(stats['covered_seconds'], .5)
        self.assertAlmostEqual(stats['time_weighted_rms'], math.sqrt(12.5))
        self.assertEqual(stats['max_acceleration_mps2'], 4)
        self.assertEqual(stats['max_braking_mps2'], 3)
        self.assertIsNone(report['vehicles']['8']['lateral_accel_mps2']['rms'])

    def test_no_identity_and_writer_integration(self):
        for ids in (None, [7]):
            with tempfile.TemporaryDirectory() as tmp:
                writer = AssessmentWriter(tmp, comfort_vehicle_ids=ids)
                writer.sample(1, 0., [dict(vehicle_id=7, longitudinal_accel_mps2=2)], [], [], {})
                comfort = writer.close('completed', {})['Comfort']
                self.assertEqual(comfort['acceleration']['status'], 'available' if ids else 'pending_vla_identity')
                self.assertIsNone(comfort['jerk']['value'])
                self.assertIn('Incomplete', comfort['jerk']['reason'])

    def test_reappearance_does_not_bridge_duration(self):
        metrics = AccelerationMetrics([1])
        row = dict(vehicle_id=1, longitudinal_accel_mps2=1)
        metrics.sample(1, 0., [row])
        metrics.sample(2, .1, [])
        metrics.sample(3, .2, [row])
        stats = metrics.summary()['vehicles']['1']['longitudinal_accel_mps2']
        self.assertEqual(stats['covered_seconds'], 0)
        self.assertIsNone(stats['time_weighted_rms'])
