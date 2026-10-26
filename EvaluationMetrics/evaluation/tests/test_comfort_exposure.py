import unittest
from evaluation.comfort_events import ComfortEvents


class ExposureTest(unittest.TestCase):
    def sample(self, e, frame, now, a, speed=5, vid=1):
        return e.sample(frame, now, [dict(vehicle_id=vid, longitudinal_accel_mps2=a,
                                        vx=speed, vy=0, yaw=0)])

    def vehicle(self, e, vid='1'):
        return e.summary()['threshold_exceedance_duration']['vehicles'][vid]

    def test_short_excursion_interpolation_and_hysteresis(self):
        e = ComfortEvents([1])
        for frame, a in enumerate((2, 4, 2, 1)):
            self.sample(e, frame, frame*.1, a)
        v = self.vehicle(e)
        m = v['metrics']['harsh_acceleration']
        self.assertEqual(m['completed_event_count'], 0)
        self.assertAlmostEqual(m['exceedance_seconds'], .1)
        self.assertAlmostEqual(m['longest_continuous_exceedance_seconds'], .1)
        self.assertAlmostEqual(m['exceedance_ratio'], 1/3)
        self.assertAlmostEqual(v['valid_forward_seconds'], .3)

    def test_both_thresholds_crossed_and_per_actor_scope(self):
        e = ComfortEvents([1, 2])
        self.sample(e, 0, 0., -4)
        self.sample(e, 1, 1., 4)
        v = self.vehicle(e)
        for m in v['metrics'].values():
            self.assertAlmostEqual(m['exceedance_seconds'], .125)
        self.assertAlmostEqual(v['combined_exceedance_ratio'], .25)
        self.assertIsNone(self.vehicle(e, '2')['combined_exceedance_ratio'])

    def test_gaps_stops_reverse_missing_break_coverage(self):
        for speed in (0, -1, None):
            e = ComfortEvents([1])
            self.sample(e, 0, 0., 4)
            self.sample(e, 1, .2, 4)
            self.sample(e, 2, .4, 4, speed)
            self.sample(e, 3, .6, 4)
            self.sample(e, 4, .8, 4)
            self.sample(e, 6, 1.2, 4)
            self.sample(e, 7, 1.4, 4)
            v = self.vehicle(e)
            self.assertAlmostEqual(v['valid_forward_seconds'], .6)
            self.assertAlmostEqual(v['metrics']['harsh_acceleration']['exceedance_seconds'], .6)
            self.assertAlmostEqual(v['metrics']['harsh_acceleration']['longest_continuous_exceedance_seconds'], .2)

    def test_event_frequency_and_censored_time_retained(self):
        e = ComfortEvents([1])
        for frame, a in enumerate((0, 4, 4, 4, 2, 1, 4, 4, 4)):
            self.sample(e, frame, frame*.25, a)
        before = self.vehicle(e)
        e.close()
        after = self.vehicle(e)
        m = after['metrics']['harsh_acceleration']
        self.assertEqual(m['completed_event_count'], 1)
        self.assertEqual(m['censored_event_count'], 1)
        self.assertEqual(m['events_per_minute'], 30)
        self.assertEqual(after['combined_exceedance_seconds'], before['combined_exceedance_seconds'])

    def test_exact_threshold_and_zero_duration(self):
        e = ComfortEvents([1])
        self.sample(e, 0, 0., 3)
        self.assertIsNone(self.vehicle(e)['combined_exceedance_ratio'])
        self.sample(e, 1, .1, 3)
        self.assertEqual(self.vehicle(e)['combined_exceedance_ratio'], 0)
