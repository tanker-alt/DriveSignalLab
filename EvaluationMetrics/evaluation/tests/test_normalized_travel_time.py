import unittest
from evaluation.normalized_travel_time import NormalizedTravelTime

class NormalizedTravelTests(unittest.TestCase):
    def test_bent_path_clipped_and_ratio_direction(self):
        e=NormalizedTravelTime(3.6)
        for f,(x,y) in enumerate(((0,0),(2,0),(2,2),(4,2))):
            e.observe(f,float(f),[dict(vehicle_id=1,x=x,y=y,z=0)])
        t=dict(vehicle_id=1,status='paired',arrival_time=.5,exit_time=2.5,travel_time_s=2.)
        e.annotate(t);self.assertEqual(t['normalized_travel_time']['path_length_m'],4)
        self.assertEqual(e.summary()['value'],2) # above one must not be clipped

    def test_mean_of_ratios_not_ratio_of_sums(self):
        e=NormalizedTravelTime(3.6)
        for f in range(5):e.observe(f,float(f),[dict(vehicle_id=1,x=f,y=0,z=0),dict(vehicle_id=2,x=2*f,y=0,z=0)])
        e.annotate(dict(vehicle_id=1,status='paired',arrival_time=0,exit_time=1,travel_time_s=1))
        e.annotate(dict(vehicle_id=2,status='paired',arrival_time=0,exit_time=4,travel_time_s=4))
        self.assertEqual(e.summary()['value'],1.5)

    def test_gap_and_censored_and_missing_geometry_excluded(self):
        e=NormalizedTravelTime(60)
        for f in (1,3):e.observe(f,float(f),[dict(vehicle_id=1,x=f,y=0,z=0)])
        e.annotate(dict(vehicle_id=1,status='paired',arrival_time=1,exit_time=3,travel_time_s=2))
        e.annotate(dict(vehicle_id=1,status='censored',reason='experiment_end'))
        self.assertIsNone(e.summary()['value']);self.assertEqual(e.summary()['excluded_trip_count'],2)

    def test_stationary_wait_in_denominator(self):
        e=NormalizedTravelTime(3.6)
        for f,x in enumerate((0,0,0,1)):e.observe(f,float(f),[dict(vehicle_id=1,x=x,y=0,z=0)])
        e.annotate(dict(vehicle_id=1,status='paired',arrival_time=0,exit_time=3,travel_time_s=3))
        self.assertAlmostEqual(e.summary()['value'],1/3)

    def test_invalid_limit(self):
        for v in (0,-1,float('nan'),float('inf')):
            with self.assertRaises(ValueError):NormalizedTravelTime(v)