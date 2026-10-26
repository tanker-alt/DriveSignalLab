import unittest
from evaluation.trip_tracker import TripTracker
from evaluation.vehicle_tracker import VehicleTracker


class TripTrackerTest(unittest.TestCase):
    def setUp(self):
        self.wait=VehicleTracker(junction_id=1,observation_warmup_seconds=0)
        self.trips=TripTracker(1)

    def step(self,frame,now,region="approach",events=(),present=True,match="unique"):
        rows=[dict(vehicle_id=7,vx=0 if region=="approach" else 1,vy=0,road_id=10,
            section_id=0,lane_id=1,current_junction_id=1 if region=="inside" else "",
            is_junction=region=="inside",signal_match=match if region=="approach" else "unmapped",
            signal_opendrive_id="A")] if present else []
        self.wait.update(frame,now,rows)
        crossings=[dict(vehicle_id=7,kind=kind,line_id=kind+"_line",estimated_crossing_time=time)
                   for kind,time in events]
        return self.trips.update(frame,now,rows,crossings,self.wait)

    def test_pair_wait_interval_and_repeat_visit(self):
        self.step(1,0)
        self.step(2,1,events=[("arrival",.5)])
        self.step(3,2)
        self.step(4,3,"inside")
        first=self.step(5,4,"outside",[("exit",3.5)])[0]
        self.assertEqual(first["travel_time_s"],3)
        self.assertEqual(first["approach_wait_s"],1.5)
        self.assertEqual(first["wait_quality"],"covered")
        self.step(6,5)
        self.step(7,6,events=[("arrival",5.5)])
        self.step(8,7,"inside")
        second=self.step(9,8,"outside",[("exit",7.5)])[0]
        self.assertNotEqual(first["trip_id"],second["trip_id"])
        self.assertEqual(second["approach_wait_s"],.5)
        self.assertEqual(self.trips.summary()["mean_paired_approach_wait_s"],1)

    def test_gap_and_missing_arrival_do_not_create_complete_trip(self):
        self.step(1,0)
        self.step(2,1,events=[("arrival",.5)])
        result=self.step(10,9,"inside")
        self.assertEqual(result[0]["reason"],"frame_gap")
        result=self.step(11,10,"outside",[("exit",9.5)])
        self.assertEqual(result[0]["reason"],"exit_without_arrival")
        self.assertEqual(self.trips.summary()["paired_trip_count"],0)

    def test_continuous_return_keeps_warmup_and_boundary_baseline(self):
        self.wait.warmup=2
        self.step(1,0)
        self.step(2,1)
        self.step(3,2)
        self.step(4,3,"inside")
        self.step(5,4,"outside")
        # Arrival coincides with the first frame of a new visit. Observation
        # has been continuous, although the per-visit wait resets here.
        self.step(6,5,events=[("arrival",4.5)])
        self.assertTrue(self.wait.active["7"]["waiting"])
        self.assertEqual(self.wait.active["7"]["first_time"],5)
        self.step(7,6)
        self.step(8,7,"inside")
        trip=self.step(9,8,"outside",[("exit",7.5)])[0]
        self.assertEqual(trip["wait_quality"],"covered")
        self.assertEqual(trip["approach_wait_s"],1)
        self.assertEqual(trip["wait_issues"],[])

    def test_lost_observation_restarts_warmup(self):
        for gap in (False,True):
            self.setUp()
            self.wait.warmup=2
            self.step(1,0)
            self.step(2,1)
            self.step(3,2)
            if not gap:
                self.step(4,3,present=False)
            self.step(5,4)
            self.step(6,5,events=[("arrival",4.5)])
            self.step(7,6,"inside")
            trip=self.step(8,7,"outside",[("exit",6.5)])[0]
            self.assertIn("observation_warmup",trip["wait_issues"])
            self.assertIsNone(trip["approach_wait_s"])

    def test_warmup_excluded_from_complete_wait_denominator(self):
        self.wait.warmup=2
        self.step(1,0)
        self.step(2,1,events=[("arrival",.5)])
        self.step(3,2,"inside")
        trip=self.step(4,3,"outside",[("exit",2.5)])[0]
        self.assertEqual(trip["status"],"paired")
        self.assertIsNone(trip["approach_wait_s"])
        self.assertIn("observation_warmup",trip["wait_issues"])
        self.assertEqual(self.trips.summary()["wait_excluded_paired_count"],1)

    def test_signal_gap_invalidates_wait_not_travel_pair(self):
        self.step(1,0)
        self.step(2,1,events=[("arrival",.5)])
        self.step(3,2,match="ambiguous")
        self.step(4,3,"inside")
        trip=self.step(5,4,"outside",[("exit",3.5)])[0]
        self.assertEqual(trip["status"],"paired")
        self.assertIsNone(trip["approach_wait_s"])
        self.assertIn("approach_signal_unresolved",trip["wait_issues"])

    def test_repeated_arrival_and_end_are_censored(self):
        self.step(1,0)
        self.step(2,1,events=[("arrival",.5)])
        emitted=self.step(3,2,events=[("arrival",1.5)])
        self.assertEqual(emitted[0]["reason"],"repeated_arrival")
        self.assertEqual(self.trips.close()[0]["reason"],"experiment_end")

    def test_exit_requires_observed_target_junction(self):
        self.step(1,0)
        self.step(2,1,events=[("arrival",.5)])
        result=self.step(3,2,"outside",[("exit",1.5)])[0]
        self.assertEqual(result["reason"],"junction_not_observed")
        self.assertIsNone(result["travel_time_s"])
