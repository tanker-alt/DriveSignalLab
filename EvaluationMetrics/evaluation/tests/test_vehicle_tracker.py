"""对等待区间、消失/缺帧截断和驶出后停车进行独立行为验证。"""

import unittest
from evaluation.vehicle_tracker import VehicleTracker


def row(speed=0, region="approach", distance=5):
    return dict(vehicle_id=1, vx=speed, vy=0, road_id=18 if region == "approach" else 99,
        section_id=0, lane_id=1, signal_opendrive_id="2401", signal_state="Red",
        signal_match="unique" if region == "approach" else "unmapped",
        current_junction_id=1427 if region == "inside" else "",
        is_junction=region == "inside", stop_plane_front_distance_m=distance)


class TrackerTest(unittest.TestCase):
    def test_downstream_stop_excluded(self):
        t = VehicleTracker(observation_warmup_seconds=0)
        for frame, r in enumerate([row(), row(), row(), row(1), row(1,"inside"),
                                   row(1,"outside"), row(0,"outside"), row(0,"outside")], 1):
            t.update(frame, float(frame), [r])
        t.close()
        self.assertEqual(t.completed[0]["wait_s"], 2)
        self.assertTrue(t.completed[0]["completed"])
        self.assertEqual(t.completed[0]["stop_count"], 1)

    def test_gap_and_disappearance_are_censored(self):
        t = VehicleTracker(observation_warmup_seconds=0)
        t.update(1, 1, [row()]); t.update(2, 2, [row()])
        t.update(10, 10, [row()]); t.update(11, 11, [])
        self.assertEqual([v["wait_s"] for v in t.completed], [1, 0])
        self.assertFalse(any(v["completed"] for v in t.completed))
        self.assertEqual([v["reason"] for v in t.completed], ["frame_gap", "observation_lost"])

    def test_hysteresis_warmup_and_reference_crossing(self):
        t = VehicleTracker(observation_warmup_seconds=1)
        t.update(1, 0, [row()]); self.assertFalse(t.active["1"]["waiting"])
        t.update(2, 1, [row()]); t.update(3, 2, [row(.2)])
        events = t.update(4, 3, [row(.4, distance=-1)])
        self.assertEqual(t.active["1"]["wait_s"], 1)
        crossing = next(e for e in events if e["event"] == "reference_plane_crossing")
        self.assertIsNone(crossing["is_violation"])

    def test_seen_inside_is_not_approach_wait(self):
        t = VehicleTracker(observation_warmup_seconds=0)
        events = t.update(1, 1, [row(0,"inside")])
        self.assertTrue(next(e for e in events if e["event"] == "junction_enter")["left_censored"])
        t.update(2, 2, [row(0,"outside")]); t.close()
        self.assertEqual(t.completed[0]["wait_s"], 0)
