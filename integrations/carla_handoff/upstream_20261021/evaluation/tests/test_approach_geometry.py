import copy
import hashlib
import unittest
from types import SimpleNamespace as NS
from evaluation.approach_geometry import build


class ReferenceCalibrationTest(unittest.TestCase):
    def setUp(self):
        self.requested = []
        def waypoint(road, lane, s):
            self.requested.append(s)
            return NS(section_id=0, road_id=road, lane_id=lane, lane_width=4,
                      transform=NS(location=NS(x=s, y=0, z=0)), previous=lambda step: [])
        road_map = NS(name="TownTest", to_opendrive=lambda: "map", get_waypoint_xodr=waypoint)
        self.world = NS(get_map=lambda: road_map)
        self.inventory = dict(map_name="TownTest", junction_id=1, traffic_lights=[
            dict(opendrive_id="9", stop_waypoints=[dict(road_id=2, section_id=0, lane_id=-1, s=20)])])
        self.config = dict(detection_length_m=50, sample_step_m=1)
        self.override = dict(map_name="TownTest", junction_id=1,
            opendrive_sha256=hashlib.sha256(b"map").hexdigest(),
            approaches={"2_0_-1": dict(s=18, calibration_id="survey-1")})

    def test_default_and_calibrated_reference(self):
        default = build(self.world, self.inventory, self.config)
        calibrated = build(self.world, self.inventory, self.config, self.override)
        self.assertEqual(self.requested, [20, 18])
        self.assertEqual(default["lanes"][0]["reference_kind"], "carla_stop_waypoint")
        self.assertEqual(calibrated["lanes"][0]["reference_kind"], "user_supplied_stop_waypoint")
        self.assertEqual(calibrated["lanes"][0]["calibration_id"], "survey-1")

    def test_reject_wrong_map_junction_fingerprint(self):
        for key, value in [("map_name", "Other"), ("junction_id", 2), ("opendrive_sha256", "bad")]:
            override = copy.deepcopy(self.override)
            override[key] = value
            with self.assertRaises(ValueError):
                build(self.world, self.inventory, self.config, override)

    def test_reject_invalid_reference(self):
        for change in [dict(s=-1), dict(s=float("nan")), dict(calibration_id="")]:
            override = copy.deepcopy(self.override)
            override["approaches"]["2_0_-1"].update(change)
            with self.assertRaises(ValueError):
                build(self.world, self.inventory, self.config, override)
        self.override["approaches"]["unknown"] = dict(s=1, calibration_id="x")
        with self.assertRaises(ValueError):
            build(self.world, self.inventory, self.config, self.override)
