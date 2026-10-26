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

    def test_empty_override_does_not_silently_use_default(self):
        for value in ({},None,{"calibration_id":"survey"}):
            override=copy.deepcopy(self.override)
            override['approaches']['2_0_-1']=value
            with self.assertRaises(ValueError):
                build(self.world,self.inventory,self.config,override)

    def survey_fixture(self):
        def wp(x):
            return NS(road_id=8 if x>.5 else 2,section_id=0,lane_id=-2 if x>.5 else -1,
                      lane_width=4,is_junction=x>.5,get_junction=lambda:NS(id=1),
                      transform=NS(location=NS(x=x,y=0,z=0),rotation=NS(yaw=0)),
                      previous=lambda step:[wp(x-step)])
        self.world.get_map().get_waypoint_xodr=lambda road,lane,s:wp(s)
        self.inventory['traffic_lights'][0]['stop_waypoints'][0].update(s=0,location=dict(x=0,y=0,z=0))
        line=dict(reference_waypoint=dict(road_id=8,section_id=0,lane_id=-2,s=7),
                  center=dict(x=7,y=0,z=0),normal_xy=[1,0],estimated_error_m=.1,
                  convention='approach_side_edge',evidence='survey.png')
        self.override['approaches']['2_0_-1']=dict(calibration_id='cross-road',stop_line=line)
        self.config['detection_length_m']=12
        return line

    def test_survey_crosses_target_junction_prefix_and_preserves_approach_id(self):
        self.survey_fixture()
        lane=build(self.world,self.inventory,self.config,self.override)['lanes'][0]
        self.assertEqual(lane['id'],'2_0_-1')
        self.assertEqual(lane['reference_kind'],'surveyed_stop_line')
        self.assertEqual(lane['samples'][0]['x'],7)
        self.assertEqual(lane['samples'][-1]['x'],-5)
        self.assertEqual({p['road_id'] for p in lane['samples']},{2,8})

    def test_survey_rejects_wrong_direction_off_lane_and_disconnected_source(self):
        for change in (dict(normal_xy=[-1,0]),dict(center=dict(x=7,y=1,z=0)),
                       dict(reference_waypoint=dict(road_id=9,section_id=0,lane_id=-2,s=7))):
            self.setUp()
            self.survey_fixture().update(change)
            with self.assertRaises(ValueError):
                build(self.world,self.inventory,self.config,self.override)
        self.setUp()
        self.survey_fixture()
        self.inventory['traffic_lights'][0]['stop_waypoints'][0]['location']['x']=-40
        with self.assertRaises(ValueError):
            build(self.world,self.inventory,self.config,self.override)
