"""Map-independent initialization, geometry, and replay regression tests."""
import csv
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

from evaluation.api import RoadSpec, create_collector
from evaluation.map_context import read_inventory
from evaluation.summarize import replay
from evaluation.topology import build


def waypoint(road, x, inside=False):
    return NS(road_id=road, section_id=0, lane_id=-1, s=1, lane_width=3.5,
              is_junction=inside, transform=NS(location=NS(x=x, y=0, z=0),
              rotation=NS(yaw=0), get_forward_vector=lambda: NS(x=1,y=0,z=0)))


class MapContextTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.entry, self.exit = waypoint(1,0,True), waypoint(2,20,True)
        self.entry.previous = lambda d: [waypoint(10,-d)]
        self.exit.next = lambda d: [waypoint(20,20+d)]
        junction = NS(id=73, bounding_box=NS(location=NS(x=10,y=0)),
                      get_waypoints=lambda kind: [(self.entry,self.exit)])
        self.entry.get_junction = lambda: junction
        self.road_map = NS(name="CustomMap", generate_waypoints=lambda d: [self.entry],
                           to_opendrive=lambda: "custom road data")
        self.world = NS(get_map=lambda: self.road_map,
                        get_traffic_lights_in_junction=lambda j: [],
                        get_settings=lambda: NS(synchronous_mode=True,fixed_delta_seconds=.05),
                        get_actors=lambda: NS(filter=lambda pattern: []),
                        get_snapshot=lambda: NS(frame=1))

    def test_unsignalized_custom_map_and_replay(self):
        inventory = read_inventory(self.world,73)
        self.assertEqual(inventory["center_xy"],[10,0])
        self.assertEqual(len(inventory["opendrive_sha256"]),64)
        collector = create_collector(self.world,RoadSpec(73,arrival_distance_m=10),self.root)
        self.addCleanup(collector.close)
        packet = collector.sample(NS(frame=1,timestamp=NS(elapsed_seconds=.05,delta_seconds=.05)))
        self.assertEqual(packet["signals"],{})
        self.assertEqual(packet["vehicles"],[])
        collector.close()
        lines = json.loads((collector.output/"detection_lines_used.json").read_text())
        self.assertEqual({line["road_id"] for line in lines["lines"]},{10,20})
        result = replay(collector.output)
        self.assertEqual(json.loads((result/"detection_lines_used.json").read_text()),lines)
        with (result/"flows.csv").open(encoding="utf-8-sig") as handle:
            self.assertEqual(len(list(csv.DictReader(handle))),2)

    def test_branch_requires_user_geometry(self):
        self.entry.previous = lambda d: [waypoint(10,-d),waypoint(11,-d)]
        with self.assertRaisesRegex(ValueError,"Ambiguous"):
            create_collector(self.world,RoadSpec(73,arrival_distance_m=10),self.root)
        self.assertEqual(list(self.root.iterdir()),[])
        # User may explicitly request raw collection with no counting sections.
        collector = create_collector(self.world,RoadSpec(73),self.root,
                    detection_config=dict(map_name="CustomMap",junction_id=73,lines=[]))
        collector.close()

    def test_wrong_map_or_junction_rejected(self):
        with self.assertRaisesRegex(ValueError,"map"):
            create_collector(self.world,RoadSpec(73,expected_map="Town05"),self.root)
        with self.assertRaisesRegex(ValueError,"not found"):
            read_inventory(self.world,1427)
        with self.assertRaisesRegex(ValueError,"configuration"):
            create_collector(self.world,RoadSpec(73),self.root,
                detection_config=dict(map_name="Town05",junction_id=73,lines=[]))

    def test_custom_section_distances(self):
        config=build(self.world,73,15,8)
        self.assertEqual([line["center_xy"][0] for line in config["lines"]],[-15,28])

    def test_default_alignment_requires_geometry(self):
        with self.assertRaisesRegex(ValueError,"approach geometry"):
            create_collector(self.world,RoadSpec(73),self.root)
        self.assertEqual(list(self.root.iterdir()),[])
