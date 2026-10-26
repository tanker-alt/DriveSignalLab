import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
from Toolkits.MapTools.__main__ import inspect


class MapToolsTest(unittest.TestCase):
    def test_current_map_and_explicit_selection(self):
        junction=NS(bounding_box=NS(location=NS(x=1,y=2,z=3)),get_waypoints=lambda kind:[])
        # No mutation methods: accidental load/tick/write would fail this test.
        world=NS(get_map=lambda:NS(name="Custom",to_opendrive=lambda:"roads"),
                 get_traffic_lights_in_junction=lambda jid:[])
        with patch("Toolkits.MapTools.__main__.junctions",return_value={73:junction}),patch(
                "Toolkits.MapTools.__main__.read_inventory",return_value={"junction_id":73}):
            overview=inspect(world)
            self.assertEqual(overview["map_name"],"Custom")
            self.assertEqual(overview["junctions"][0]["junction_id"],73)
            self.assertNotIn("inventory",overview["junctions"][0])
            detail=inspect(world,73)
            self.assertEqual(detail["junctions"][0]["inventory"],{"junction_id":73})
            with self.assertRaises(ValueError):inspect(world,1427)
