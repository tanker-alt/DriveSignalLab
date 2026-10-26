import copy
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from Toolkits.JunctionToolkit.safety_rules import (
    default_rules, validate_rules, write_default_rules, resolve_stop_lines)


MAP = (Path(__file__).resolve().parents[3] / 'InformationCollector').parent / 'Maps/MapDataset/maps/cross3'


class SafetyRulesTests(unittest.TestCase):
    def setUp(self):
        self.raw = (MAP / 'map.xodr').read_bytes()
        self.design = json.loads((MAP / 'manifest.json').read_text(encoding='utf-8'))
        self.rules = default_rules(self.design)

    def test_cross3_complete_geometry_without_assessment_policy(self):
        validate_rules(self.rules, self.design, self.raw)
        self.assertEqual(len(self.rules['stop_lines']), 12)
        self.assertEqual({(line['road_id'], line['lane_id'], line['s_m']) for line in self.rules['stop_lines']},
                         {(road, lane, 144) for road in (1, 2, 3, 4) for lane in (-1, -2, -3)})
        self.assertIsNone(self.rules['assessment']['red_light_violation'])

    def test_missing_duplicate_or_wrong_signal_rejected(self):
        missing = copy.deepcopy(self.rules)
        missing['stop_lines'].pop()
        duplicate = copy.deepcopy(self.rules)
        duplicate['stop_lines'].append(copy.deepcopy(duplicate['stop_lines'][0]))
        wrong = copy.deepcopy(self.rules)
        wrong['stop_lines'][0]['signal_opendrive_id'] = '1043'
        for rules in (missing, duplicate, wrong):
            with self.assertRaises(ValueError):
                validate_rules(rules, self.design, self.raw)

    def test_station_bounds_and_nonfinite_rejected(self):
        for station in (-1, 0, 150, float('nan'), float('inf'), True):
            rules = copy.deepcopy(self.rules)
            rules['stop_lines'][0]['s_m'] = station
            with self.assertRaises(ValueError):
                validate_rules(rules, self.design, self.raw)

    def test_changed_map_rejected(self):
        with self.assertRaisesRegex(ValueError, 'fingerprint'):
            validate_rules(self.rules, self.design, self.raw + b'\n')

    def test_custom_station_preserved_and_not_overwritten(self):
        rules = copy.deepcopy(self.rules)
        rules['stop_lines'][0]['s_m'] = 140
        validate_rules(rules, self.design, self.raw)
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            (folder / 'map.xodr').write_bytes(self.raw)
            write_default_rules(folder, self.design)
            path = folder / 'safety_rules.json'
            path.write_text(json.dumps(rules), encoding='utf-8')
            with self.assertRaises(FileExistsError):
                write_default_rules(folder, self.design)
            self.assertEqual(json.loads(path.read_text())['stop_lines'][0]['s_m'], 140)

    def test_resolved_segment_and_direction(self):
        vector = lambda x, y, z=0: SimpleNamespace(x=x, y=y, z=z)
        transform = SimpleNamespace(location=vector(-28, 1.75),
                                    get_right_vector=lambda: vector(0, 1),
                                    get_forward_vector=lambda: vector(1, 0))
        wp = SimpleNamespace(road_id=1, lane_id=-1, s=144, is_junction=False,
                             lane_width=3.5, transform=transform)
        road_map = SimpleNamespace(to_opendrive=lambda: self.raw.decode('utf-8'),
                                   get_waypoint_xodr=lambda *args: wp)
        rules = copy.deepcopy(self.rules)
        rules['stop_lines'] = rules['stop_lines'][:1]
        line = resolve_stop_lines(road_map, rules)[0]
        self.assertEqual(line['start_world_m'], [-28, 0, 0])
        self.assertEqual(line['end_world_m'], [-28, 3.5, 0])
        self.assertEqual(line['forward_world'], [1, 0, 0])
        wp.s = 143
        with self.assertRaisesRegex(ValueError, 'resolve'):
            resolve_stop_lines(road_map, rules)


if __name__ == '__main__':
    unittest.main()
