import copy
import json
from pathlib import Path
import unittest

from control_t_phases import build_plan, expected_states, COMPASS

MAPS = Path(__file__).resolve().parent / 'fixtures'


class TPhaseTests(unittest.TestCase):
    def setUp(self):
        self.design = json.loads((MAPS / 't3_north/manifest.json').read_text())

    def test_example_and_straight_always_green(self):
        plan = build_plan(self.design)
        self.assertEqual([p['arm'] for p in plan['phases']], ['west', 'south', 'east'])
        self.assertEqual(set(plan['always_green_signal_ids']), {'1012', '1022', '1032'})
        for index, sid in enumerate(('1011', '1021', '1031')):
            for stage, color in [('GREEN', 'Green'), ('YELLOW', 'Yellow')]:
                states = expected_states(plan, index, stage)
                self.assertEqual(states[sid], color)
                self.assertEqual({s for s in ('1011', '1021', '1031') if states[s] != 'Red'}, {sid})
                self.assertTrue(all(states[s] == 'Green' for s in ('1012', '1022', '1032')))
        self.assertEqual(set(expected_states(plan, 0, 'ALL_RED').values()), {'Red'})
        states = expected_states(plan, 0, 'MAIN_RED')
        self.assertEqual({s for s, color in states.items() if color == 'Green'}, {'1012', '1022', '1032'})

    def test_four_rotations(self):
        for rotation in range(4):
            design = copy.deepcopy(self.design)
            arms = design['junctions'][0]['arms']
            design['junctions'][0]['arms'] = {
                COMPASS[(COMPASS.index(a) + rotation) % 4]: r for a, r in arms.items()}
            plan = build_plan(design)
            self.assertEqual(plan['missing_arm'], COMPASS[(3 + rotation) % 4])
            self.assertEqual([p['arm'] for p in plan['phases']],
                             [COMPASS[(i + rotation) % 4] for i in range(3)])
            self.assertEqual(set(plan['always_green_signal_ids']), {'1012', '1022', '1032'})

    def test_network_isolation(self):
        found = set()
        for name in ('t_1x6', 't_3x3', 'mixed_1x6', 'mixed_3x3'):
            design = json.loads((MAPS / name / 'manifest.json').read_text())
            with self.assertRaises(ValueError):
                build_plan(design)
            for junction in design['junctions']:
                if len(junction.get('arms', {})) != 3:
                    continue
                plan = build_plan(design, junction['id'])
                found.add(plan['missing_arm'])
                self.assertEqual(len(plan['always_green_signal_ids']), 3)
                self.assertTrue(all(s['junction_id'] == junction['id'] for s in plan['signals'].values()))
                self.assertEqual(set(expected_states(plan, 0, 'GREEN')), set(plan['signals']))
        self.assertEqual(found, set(COMPASS))

    def test_reject_bad_coverage_and_shared_signals(self):
        for mutation in ('missing', 'shared', 'duplicate'):
            design = copy.deepcopy(self.design)
            if mutation == 'missing':
                del design['signals']['1012']
            elif mutation == 'shared':
                design['signals']['1011']['incoming_lanes'] = [-1, -2]
            else:
                design['signals']['extra'] = copy.deepcopy(design['signals']['1011'])
            with self.assertRaises(ValueError):
                build_plan(design)


if __name__ == '__main__':
    unittest.main()
