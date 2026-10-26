import copy
import json
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from control_four_phases import DEFAULT_MAP, PHASES, active_ids, prepare, normalize_design, expected_states

LEGACY_MAP = DEFAULT_MAP.with_name('test')


class FourPhaseTests(unittest.TestCase):
    def test_native_validity_and_phase_membership(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp) / 'map'
            design = prepare(LEGACY_MAP, folder)
            xml = ET.parse(str(folder / 'map.xodr'))
            self.assertEqual(len(design['signals']), 16)
            for sid, cfg in design['signals'].items():
                validity = xml.find("./road/signals/signal[@id='{}']/validity".format(sid))
                self.assertEqual(int(validity.get('fromLane')), cfg['incoming_lanes'][0])
                self.assertEqual(validity.get('fromLane'), validity.get('toLane'))
            groups = [active_ids(design, phase, 'with-straight') for phase in PHASES]
            self.assertEqual([len(g) for g in groups], [6, 2, 6, 2])
            self.assertEqual(set.union(*groups), set(design['signals']))
            for i, group in enumerate(groups):
                for other in groups[i + 1:]:
                    self.assertFalse(group & other)
            ns = [design['signals'][sid] for sid in groups[0]]
            self.assertEqual({c['arm'] for c in ns}, {'north', 'south'})
            self.assertNotIn('left', {c['movement'] for c in ns})
            for phase in PHASES:
                self.assertFalse(any(design['signals'][sid]['movement'] == 'right'
                                     for sid in active_ids(design, phase, 'red')))
            with self.assertRaises(ValueError):
                prepare(LEGACY_MAP, folder)

    def test_shared_left_straight_lane_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / 'source'
            source.mkdir()
            design = json.loads((LEGACY_MAP / 'manifest.json').read_text())
            extra = copy.deepcopy(design['movements'][0])
            extra['movement'] = 'straight'
            design['movements'].append(extra)
            (source / 'manifest.json').write_text(json.dumps(design))
            (source / 'map.xodr').write_bytes((LEGACY_MAP / 'map.xodr').read_bytes())
            with self.assertRaisesRegex(ValueError, 'Dedicated turn lanes'):
                prepare(source, Path(tmp) / 'out')

    def test_current_native_map_and_continuous_right_green(self):
        raw = json.loads((DEFAULT_MAP / 'manifest.json').read_text())
        design = normalize_design(raw)
        self.assertEqual(design['opendrive_sha256'], raw['opendrive_sha256'])
        self.assertEqual(len(design['signals']), 12)
        rights = {sid for sid, c in design['signals'].items() if c['movement'] == 'right'}
        self.assertEqual(len(rights), 4)
        for phase in PHASES:
            for stage in ('GREEN', 'YELLOW', 'MAIN_RED', 'ALL_RED'):
                states = expected_states(design, phase, stage)
                self.assertTrue(all(states[sid] == 'Green' for sid in rights))
                for sid, cfg in design['signals'].items():
                    if cfg['movement'] != 'right':
                        wanted = stage.title() if stage in ('GREEN', 'YELLOW') and sid in active_ids(design, phase, 'red') else 'Red'
                        self.assertEqual(states[sid], wanted)
        startup = expected_states(design, '', 'MAIN_RED')
        self.assertEqual({s for s, value in startup.items() if value == 'Green'}, rights)

    def test_native_shared_signal_is_rejected(self):
        design = json.loads((DEFAULT_MAP / 'manifest.json').read_text())
        next(iter(design['signals'].values()))['incoming_lanes'] = [-1, -2]
        with self.assertRaisesRegex(ValueError, 'Independent per-lane'):
            normalize_design(design)

    def test_prepare_preserves_existing_native_signals(self):
        with tempfile.TemporaryDirectory() as tmp:
            destination = Path(tmp) / 'native'
            result = prepare(DEFAULT_MAP, destination)
            self.assertEqual((destination / 'map.xodr').read_bytes(), (DEFAULT_MAP / 'map.xodr').read_bytes())
            self.assertEqual(set(result['signals']), set(json.loads((DEFAULT_MAP / 'manifest.json').read_text())['signals']))


if __name__ == '__main__':
    unittest.main()
