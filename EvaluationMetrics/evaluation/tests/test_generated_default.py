import json
import shutil
import tempfile
import unittest
from pathlib import Path
from evaluation.map_profile import ROOT, read_profile, adapt_world


class GeneratedDefaultTest(unittest.TestCase):
    def test_relocated_profile_and_manifest_tampering(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)/'InformationCollector'
            shutil.copytree(str(ROOT/'map_profiles/generated_cross3'),str(root/'map_profiles/generated_cross3'))
            target=root.parent/'Maps/MapDataset/maps/cross3'
            target.mkdir(parents=True)
            shutil.copy2(str(ROOT.parent/'Maps/MapDataset/maps/cross3/manifest.json'),str(target/'manifest.json'))
            profile=read_profile(root/'map_profiles/generated_cross3/profile.json')
            self.assertEqual(profile['junction_id'],10)
            signals=profile['_design']['signals']
            self.assertEqual(len(signals),12)
            self.assertTrue(all(len(s['incoming_lanes'])==1 for s in signals.values()))
            self.assertEqual(len({s['phase_slot'] for s in signals.values()}),12)
            (target/'manifest.json').write_text('{}')
            with self.assertRaisesRegex(ValueError,'manifest changed'):
                read_profile(root/'map_profiles/generated_cross3/profile.json')

    def test_town_adapter_is_noop(self):
        world=object()
        self.assertIs(adapt_world(world,None),world)
        self.assertIs(adapt_world(world,{'junction_id':148}),world)

    def test_default_points_to_bundled_map(self):
        pointer=json.loads((ROOT/'evaluation/configs/active_map_profile.json').read_text())
        profile=read_profile(ROOT/'evaluation/configs'/pointer['profile'])
        self.assertEqual(profile['junction_id'],10)
        self.assertEqual(profile['_design']['config']['incoming_lanes'],3)
