import argparse
import hashlib
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

from evaluation.map_profile import configure_arguments, read_profile, validate_world
from Toolkits.CollectionTools.map_setup import activate, inspect_junction, prepare_profile, switch_map


class MapProfileTest(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name)
        self.fingerprint=hashlib.sha256(b'road').hexdigest()
        self.world=Mock()
        self.world.get_map.return_value=NS(name='Carla/Maps/Town04',to_opendrive=lambda:'road')
        self.world.get_traffic_lights_in_junction.return_value=[Mock()]

    def profile(self):
        inventory=dict(map_name='Carla/Maps/Town04',opendrive_sha256=self.fingerprint)
        with patch('Toolkits.CollectionTools.map_setup.inspect_junction',return_value=(
                dict(status='ready',calibration_status='reference_only'),
                dict(inventory=inventory,detection_config={'lines':[]},approach_geometry={'lanes':[]}))):
            return prepare_profile(self.world,148,self.root/'profile')

    def args(self,**kwargs):
        values=dict(profile=None,junction_id=None,host=None,port=None,
                    inventory='',detection_config='',stop_reference_config='')
        values.update(kwargs)
        return NS(**values)

    def test_prepare_activate_and_resolve_both_runtimes(self):
        path=self.profile();active=self.root/'active.json';activate(path,active)
        for controller in (True,False):
            args=self.args()
            if not controller:
                for name in ('inventory','detection_config','stop_reference_config'):
                    delattr(args,name)
            profile=configure_arguments(argparse.ArgumentParser(),args,active)
            self.assertEqual((args.junction_id,args.host,args.port),(148,'localhost',2000))
            validate_world(self.world,profile)
        self.assertTrue(Path(profile['_files']['inventory']).is_file())

    def test_tampered_file_rejected(self):
        path=self.profile()
        (path.parent/'inventory.json').write_text('{}')
        with self.assertRaisesRegex(ValueError,'changed'):
            read_profile(path)

    def test_missing_selection_and_explicit_junction(self):
        parser=argparse.ArgumentParser()
        with self.assertRaises(SystemExit):
            configure_arguments(parser,self.args(),self.root/'absent')
        active=self.root/'active.json';active.write_text('invalid')
        args=self.args(junction_id=621)
        self.assertIsNone(configure_arguments(parser,args,active))
        self.assertEqual(args.junction_id,621)

    def test_conflicting_geometry_and_junction_rejected(self):
        path=self.profile()
        for overrides in (dict(junction_id=621),dict(detection_config='other.json')):
            with self.assertRaises(SystemExit):
                configure_arguments(argparse.ArgumentParser(),self.args(profile=path,**overrides))

    def test_wrong_map_and_missing_signals_rejected(self):
        profile=read_profile(self.profile())
        self.world.get_map.return_value.to_opendrive=lambda:'changed road'
        with self.assertRaisesRegex(ValueError,'differs'):
            validate_world(self.world,profile)
        self.world.get_map.return_value.to_opendrive=lambda:'road'
        self.world.get_traffic_lights_in_junction.return_value=[]
        with self.assertRaisesRegex(ValueError,'no traffic lights'):
            validate_world(self.world,profile)

    def test_switch_same_map_no_reload_and_busy_world_refusal(self):
        client=Mock();client.get_available_maps.return_value=['/Game/Carla/Maps/Town04','/Game/Carla/Maps/Town05']
        client.get_world.return_value=self.world
        self.assertIs(switch_map(client,'Town04'),self.world)
        client.load_world.assert_not_called()
        self.world.get_settings.return_value=NS(synchronous_mode=True)
        with self.assertRaisesRegex(ValueError,'End the current experiment'):
            switch_map(client,'Town05')
        client.load_world.assert_not_called()
        with self.assertRaises(ValueError):
            switch_map(client,'Missing')

    def test_unsignalized_and_duplicate_poles_rejected(self):
        self.world.get_traffic_lights_in_junction.return_value=[]
        result,files=inspect_junction(self.world,148)
        self.assertEqual(result['status'],'unsupported');self.assertIsNone(files)
        lights=[Mock(),Mock()]
        for light in lights:
            light.get_pole_index.return_value=0
        self.world.get_traffic_lights_in_junction.return_value=lights
        result,files=inspect_junction(self.world,148)
        self.assertIn('pole_index',result['reasons'][0])

    def test_delivery_excludes_local_pointer(self):
        from Toolkits.CollectionTools.build_delivery import selected
        folder=self.root/'evaluation/configs';folder.mkdir(parents=True)
        (folder/'active_map_profile.json').write_text('{}')
        # The delivery builder now fails closed if required runtime files are absent.
        with self.assertRaises(FileNotFoundError):
            selected(self.root)
        project = (Path(__file__).resolve().parents[3] / 'InformationCollector')
        self.assertNotIn(project/'evaluation/configs/active_map_profile.json', selected(project))
