"""Public API tests; the CARLA client constructor is forbidden."""
import copy
import os
import unittest
from pathlib import Path
from unittest.mock import patch
import carla
try:
    from . import adapt, build_od_catalog, audit_route
except ImportError:
    from route_adapter import adapt, build_od_catalog, audit_route

class TopologyApiTests(unittest.TestCase):
    def test_catalog_and_corrupt_route(self):
        folder=Path(os.environ.get('ROUTE_ADAPTER_DATASET',str(Path(__file__).resolve().parent.parent/'MapDataset')))/'cross3'
        with patch.object(carla,'Client',side_effect=AssertionError('No server allowed')):
            data=build_od_catalog(folder)
            self.assertEqual(12,len(data['route_catalog']))
            row=data['route_catalog'][0]
            route=row['route']
            self.assertTrue(audit_route(folder,route,(row['destination'],row['destination_lane']))['passed'])
            self.assertFalse(audit_route(folder,list(reversed(route)))['passed'])
            bad=copy.deepcopy(route);bad[10]['x']+=100
            self.assertFalse(audit_route(folder,bad)['passed'])
            self.assertFalse(audit_route(folder,route,(999,1))['passed'])

if __name__=='__main__':unittest.main()
