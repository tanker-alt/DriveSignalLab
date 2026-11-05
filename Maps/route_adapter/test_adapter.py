import copy
import hashlib
import json
import tempfile
import unittest
from pathlib import Path

try:
    from .adapter import DEFAULT_DATASET, read_map, load_route, validate_route, carla_locations, spawn_transform
except ImportError:
    from adapter import DEFAULT_DATASET, read_map, load_route, validate_route, carla_locations, spawn_transform


class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.folder = DEFAULT_DATASET / 't_north'
        self.raw, self.design, self.digest = read_map(self.folder)
        self.route = load_route(self.folder)

    def test_all_published_routes_match_maps_and_release_checksums(self):
        release = json.loads((DEFAULT_DATASET / 'route_release.json').read_text())
        self.assertTrue(release['passed'])
        folders = {p.name for p in DEFAULT_DATASET.iterdir() if (p/'map.xodr').is_file()}
        self.assertEqual(folders, {r['map'] for r in release['maps']})
        for row in release['maps']:
            folder=DEFAULT_DATASET/row['map']
            route=load_route(folder)
            self.assertEqual(row['opendrive_sha256'],route['opendrive_sha256'])
            self.assertEqual(row['route_sha256'],hashlib.sha256((folder/'generated_routes.json').read_bytes()).hexdigest())

    def test_rejects_discontinuity_and_missing_junction(self):
        for kind in ('gap','junction','nan','hash'):
            route=copy.deepcopy(self.route)
            if kind=='gap': route['points'][3][0]+=50
            elif kind=='junction': route['junction_order']=[]
            elif kind=='nan': route['points'][3][0]=float('nan')
            else: route['opendrive_sha256']='wrong'
            with self.assertRaises(ValueError): validate_route(route,self.design,self.digest)

    def test_rejects_changed_map_bytes(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp)
            (folder/'map.xodr').write_bytes(self.raw+b' ')
            (folder/'manifest.json').write_text(json.dumps(self.design))
            with self.assertRaises(ValueError): read_map(folder)

    def test_rejects_wrong_live_world(self):
        class WrongMap:
            def to_opendrive(self): return 'another map'
        class WrongWorld:
            def get_map(self): return WrongMap()
        with self.assertRaises(ValueError): load_route(self.folder,WrongWorld())

    def test_carla_route_consumption_and_start_transform(self):
        import carla
        map_obj=carla.Map('test', self.raw.decode('utf-8'))
        class World:
            def get_map(self): return map_obj
        points=carla_locations(self.folder,World())
        self.assertEqual(len(points),len(self.route['points']))
        transform=spawn_transform(self.folder,World())
        self.assertLess(abs(transform.location.z-points[0].z-0.3),1e-4)


if __name__=='__main__': unittest.main()
