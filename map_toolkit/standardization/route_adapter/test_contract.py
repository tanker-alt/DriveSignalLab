"""Offline contracts and deliberately damaged topology, never a server."""
import copy
import hashlib
import json
import os
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import patch
import carla
import jsonschema
from . import adapt, build_od_catalog, AdapterError, lane_s_direction
from .contract import validate_opendrive
from .route_topology import LaneGraph

DATASET = Path(os.environ.get('ROUTE_ADAPTER_DATASET', str(Path(__file__).parent.parent/'MapDataset')))


class ContractTests(unittest.TestCase):
    def setUp(self):
        self.client = patch.object(carla, 'Client', side_effect=AssertionError('No CARLA Server'))
        self.client.start(); self.addCleanup(self.client.stop)

    def test_schema_and_od_identity_on_all_maps(self):
        schema = json.loads((Path(__file__).parent/'output.schema.json').read_text())
        jsonschema.Draft7Validator.check_schema(schema)
        folders = sorted(p for p in DATASET.iterdir() if (p/'map.xodr').is_file())
        self.assertTrue(folders)
        for folder in folders:
            with self.subTest(map=folder.name):
                result = build_od_catalog(folder)
                jsonschema.validate(result, schema)
                ids = [r['route_id'] for r in result['route_catalog']]
                self.assertEqual(len(ids), len(set(ids)))
                self.assertEqual(ids, [r['route_id'] for r in result['topology_checks']])
                self.assertEqual(ids, [r['route_id'] for r in result['od_connections']])
                for row in result['route_catalog']:
                    first, last = row['route'][0], row['route'][-1]
                    self.assertEqual((row['origin'], row['lane']), (first['road_id'], first['lane_id']))
                    self.assertEqual((row['destination'], row['destination_lane']), (last['road_id'], last['lane_id']))

    def test_x_t_r_counts_and_determinism(self):
        for name, count in [('cross3', 12), ('t_north', 6), ('roundabout4_8groups', 32)]:
            first = build_od_catalog(DATASET/name)
            self.assertEqual(count, len(first['route_catalog']))
            self.assertEqual(first, build_od_catalog(DATASET/name))

    def test_damaged_connectors_rejected_before_native_map(self):
        raw = (DATASET/'cross3/map.xodr').read_bytes()
        for attribute, value, code in [('connectingRoad', '999999', 'unresolved_connector'), ('lane', '999999', 'unresolved_connector_lane')]:
            root = ET.fromstring(raw)
            connection = root.find('junction/connection')
            if attribute == 'lane':
                connection.find('laneLink').set('to', value)
            else:
                connection.set(attribute, value)
            with self.assertRaises(AdapterError) as caught:
                validate_opendrive(ET.tostring(root))
            self.assertEqual(code, caught.exception.code)

    def test_signal_and_stop_corruption_is_not_silent(self):
        source = DATASET/'cross3'
        for defect, code in [('signal', 'signal_mapping_unresolved'), ('stop_hash', 'stop_line_map_hash_mismatch'), ('stop_lane', 'stop_lane_unresolved')]:
            with tempfile.TemporaryDirectory() as temp:
                folder = Path(temp)
                for name in ('map.xodr', 'manifest.json', 'safety_rules.json'):
                    (folder/name).write_bytes((source/name).read_bytes())
                name = 'manifest.json' if defect=='signal' else 'safety_rules.json'
                data = json.loads((folder/name).read_text())
                if defect=='signal':
                    data['signals']['999999'] = data['signals'].pop(next(iter(data['signals'])))
                elif defect=='stop_hash': data['opendrive_sha256'] = '0'*64
                else: data['stop_lines'][0]['lane_id'] = 9999
                (folder/name).write_text(json.dumps(data))
                result = adapt(folder)
                self.assertFalse(result['passed'])
                self.assertIn(code, [d['code'] for d in result['diagnostics']])
                with self.assertRaises(AdapterError): build_od_catalog(folder)

    def test_closed_graph_and_short_boundary_have_explicit_errors(self):
        graph = LaneGraph.__new__(LaneGraph)
        graph.entries = []; graph.exits = []
        with self.assertRaises(AdapterError) as caught: graph.catalog()
        self.assertEqual('no_boundary_od', caught.exception.code)
        from types import SimpleNamespace as N
        lane = (1, 0, -1)
        graph.entries = [lane]; graph.exits = [lane]
        graph.segments = {lane: (N(s=.00001), N(s=20))}; graph.edges = {lane: set()}
        with self.assertRaises(AdapterError) as caught: graph.catalog()
        self.assertEqual('boundary_road_too_short', caught.exception.code)

    def test_cycle_without_reachable_exit_terminates(self):
        graph = LaneGraph.__new__(LaneGraph)
        a,b,c = (1,0,-1),(2,0,-1),(3,0,-1)
        graph.entries=[a]; graph.exits=[c]; graph.segments={a:None,b:None,c:None}
        graph.edges={a:{b},b:{a},c:set()}
        with self.assertRaises(AdapterError) as caught: graph.catalog()
        self.assertEqual('unreachable_boundary_exit', caught.exception.code)

    def test_endpoint_direction_and_invalid_route_metadata(self):
        raw=(DATASET/'cross3/map.xodr').read_text(encoding='utf-8-sig')
        road_map=carla.Map('offline',raw)
        self.assertEqual(1,lane_s_direction(road_map,1,-1,149.99999))
        self.assertEqual(-1,lane_s_direction(road_map,1,1,149.99999))
        graph=LaneGraph(road_map)
        route=graph.catalog()[0]['route']
        bad=copy.deepcopy(route); del bad[1]['lane_id']
        self.assertFalse(graph.audit(bad)['passed'])
        bad=[p for p in route if p['road_id']<100]
        self.assertFalse(graph.audit(bad)['passed'])

    def test_short_internal_connector_is_not_skipped(self):
        roads=[]; position=0.
        for rid,length in ((1,40.),(2,.05),(3,40.)):
            links=''
            lane_links=''
            if rid>1:
                links+='<predecessor elementType="road" elementId="{}" contactPoint="end"/>'.format(rid-1)
                lane_links+='<predecessor id="-1"/>'
            if rid<3:
                links+='<successor elementType="road" elementId="{}" contactPoint="start"/>'.format(rid+1)
                lane_links+='<successor id="-1"/>'
            roads.append('''<road name="test" length="{length}" id="{rid}" junction="-1">
              <link>{links}</link><planView><geometry s="0" x="{x}" y="0" hdg="0" length="{length}"><line/></geometry></planView>
              <lanes><laneSection s="0"><center><lane id="0" type="none" level="false"/></center>
              <right><lane id="-1" type="driving" level="false"><link>{lane_links}</link>
              <width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane></right></laneSection></lanes></road>'''.format(
                  rid=rid,length=length,links=links,lane_links=lane_links,x=position))
            position+=length
        raw='<OpenDRIVE><header revMajor="1" revMinor="4" name="tiny" version="1" date="" north="0" south="0" east="0" west="0"/>'+''.join(roads)+'</OpenDRIVE>'
        graph=LaneGraph(carla.Map('short_connector',raw))
        rows=graph.catalog()
        self.assertEqual(1,len(rows))
        self.assertEqual({1,2,3},{p['road_id'] for p in rows[0]['route']})
        self.assertTrue(graph.audit(rows[0]['route'],(3,-1))['passed'])

    def test_matrix_records_native_failure_and_timeout(self):
        import subprocess
        from .compatibility import run
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp); dataset=root/'dataset'; dataset.mkdir()
            for name in ('a','b'):
                folder=dataset/name; folder.mkdir(); (folder/'map.xodr').write_text('fixture')
            output=root/'report.json'
            with patch('subprocess.run',side_effect=[subprocess.CompletedProcess([], -1, '', 'native crash'),
                                                    subprocess.TimeoutExpired([], 1)]):
                result=run(dataset,output,1)
            self.assertTrue(result['completed']); self.assertFalse(result['passed'])
            self.assertEqual(['worker_failure','timeout'],[r['failure_kind'] for r in result['maps']])
            self.assertEqual(result,json.loads(output.read_text()))


if __name__ == '__main__': unittest.main()
