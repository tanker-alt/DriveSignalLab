import copy
import json
import unittest
from pathlib import Path
from network_appearance_plan import plan
from appearance_standard import settings


class AppearancePlanTests(unittest.TestCase):
    def setUp(self):
        self.maps=Path(__file__).resolve().parents[1]/'MapDataset'
        p=self.maps/'heterogeneous_3x3_roundabout'
        self.design=json.loads((p/'manifest.json').read_text());self.raw=(p/'map.xodr').read_bytes()

    def test_mixed_counts_and_all_internal_signal_roads(self):
        result=plan(self.design,self.raw,'complete');roads=result['roads']
        self.assertEqual({r['incoming_lanes'] for r in roads},{2,3})
        self.assertEqual(result['style'],settings({'type':'cross'}))
        self.assertFalse(result['grass']);self.assertFalse(result['trees'])
        covered=[sid for r in roads for sid in r['signal_ids']]
        self.assertCountEqual(covered,self.design['signals'])
        ring=self.design['roundabouts'][0]
        circular=[r for r in roads if r['circulating']]
        self.assertEqual({r['road_id'] for r in circular},set(ring['ring_roads']+ring['ring_continuations']))
        self.assertEqual(sum(bool(r['signal_ids']) for r in circular),4)
        self.assertTrue(all(not r['crosswalk'] and not r['centerline'] for r in circular))
        self.assertTrue(result['ring_perimeter'])
        self.assertFalse(plan(self.design,self.raw,'simple')['ring_perimeter'])
        self.assertFalse(plan(self.design,self.raw,'off')['ring_perimeter'])

    def test_metadata_not_changed_and_bad_ownership_rejected(self):
        before=copy.deepcopy(self.design);plan(self.design,self.raw)
        self.assertEqual(before,self.design)
        sid=next(iter(self.design['signals']));self.design['signals'][sid]['incoming_lanes']=[-9]
        with self.assertRaisesRegex(ValueError,'ownership'):plan(self.design,self.raw)

    def test_signal_station_mismatch_rejected(self):
        rid=next(iter(self.design['signals'].values()))['road_id']
        for signal in self.design['signals'].values():
            if signal['road_id']==rid:signal['signal_s']=1
        with self.assertRaises(ValueError):plan(self.design,self.raw)

    def test_existing_network_styles_and_no_ring(self):
        for name in ['cross_3x3','t_3x3','heterogeneous_1x6','heterogeneous_3x3']:
            p=self.maps/name;d=json.loads((p/'manifest.json').read_text())
            result=plan(d,(p/'map.xodr').read_bytes())
            self.assertFalse(result['ring_perimeter'])
            self.assertFalse(any(r['circulating'] for r in result['roads']))
            self.assertCountEqual([s for r in result['roads'] for s in r['signal_ids']],d['signals'])


if __name__=='__main__':unittest.main()
