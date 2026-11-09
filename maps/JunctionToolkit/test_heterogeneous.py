import copy
import math
import unittest
import xml.etree.ElementTree as ET
from heterogeneous import sample, bridge, ring_position, build
from transition_sidewalks import add_edges


class HeterogeneousTests(unittest.TestCase):
    def test_optional_ring_and_invalid_positions(self):
        self.assertIsNone(ring_position({},3,3))
        self.assertEqual(ring_position(dict(include_roundabout=True),3,3),4)
        cells,_,stats=sample(dict(include_roundabout=True,rows=3,columns=3))
        self.assertEqual(cells[4]['type'],'roundabout')
        self.assertEqual(sum(c['type']=='roundabout' for c in cells),1)
        self.assertEqual(stats['proposal']['t'],.5)
        self.assertEqual(ring_position(dict(include_roundabout=True,roundabout_position=[0,1]),3,3),1)
        for cfg in [dict(include_roundabout=1),dict(roundabout_position=[0,0]),dict(include_roundabout=True,roundabout_position=[3,0])]:
            with self.assertRaises(ValueError):ring_position(cfg,3,3)
        cells[0]={'type':'roundabout'}
        with self.assertRaises(ValueError):sample(dict(include_roundabout=True,layout=cells))

    def test_generated_ring_reproducibility_and_broken_sidewalk(self):
        import tempfile,json,hashlib
        from pathlib import Path
        from validate_heterogeneous_static import validate
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parents[1]/'maps') as temp:
            a=Path(temp)/'a';b=Path(temp)/'b'
            design=build(dict(name='test',include_roundabout=True),a)
            self.assertTrue(validate(a)['passed'])
            ring=design['roundabouts'][0]
            self.assertEqual(len(ring['sidewalk_connections']),16)
            xml=ET.parse(a/'map.xodr')
            for rid in ring['ring_roads']:
                self.assertEqual(len(xml.findall("./road[@id='%s']/lanes/laneSection/right/lane[@type='sidewalk']"%rid)),2)
            for rid in ring['ring_continuations']:
                self.assertFalse(xml.findall("./road[@id='%s']/lanes/laneSection/*/lane[@type='sidewalk']"%rid),'Curbs must not block access mouths')
            build(design['config'],b)
            self.assertEqual((a/'map.xodr').read_bytes(),(b/'map.xodr').read_bytes())
            root=ET.parse(a/'map.xodr');rid=design['seams'][0]['connector_road']
            link=root.find("./road[@id='%s']/lanes/laneSection/left/lane[@type='sidewalk']/link"%rid)
            link.remove(link.find('predecessor'));root.write(a/'map.xodr')
            design['opendrive_sha256']=hashlib.sha256((a/'map.xodr').read_bytes()).hexdigest()
            (a/'manifest.json').write_text(json.dumps(design))
            with self.assertRaisesRegex(ValueError,'Unlinked sidewalk'):validate(a)

    def test_seed_and_saved_layout_are_reproducible(self):
        cells,design,stats=sample(dict(rows=3,columns=3,seed=42))
        self.assertEqual(cells,sample(dict(rows=3,columns=3,seed=42))[0])
        self.assertEqual(cells,sample(dict(rows=3,columns=3,seed=999,layout=cells))[0])
        self.assertEqual({c['type'] for c in cells},{'t','cross'})
        self.assertEqual(stats['proposal']['t'],.5)
        self.assertEqual(set(stats['proposal']['missing_directions'].values()),{.25})

    def test_fixed_disconnected_layout_is_not_silently_resampled(self):
        with self.assertRaises(ValueError):
            sample(dict(rows=1,columns=2,layout=[dict(type='t',missing_arm='east'),dict(type='cross')]))

    def test_taper_has_matching_endpoints_not_a_width_step(self):
        from scenariogeneration import xodr
        for na,nb in ((2,3),(3,2)):
            root=ET.Element('OpenDRIVE')
            for rid,n,x,h in ((1,na,0,math.pi),(2,nb,160,0)):
                r=xodr.create_road(xodr.Line(150),rid,left_lanes=n,right_lanes=n,lane_width=3.5)
                r.planview.set_start_point(x,0,h);r.planview.adjust_geometries()
                xml=r.get_element();add_edges(xml);root.append(xml)
            a,b=root.findall('road');bridge(root,a,b,900000,na,nb,3.5)
            r=root.findall('road')[-1];sections=r.findall('./lanes/laneSection')
            self.assertEqual(len(sections[0].findall("./right/lane[@type='driving']")),na)
            self.assertEqual(len(sections[-1].findall("./right/lane[@type='driving']")),nb)
            middle=sections[1];poly=middle.find("./right/lane[@id='-3']/width")
            coeff=[float(poly.get(k)) for k in ('a','b','c','d')];length=120.
            end=sum(v*length**i for i,v in enumerate(coeff))
            self.assertAlmostEqual(coeff[0],0 if na==2 else 3.5)
            self.assertAlmostEqual(end,3.5 if nb==3 else 0)
            self.assertAlmostEqual(coeff[1],0)
            self.assertAlmostEqual(coeff[1]+2*coeff[2]*length+3*coeff[3]*length**2,0)
            self.assertEqual(a.find('./link/predecessor').get('contactPoint'),'start')
            self.assertEqual(b.find('./link/predecessor').get('contactPoint'),'end')


if __name__=='__main__':unittest.main()
