"""Offline regression gates for shared stitches and multi-junction planning."""
import copy
import math
import unittest
import xml.etree.ElementTree as ET
from network_seams import stitch
from network_itinerary import itinerary


def road(rid, heading):
    r=ET.Element('road',id=str(rid))
    link=ET.SubElement(r,'link')
    ET.SubElement(link,'successor',elementType='junction',elementId=str(rid+10))
    p=ET.SubElement(r,'planView');ET.SubElement(p,'geometry',x='0',y='0',hdg=str(heading))
    section=ET.SubElement(ET.SubElement(r,'lanes'),'laneSection')
    for side,lid in [('left',1),('right',-1)]:
        ET.SubElement(ET.SubElement(section,side),'lane',id=str(lid),type='driving')
    return r


class Connectivity(unittest.TestCase):
    def test_reciprocal_start_links_preserve_junction_and_are_idempotent(self):
        a,b=road(1,0),road(2,math.pi)
        stitch(a,b);first=ET.tostring(a);stitch(a,b)
        self.assertEqual(first,ET.tostring(a))
        for src,dst in ((a,b),(b,a)):
            self.assertEqual(src.find('./link/predecessor').get('elementId'),dst.get('id'))
            self.assertEqual(src.find('./link/successor').get('elementType'),'junction')
            for lane in src.findall('./lanes/laneSection/*/lane'):
                self.assertEqual(int(lane.find('./link/predecessor').get('id')),-int(lane.get('id')))
                self.assertIsNone(lane.find('./link/successor'))

    def test_displaced_or_mismatched_seams_rejected(self):
        a,b=road(1,0),road(2,math.pi)
        bad=copy.deepcopy(b);bad.find('./planView/geometry').set('x','2')
        with self.assertRaises(ValueError):stitch(a,bad)
        bad=copy.deepcopy(b);bad.find('./lanes/laneSection/left/lane').set('id','2')
        with self.assertRaises(ValueError):stitch(a,bad)

    def test_route_requires_legal_movements_not_just_adjacency(self):
        design=dict(junctions=[dict(id=10),dict(id=20)],seams=[dict(road_a=2,road_b=3)],movements=[
            dict(road=1,target_road=2,junction_id=10,movement='straight',lane=-1),
            dict(road=3,target_road=4,junction_id=20,movement='straight',lane=-1)])
        self.assertEqual([v['junction_id'] for v in itinerary(design)],[10,20])
        design['movements'][1]['road']=5
        with self.assertRaises(ValueError):itinerary(design)


if __name__=='__main__':unittest.main()
