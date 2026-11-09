"""Regression checks for physical-only placement and translated network tiles."""
import copy
import unittest
import xml.etree.ElementTree as ET
from signal_placement import apply


class SignalPlacementTests(unittest.TestCase):
    def fixture(self, arms=None, role=None):
        root=ET.fromstring('''<OpenDRIVE><road id="1" length="150"><planView><geometry s="0" x="-169" y="0" hdg="0" length="150"><line/></geometry></planView><lanes><laneSection><right><lane id="-1" type="driving"><width a="3.5"/></lane><lane id="-2" type="driving"><width a="3.5"/></lane></right></laneSection></lanes><signals><signal id="1011" s="143.45" t="-7.75"><validity fromLane="-1" toLane="-1"/><positionInertial x="-25.55" y="-7.75" z="0" hdg="0"/></signal></signals></road><controller id="1"><control signalId="1011"/></controller></OpenDRIVE>''')
        cfg=dict(road_id=1,junction_id=10,reference_s=142.85,incoming_lanes=[-1])
        if role:cfg['role']=role
        design=dict(config=dict(junction_radius_m=19),junctions=[dict(id=10,center=[0,0],arms=arms or dict(west=1,east=2,north=3,south=4))],signals={'1011':cfg})
        return root,design

    def test_logical_geometry_unchanged_and_idempotent(self):
        root,d=self.fixture();before=copy.deepcopy(root)
        apply(root,d);once=ET.tostring(root);metadata=copy.deepcopy(d)
        apply(root,d)
        self.assertEqual(once,ET.tostring(root));self.assertEqual(metadata,d)
        for tree in (before,root):
            sig=tree.find('./road/signals/signal');sig.remove(sig.find('positionInertial'))
        self.assertEqual(ET.tostring(before),ET.tostring(root))

    def test_closed_t_head_is_at_pavement_edge(self):
        root,d=self.fixture(dict(west=1,north=3,south=4));apply(root,d)
        self.assertAlmostEqual(float(root.find('./road/signals/signal/positionInertial').get('x')),7)

    def test_translated_tile_updates_manifest_without_double_shift(self):
        root,d=self.fixture();apply(root,d)
        pose=root.find('./road/signals/signal/positionInertial')
        pose.set('x',str(float(pose.get('x'))+500));pose.set('y',str(float(pose.get('y'))+350))
        before=dict(pose.attrib);apply(root,d)
        self.assertEqual(before,pose.attrib)
        self.assertEqual(before,d['signals']['1011']['physical_pose_opendrive'])

    def test_roundabout_uses_local_downstream_layout(self):
        root,d=self.fixture(role='circulating');apply(root,d)
        self.assertAlmostEqual(float(root.find('./road/signals/signal/positionInertial').get('x')),-7.55)


if __name__=='__main__':unittest.main()
