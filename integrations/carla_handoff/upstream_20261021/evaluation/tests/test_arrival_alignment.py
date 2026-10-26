import copy
import unittest
from evaluation.topology import approach_arrivals
from evaluation.detection_lines import DetectionLines


class ArrivalAlignmentTest(unittest.TestCase):
    def geometry(self):
        return dict(junction_id=1,lanes=[dict(id="2_0_-1",geometry_status="upstream_end_or_branch",
            samples=[dict(x=x,y=0,z=0,s=-x,width=4,road_id=2,section_id=0,lane_id=-1)
                     for x in (0,-41,-42)])])

    def test_actual_truncated_boundary_and_downstream_direction(self):
        lines=approach_arrivals(self.geometry())
        self.assertEqual(lines[0]["center_xy"],[-42,0])
        self.assertEqual(lines[0]["normal_xy"],[1,0])
        self.assertEqual(lines[0]["actual_detection_length_m"],42)
        DetectionLines(lines)

    def test_empty_and_ambiguous_geometry_rejected(self):
        geometry=self.geometry()
        geometry["lanes"][0]["samples"]=geometry["lanes"][0]["samples"][:1]
        with self.assertRaises(ValueError):approach_arrivals(geometry)
        geometry=self.geometry()
        other=copy.deepcopy(geometry["lanes"][0]);other["id"]="3_0_-1"
        geometry["lanes"].append(other)
        with self.assertRaises(ValueError):approach_arrivals(geometry)
