"""Run with carla-env; checks actual crossing evidence, without a server."""
import copy
import unittest
from heterogeneous_route import audit


class CrossingAuditTests(unittest.TestCase):
    def setUp(self):
        self.design=dict(seams=[dict(road_a=1,road_b=2,connector_road=900)],
                         roundabouts=[dict(cell_id=10,ring_roads=[20,21,22,23])])
        self.route=dict(movements=[dict(target_road=1),dict(road=2)],ring_sequences={'10':[20,21,22,23,20]})
        self.report=dict(road_transitions=[dict(road=r) for r in [1,900,2,20,21,22,23,20]])

    def test_full_lap_and_bridge_pass(self):
        self.assertTrue(audit(self.report,self.route,self.design))

    def test_missing_access_or_bridge_fails(self):
        for missing in (22,900):
            report=copy.deepcopy(self.report)
            report['road_transitions']=[r for r in report['road_transitions'] if r['road']!=missing]
            self.assertFalse(audit(report,self.route,self.design))

    def test_incomplete_lap_fails(self):
        self.report['road_transitions'].pop()
        self.assertFalse(audit(self.report,self.route,self.design))


if __name__=='__main__':unittest.main()
