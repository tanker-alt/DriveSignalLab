import unittest
from evaluation.traffic_efficiency import TrafficEfficiency

def lane(k,n):return dict(approach_id=k,queue_length_m=n,metric_status='valid',queue_vehicle_count=1,queue_head_distance_m=0,detection_length_m=50)
def exit_event(vid,line,t):return dict(vehicle_id=vid,line_id=line,estimated_crossing_time=t)

class TrafficEfficiencyTests(unittest.TestCase):
    def evaluator(self):return TrafficEfficiency([dict(id=k,kind='exit',enabled=True) for k in ('e1','e2')],['a','b'],2.)
    def test_queue_sum_not_lane_mean_and_missing_lane_not_zero(self):
        e=self.evaluator();e.update(1,0,[],[lane('a',10),lane('b',20)]);e.update(2,1,[],[lane('a',20),lane('b',30)])
        e.update(3,2,[],[lane('a',0)]);e.update(4,3,[],[lane('a',0),lane('b',0)])
        q=e.summary([])['queue'];self.assertEqual(q['value'],40);self.assertEqual(q['covered_seconds'],1)
    def test_flow_dedup_repeat_visits_and_ambiguous_exits(self):
        e=self.evaluator();e.update(1,0,[],[])
        e.update(2,1,[exit_event(1,'e1',.5)]*2,[])
        e.update(3,2,[exit_event(1,'e1',1.5),exit_event(2,'e1',1.8),exit_event(2,'e2',1.8)],[])
        f=e.summary([])['throughput'];self.assertEqual(f['value'],2);self.assertEqual(f['unique_vehicle_count'],1);self.assertEqual(f['ambiguous_excluded'],1)
        self.assertEqual(f['window_status'],'complete')
    def test_gap_does_not_invent_coverage(self):
        e=self.evaluator();e.update(1,0,[],[]);e.update(3,2,[],[])
        f=e.summary([])['throughput'];self.assertEqual(f['covered_seconds'],0);self.assertIsNone(f['complete_window_rate_veh_per_hour'])
    def test_travel_time_excludes_censored_not_zero(self):
        e=self.evaluator();t=e.summary([dict(status='paired',travel_time_s=10),dict(status='paired',travel_time_s=30),dict(status='censored',reason='experiment_end',travel_time_s=None)])['travel_time']
        self.assertEqual(t['value'],20);self.assertEqual(t['paired_count'],2);self.assertEqual(t['excluded_count'],1)
    def test_empty_geometry_unavailable(self):
        e=TrafficEfficiency();e.update(1,0,[],[]);r=e.summary([])
        self.assertIsNone(r['queue']['value']);self.assertIsNone(r['throughput']['value']);self.assertIsNone(r['travel_time']['value'])