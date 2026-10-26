import unittest
from evaluation.junction_waiting import JunctionWaitingTime,aggregate_junction_waiting

def row(vid,s):return dict(vehicle_id=vid,vx=s,vy=0)
class WaitingTests(unittest.TestCase):
 def test_zero_wait_in_denominator(self):
  e=JunctionWaitingTime(10,observation_warmup_seconds=0)
  for f in range(3):e.sample(f,f,[row(1,0),row(2,1)])
  r=e.summary();self.assertEqual(r['valid_vehicle_count'],2);self.assertEqual(r['value'],1.)
 def test_gap_excludes_bridge(self):
  e=JunctionWaitingTime(10,observation_warmup_seconds=0);e.sample(1,0,[row(1,0)]);e.sample(3,2,[row(1,0)])
  self.assertEqual(e.summary()['missing_frames'],1);self.assertIsNone(e.summary()['value'])
 def test_invalid_and_unfinished_kept_out_of_mean(self):
  e=JunctionWaitingTime(10,observation_warmup_seconds=0);e.sample(1,0,[row(1,0),dict(vehicle_id=2,vx='bad',vy=0)]);e.sample(2,1,[row(1,0)])
  self.assertEqual(e.summary()['excluded_vehicle_count'],1)
 def test_multi_junction_unweighted_mean(self):
  a=dict(junction_id=1,value=2,start_sim_time=0,end_sim_time=1,config={});b=dict(junction_id=2,value=4,start_sim_time=0,end_sim_time=1,config={});self.assertEqual(aggregate_junction_waiting([a,b])['value'],3)
 def test_no_valid_is_null(self):
  self.assertIsNone(aggregate_junction_waiting([dict(junction_id=1,value=None,start_sim_time=0,end_sim_time=1,config={})])['value'])