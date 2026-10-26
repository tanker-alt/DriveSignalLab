import math
import random
import unittest
from learning import IntervalReturn, bellman, epsilon, sparse_demand, prepare_targets

class LearningTest(unittest.TestCase):
    def test_tick_partition_and_composition(self):
        a=IntervalReturn();a.add(-2.,17.)
        b=IntervalReturn()
        for _ in range(340):b.add(-2.,.05)
        self.assertAlmostEqual(a.discounted,b.discounted,12)
        left=IntervalReturn();left.add(-2.,10.)
        right=IntervalReturn();right.add(-2.,7.)
        self.assertAlmostEqual(a.discounted,left.discounted+.8*right.discounted,12)
        expected=-2*(1-.8**1.7)/(-math.log(.8))
        self.assertAlmostEqual(a.discounted,expected)

    def test_terminal_and_truncation(self):
        interval=IntervalReturn();interval.add(-1.,20.)
        terminal=interval.sample({},0,{},0,True,False)
        truncated=interval.sample({},0,{},0,False,True)
        self.assertAlmostEqual(bellman(terminal,9.),interval.discounted/20)
        self.assertAlmostEqual(bellman(truncated,9.)-bellman(terminal,9.),.64*9)
        self.assertAlmostEqual(bellman(terminal,9.,variant='legacy'),-1/20+.8*9)

    def test_sample_schedule_and_coverage(self):
        self.assertEqual(epsilon(0),.8)
        self.assertAlmostEqual(epsilon(6000),.45)
        self.assertAlmostEqual(epsilon(12000),.1)
        self.assertAlmostEqual(epsilon(100000),.1)
        combinations=set()
        for i in range(32):
            d=sparse_demand(i)[0]
            combinations.add((d['road'],d['lane'],(i//8)%4))
        self.assertEqual(len(combinations),32)

    def test_actual_target_arrays(self):
        import numpy as np
        class Net:
            def predict(self,x):return np.tile([1.,2.,3.,4.],(len(x[0]),1))
        class Agent:
            dic_agent_conf={'SAMPLE_SIZE':20,'NORMAL_FACTOR':20.}
            dic_traffic_env_conf={'LIST_STATE_FEATURE':['x']}
            q_network=Net();q_network_bar=Net()
        rows=[]
        for action,terminal in enumerate([True,False]):
            ret=IntervalReturn();ret.add(-3.,15.)
            rows.append(ret.sample({'x':[action]},action,{'x':[2]},0,terminal,not terminal))
        agent=Agent();prepare_targets(agent,rows,'smdp',random.Random(7),np)
        for i,x in enumerate(agent.Xs[0]):
            action=int(x[0]);row=rows[action]
            self.assertAlmostEqual(agent.Y[i,action],bellman(row,4.))
            other=1-action
            self.assertEqual(agent.Y[i,other],other+1)

if __name__=='__main__':unittest.main()
