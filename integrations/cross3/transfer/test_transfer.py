import copy
import tempfile
from pathlib import Path
import unittest
import numpy as np
import torch
from .core import Metrics,Signals,observation
from .learning import Policy

class ContractTests(unittest.TestCase):
    def test_censored_and_pending_demand(self):
        m=Metrics([dict(id='a',road=1,time=0),dict(id='b',road=2,time=.5),dict(id='future',road=3,time=5)])
        v={'a':dict(speed=2)}
        m.sample(1,1,v,['a'])
        m.sample(2,1,{})
        r=m.report()
        self.assertEqual(r['due'],2); self.assertEqual(r['unfinished'],1)
        self.assertAlmostEqual(r['censored_vehicle_time_s'],2.5)
        self.assertAlmostEqual(r['waiting_vehicle_seconds'],1.5)
        self.assertEqual(r['completed_travel_time_mean_s'],1)
        self.assertIsNone(r['red_light_violations'])
    def test_ttc_closing_only_and_duration(self):
        m=Metrics([dict(id='a',road=1,time=0)])
        m.sample(.5,.5,{'a':dict(speed=10,leader_speed=5,gap=5)})
        m.sample(1,.5,{'a':dict(speed=5,leader_speed=10,gap=5)})
        self.assertEqual(m.report()['rear_end_min_ttc_s'],1)
        self.assertEqual(m.report()['rear_end_ttc_below_1_5_vehicle_seconds'],.5)
    def test_interlock_mask_and_max_green(self):
        s=Signals(); s.tick(2,True); self.assertEqual(s.stage,'RED')
        s.tick(.1,False); self.assertEqual(s.stage,'GREEN')
        with self.assertRaises(ValueError): s.request(1,[1]*4)
        s.tick(5); s.request(1,[1]*4)
        s.tick(3); self.assertEqual(s.stage,'RED')
        s.tick(3,True); self.assertEqual(s.stage,'RED')
        s.tick(.1); self.assertEqual(s.active,1)
        s.tick(20); self.assertFalse(s.mask([1]*4)[1])
    def test_phase_groups_and_counts(self):
        s=Signals()
        obs=observation({'a':dict(road=2,lane=-2,approach=True,speed=0),
                         'b':dict(road=1,lane=-1,approach=False,outgoing=True,speed=4)},s)
        self.assertEqual(obs['light'].shape,(28,)); self.assertEqual(obs['global_state'].shape,(35,))
        self.assertAlmostEqual(obs['light'][0],.05); self.assertAlmostEqual(obs['light'][11],.05)
    def test_real_internal_head_updates_and_backbone_unchanged(self):
        p=Policy(2); layer=torch.nn.Linear(8,10); p.bind_head(layer)
        backbone=torch.nn.Linear(4,8).requires_grad_(False)
        before=copy.deepcopy(backbone.state_dict()); rows=[]
        signals=Signals(); signals.tick(2); signals.tick(6)
        obs=observation({},signals)
        for i in range(5):
            a,lp,value=p.act(obs)
            features=backbone(torch.randn(4)).detach()
            mean=p.head(features).detach(); dist=torch.distributions.Normal(mean,p.std)
            act=dist.sample()
            rows.append(dict(obs=obs,action=a,logp=lp,value=value,reward=-float(i),duration=1.,terminated=i==4,
                head_events=[dict(features=features,mean=mean,action=act,logp=float(dist.log_prob(act).sum()))]))
        result=p.update(rows,0,epochs=1)
        self.assertGreater(result['head_parameter_change_l1'],0)
        self.assertTrue(all(torch.equal(v,before[k]) for k,v in backbone.state_dict().items()))
        self.assertTrue(all(torch.isfinite(v).all() for v in p.head.state_dict().values()))
    def test_one_sample_and_timeout_bootstrap(self):
        p=Policy(); obs=observation({},Signals()); a,lp,v=p.act(obs)
        result=p.update([dict(obs=obs,action=a,logp=lp,value=v,reward=-1.,duration=.5,terminated=False)],2,1)
        self.assertTrue(np.isfinite(result['loss']))

    def test_checkpoint_resume_and_legacy_rejection(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'policy.pt'
            p=Policy(); p.bind_head(torch.nn.Linear(8,10)); p.save(path,{'backend':'carla'})
            restored=Policy(99); restored.load(path); layer=torch.nn.Linear(8,10)
            restored.bind_head(layer)
            self.assertTrue(torch.equal(p.head.weight,layer.weight))
            torch.save({'light':{}},str(path))
            with self.assertRaisesRegex(ValueError,'contract'): restored.load(path)
    def test_stop_debounce_and_tick_invariance(self):
        reports=[]
        for dt in (.1,.05):
            m=Metrics([dict(id='a',road=1,time=0)])
            for i in range(round(2/dt)):
                m.sample((i+1)*dt,dt,{'a':dict(speed=0)})
            reports.append(m.report())
        self.assertEqual(reports[0]['stops'],1)
        self.assertEqual(reports[1]['stops'],1)
        self.assertAlmostEqual(reports[0]['waiting_vehicle_seconds'],reports[1]['waiting_vehicle_seconds'])

if __name__=='__main__': unittest.main()
