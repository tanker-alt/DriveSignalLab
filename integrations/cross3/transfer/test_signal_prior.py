import tempfile
import unittest
from pathlib import Path
import numpy as np
import torch
from .core import Signals,observation
from .learning import Policy

class SignalPrior(unittest.TestCase):
    def test_initial_policy_serves_each_demand_and_respects_mask(self):
        p=Policy(signal_prior='maxpressure')
        for active in range(4):
            s=Signals();s.active=active;s.stage='GREEN';s.age=6
            for desired,(road,lane) in enumerate([(2,-2),(2,-1),(1,-2),(1,-1)]):
                vs={str(i):dict(road=road,lane=lane,approach=True,speed=0.) for i in range(3)}
                obs=observation(vs,s)
                self.assertEqual(p.act(obs,True)[0],desired)
            s.age=1
            self.assertEqual(p.act(observation(vs,s),True)[0],active)
    def test_residual_is_trainable_and_checkpoint_preserves_prior(self):
        p=Policy(signal_prior='maxpressure');s=Signals();s.stage='GREEN';s.age=6
        obs=observation({},s); a,lp,v=p.act(obs)
        p.update([dict(obs=obs,action=a,logp=lp,value=v,reward=-1.,duration=1.,terminated=True)],0,1)
        self.assertGreater(float(p.light[-1].bias.abs().sum()),0)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'x.pt';p.save(path,{})
            q=Policy();q.load(path)
            self.assertEqual(q.signal_prior,'maxpressure')
            self.assertTrue(torch.equal(p.light_logits(torch.tensor(obs['light'])),q.light_logits(torch.tensor(obs['light']))))
    def test_no_hidden_action_override(self):
        p=Policy(signal_prior='maxpressure');s=Signals();s.stage='GREEN';s.age=6
        obs=observation({'a':dict(road=2,lane=-2,speed=0.,approach=True)},s)
        with torch.no_grad(): p.light[-1].bias[3]=10
        self.assertEqual(p.act(obs,True)[0],3) # A learned residual can override the fixed prior.

if __name__=='__main__':unittest.main()
