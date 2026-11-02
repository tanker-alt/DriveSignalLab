"""Minimal continuous-action MAPPO used for the SUMO first stage."""
from __future__ import annotations
from dataclasses import dataclass
import torch
from torch import nn
from torch.distributions import Categorical, Normal
class Actor(nn.Module):
    def __init__(self, dim, out, continuous=False):
        super().__init__(); self.continuous=continuous; self.body=nn.Sequential(nn.Linear(dim,64),nn.Tanh(),nn.Linear(64,64),nn.Tanh()); self.head=nn.Linear(64,out); self.log_std=nn.Parameter(torch.zeros(out)) if continuous else None
    def forward(self,x):
        z=self.head(self.body(x)); return Normal(z, self.log_std.exp().expand_as(z)) if self.continuous else Categorical(logits=z)
class Critic(nn.Module):
    def __init__(self, dim): super().__init__(); self.net=nn.Sequential(nn.Linear(dim,128),nn.Tanh(),nn.Linear(128,128),nn.Tanh(),nn.Linear(128,1))
    def forward(self,x): return self.net(x).squeeze(-1)
@dataclass
class Batch:
    lo: torch.Tensor; la: torch.Tensor; ll: torch.Tensor; vo: torch.Tensor; va: torch.Tensor; vl: torch.Tensor; glob: torch.Tensor; ret: torch.Tensor; adv: torch.Tensor
class ContinuousMAPPO:
    def __init__(self, light_dim=13, vehicle_dim=6, global_dim=19, lr=3e-4):
        self.light=Actor(light_dim,4); self.vehicle=Actor(vehicle_dim,1,True); self.critic=Critic(global_dim); self.opt=torch.optim.Adam(list(self.light.parameters())+list(self.vehicle.parameters())+list(self.critic.parameters()),lr=lr)
    @torch.no_grad()
    def act(self, lo, vo, glob, deterministic=False):
        ld=self.light(lo); vd=self.vehicle(vo); la=ld.probs.argmax() if deterministic else ld.sample(); raw=vd.mean if deterministic else vd.sample(); va=2.5*torch.tanh(raw); return la, va.squeeze(-1), ld.log_prob(la), vd.log_prob(raw).sum(-1), self.critic(glob)
    def update(self,batch,epochs=10,clip=.2,entropy=.01):
        adv=(batch.adv-batch.adv.mean())/(batch.adv.std()+1e-8)
        for _ in range(epochs):
            ld=self.light(batch.lo); vd=self.vehicle(batch.vo); val=self.critic(batch.glob); raw=torch.atanh(torch.clamp(batch.va/2.5,-.999,.999)); lratio=torch.exp(ld.log_prob(batch.la)-batch.ll); vratio=torch.exp(vd.log_prob(raw).sum(-1)-batch.vl)
            p=lambda r: -torch.min(r*adv,torch.clamp(r,1-clip,1+clip)*adv).mean(); loss=p(lratio)+p(vratio)+.5*(val-batch.ret).pow(2).mean()-entropy*(ld.entropy().mean()+vd.entropy().sum(-1).mean())
            self.opt.zero_grad(); loss.backward(); nn.utils.clip_grad_norm_(list(self.light.parameters())+list(self.vehicle.parameters())+list(self.critic.parameters()),1.0); self.opt.step()
        return {'loss':float(loss.detach()),'value_loss':float((val-batch.ret).pow(2).mean().detach())}
