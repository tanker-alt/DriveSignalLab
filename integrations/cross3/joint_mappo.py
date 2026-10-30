"""Compact MAPPO implementation for the CARLA/CoTV-style joint interface.

It is deliberately simulator-agnostic: CARLA integration supplies transitions,
while this module handles masked categorical actors, GAE and centralized PPO.
"""
from dataclasses import dataclass
from typing import Optional
import torch
from torch import nn
from .vehicle_intent_policy import VehicleIntentActor, INTENTS

class LightActor(nn.Module):
    def __init__(self, obs_dim=80, hidden=64, actions=4):
        super().__init__(); self.net=nn.Sequential(nn.Linear(obs_dim,hidden),nn.Tanh(),nn.Linear(hidden,hidden),nn.Tanh(),nn.Linear(hidden,actions))
    def forward(self, obs, mask=None):
        logits=self.net(obs)
        if mask is not None: logits=logits.masked_fill(~mask.bool(),-1e9)
        return torch.distributions.Categorical(logits=logits)

class CentralCritic(nn.Module):
    def __init__(self, dim, hidden=128):
        super().__init__(); self.net=nn.Sequential(nn.Linear(dim,hidden),nn.Tanh(),nn.Linear(hidden,hidden),nn.Tanh(),nn.Linear(hidden,1))
    def forward(self,x): return self.net(x).squeeze(-1)

@dataclass
class Rollout:
    light_obs: torch.Tensor; light_act: torch.Tensor; light_old_logp: torch.Tensor
    vehicle_obs: torch.Tensor; vehicle_act: torch.Tensor; vehicle_old_logp: torch.Tensor
    global_obs: torch.Tensor; reward: torch.Tensor; done: torch.Tensor
    value: torch.Tensor; advantage: torch.Tensor; ret: torch.Tensor

def gae(reward, value, done, gamma=.99, lam=.95):
    adv=torch.zeros_like(reward); last=torch.zeros((), device=reward.device)
    for t in range(len(reward)-1,-1,-1):
        nxt=0 if t==len(reward)-1 else value[t+1]
        delta=reward[t]+gamma*nxt*(1-done[t])-value[t]
        last=delta+gamma*lam*(1-done[t])*last; adv[t]=last
    return adv, adv+value

class JointMAPPO:
    def __init__(self, light_dim=80, vehicle_dim=12, global_dim=92, lr=3e-4):
        self.light=LightActor(light_dim); self.vehicle=VehicleIntentActor(vehicle_dim); self.critic=CentralCritic(global_dim)
        self.opt=torch.optim.Adam(list(self.light.parameters())+list(self.vehicle.parameters())+list(self.critic.parameters()),lr=lr)
    @torch.no_grad()
    def act(self, light_obs, vehicle_obs, global_obs, light_mask=None, vehicle_mask=None, deterministic=False):
        ld=self.light(light_obs,light_mask); vd=self.vehicle(vehicle_obs,vehicle_mask)
        la=ld.probs.argmax(-1) if deterministic else ld.sample(); va=vd.probs.argmax(-1) if deterministic else vd.sample()
        return la,va,ld.log_prob(la),vd.log_prob(va),self.critic(global_obs)
    def update(self, ro: Rollout, epochs=10, clip=.2, entropy=.01):
        adv=(ro.advantage-ro.advantage.mean())/(ro.advantage.std()+1e-8); metrics={}
        for _ in range(epochs):
            ld=self.light(ro.light_obs); vd=self.vehicle(ro.vehicle_obs); val=self.critic(ro.global_obs)
            lr=torch.exp(ld.log_prob(ro.light_act)-ro.light_old_logp); vr=torch.exp(vd.log_prob(ro.vehicle_act)-ro.vehicle_old_logp)
            def ppo_ratio(r): return -torch.min(r*adv,torch.clamp(r,1-clip,1+clip)*adv).mean()
            loss=ppo_ratio(lr)+ppo_ratio(vr)+.5*(val-ro.ret).pow(2).mean()-entropy*(ld.entropy().mean()+vd.entropy().mean())
            self.opt.zero_grad(); loss.backward(); nn.utils.clip_grad_norm_(list(self.light.parameters())+list(self.vehicle.parameters())+list(self.critic.parameters()),1.0); self.opt.step()
        metrics.update(loss=float(loss.detach()), value_loss=float((val-ro.ret).pow(2).mean().detach()), entropy=float((ld.entropy().mean()+vd.entropy().mean()).detach()))
        return metrics
