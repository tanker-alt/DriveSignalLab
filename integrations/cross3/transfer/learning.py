"""PPO for the signal actor and LMDrive's actual internal final waypoint layer.

VLA sampling records frozen upstream activations. Re-evaluating the trainable
last layer on these exact activations gives policy likelihoods without retaining
7B-model graphs. No auxiliary vehicle intent actor or imitation labels are used.
"""
import copy
import numpy as np
import torch
from torch import nn
from torch.distributions import Categorical, Normal
from .core import CONTRACT

class Policy:
    def __init__(self, seed=0, lr=3e-4, signal_prior='none'):
        torch.manual_seed(seed)
        self.light = nn.Sequential(nn.Linear(28,64),nn.Tanh(),nn.Linear(64,64),nn.Tanh(),nn.Linear(64,4))
        if signal_prior not in ('none','maxpressure'): raise ValueError('Unknown signal prior')
        self.signal_prior=signal_prior
        if signal_prior=='maxpressure':
            nn.init.zeros_(self.light[-1].weight); nn.init.zeros_(self.light[-1].bias)
        self.critic = nn.Sequential(nn.Linear(35,128),nn.Tanh(),nn.Linear(128,1))
        self.optimizer = torch.optim.Adam(list(self.light.parameters())+list(self.critic.parameters()),lr=lr)
        self.head = self.reference_head = self.head_optimizer = None
        self.std = .05
        self.updates = 0
        self.loaded_head = None
        self.live_layer = None
    def bind_head(self, layer, lr=1e-5):
        if not isinstance(layer, nn.Linear) or layer.out_features != 10:
            raise ValueError('Expected LMDrive waypoints_predictor final Linear(...,10)')
        if self.head is None:
            self.head = copy.deepcopy(layer).cpu().float()
            self.head.requires_grad_(True)
            if self.loaded_head:
                self.head.load_state_dict(self.loaded_head['head'])
            self.reference_head = copy.deepcopy(self.head).requires_grad_(False)
            self.head_optimizer = torch.optim.Adam(self.head.parameters(),lr=lr)
            if self.loaded_head:
                self.reference_head.load_state_dict(self.loaded_head['reference_head'])
                if self.loaded_head.get('head_optimizer'):
                    self.head_optimizer.load_state_dict(self.loaded_head['head_optimizer'])
        layer.load_state_dict(self.head.state_dict())
        self.live_layer = layer
    def light_logits(self, features):
        residual=self.light(features)
        if self.signal_prior=='maxpressure':
            # Explicit fixed inductive bias, not a post-sampling action override.
            # Counts were normalized by 20; 3 logits per vehicle retain exploration.
            return residual+60*(features[...,4:8]-features[...,8:12])+.001*features[...,12:16]
        return residual

    def distribution(self, obs):
        logits = self.light_logits(torch.as_tensor(obs['light'],dtype=torch.float32))
        return Categorical(logits=logits.masked_fill(~torch.as_tensor(obs['light_mask']),-1e9))
    @torch.no_grad()
    def act(self, obs, deterministic=False):
        d = self.distribution(obs)
        a = d.probs.argmax() if deterministic else d.sample()
        return int(a), float(d.log_prob(a)), float(self.critic(torch.as_tensor(obs['global_state'])).squeeze())
    def update(self, rows, bootstrap, epochs=4):
        if not rows:
            raise ValueError('No real transitions')
        values=np.array([r['value'] for r in rows]+[bootstrap]); adv=np.zeros(len(rows),np.float32)
        last=0.
        for i in reversed(range(len(rows))):
            discount=.99**rows[i]['duration']
            continuation=0. if rows[i]['terminated'] else 1.
            delta=rows[i]['reward']+discount*values[i+1]*continuation-values[i]
            last=delta+discount*(.95**rows[i]['duration'])*continuation*last
            adv[i]=last
        returns=torch.tensor(adv+values[:-1],dtype=torch.float32)
        advantage=torch.tensor(adv)
        if len(rows)>1:
            advantage=(advantage-advantage.mean())/advantage.std(unbiased=False).clamp_min(1e-6)
        obs=torch.tensor(np.stack([r['obs']['light'] for r in rows]))
        glob=torch.tensor(np.stack([r['obs']['global_state'] for r in rows]))
        masks=torch.tensor(np.stack([r['obs']['light_mask'] for r in rows]))
        acts=torch.tensor([r['action'] for r in rows]); old=torch.tensor([r['logp'] for r in rows])
        head_events=[(i,e) for i,r in enumerate(rows) for e in r.get('head_events',[])]
        head_loss=torch.tensor(0.)
        head_change=0.
        light_before = copy.deepcopy(self.light.state_dict())
        before = copy.deepcopy(self.head.state_dict()) if self.head is not None else None
        for epoch in range(epochs):
            d=Categorical(logits=self.light_logits(obs).masked_fill(~masks,-1e9))
            logratio=d.log_prob(acts)-old
            ratio=logratio.exp()
            loss=-torch.minimum(ratio*advantage,ratio.clamp(.8,1.2)*advantage).mean()
            value=self.critic(glob).squeeze(-1)
            loss=loss+.5*(value-returns).square().mean()-.01*d.entropy().mean()
            self.optimizer.zero_grad(); loss.backward()
            nn.utils.clip_grad_norm_(list(self.light.parameters())+list(self.critic.parameters()),1.)
            self.optimizer.step()
            if head_events:
                features=torch.stack([e['features'] for _,e in head_events])
                actions=torch.stack([e['action'] for _,e in head_events])
                prior=torch.tensor([e['logp'] for _,e in head_events])
                indices=torch.tensor([i for i,_ in head_events])
                mean=self.head(features)
                logp=Normal(mean,self.std).log_prob(actions).sum(-1)
                # Joint likelihood of all internal VLA samples within each signal interval.
                sums=torch.zeros(len(rows)).scatter_add(0,indices,logp-prior)
                valid=torch.zeros(len(rows),dtype=torch.bool); valid[indices]=True
                hr=sums[valid].exp(); ha=advantage[valid]
                behavior=torch.stack([e['mean'] for _,e in head_events])
                kl=((mean-behavior).square()/(2*self.std**2)).sum(-1).mean()
                if float(kl.detach()) > .03:
                    break
                anchor=(mean-self.reference_head(features)).square().mean()
                head_loss=-torch.minimum(hr*ha,hr.clamp(.8,1.2)*ha).mean()+anchor
                self.head_optimizer.zero_grad(); head_loss.backward()
                nn.utils.clip_grad_norm_(self.head.parameters(),.1); self.head_optimizer.step()
        if before:
            head_change=sum(float((v-before[k]).abs().sum()) for k,v in self.head.state_dict().items())
        if self.live_layer is not None:
            self.live_layer.load_state_dict(self.head.state_dict())
        self.updates += 1
        return dict(signal_parameter_change_l1=sum(float((v-light_before[k]).abs().sum()) for k,v in self.light.state_dict().items()),
                    loss=float(loss.detach()), head_loss=float(head_loss.detach()),
                    head_parameter_change_l1=head_change, head_samples=len(head_events), transitions=len(rows))
    def save(self, path, provenance):
        payload=dict(contract=CONTRACT,light=self.light.state_dict(),critic=self.critic.state_dict(),
                     optimizer=self.optimizer.state_dict(),updates=self.updates,provenance=provenance,std=self.std,
                     policy_spec=dict(schema='signal_residual_logits_v1',prior=self.signal_prior))
        if self.head is not None:
            payload.update(head=self.head.state_dict(),reference_head=self.reference_head.state_dict(),
                           head_optimizer=self.head_optimizer.state_dict())
        torch.save(payload,str(path))
    def load(self, path):
        data=torch.load(str(path),map_location='cpu')
        if data.get('contract') != CONTRACT:
            raise ValueError('Incompatible/missing transfer contract; legacy hash-lane checkpoints cannot be migrated faithfully')
        spec=data.get('policy_spec',dict(schema='signal_residual_logits_v1',prior='none'))
        if spec.get('schema')!='signal_residual_logits_v1' or spec.get('prior') not in ('none','maxpressure'):
            raise ValueError('Unknown signal policy semantics')
        self.signal_prior=spec['prior']
        self.light.load_state_dict(data['light']); self.critic.load_state_dict(data['critic'])
        self.optimizer.load_state_dict(data['optimizer']); self.updates=data['updates']; self.std=data['std']
        if 'head' in data: self.loaded_head=data
        return data['provenance']
