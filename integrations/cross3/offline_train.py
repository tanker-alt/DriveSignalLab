#!/usr/bin/env python3
"""Fast offline PressLight pretraining with the official Cross3 80-D state."""
import argparse,json,random
from collections import deque
from pathlib import Path
import numpy as np, torch
from torch import nn
PHASE_BITS=np.asarray([[0,0,0,0,0,1,0,1],[0,0,0,0,1,0,1,0],[0,1,0,1,0,0,0,0],[1,0,1,0,0,0,0,0]],dtype='float32'); LANES=12; BINS=3; STATE=80
class Net(nn.Module):
 def __init__(self):
  super().__init__(); self.net=nn.Sequential(nn.Linear(STATE,20),nn.ReLU(),nn.Linear(20,20),nn.ReLU(),nn.Linear(20,4))
 def forward(self,x): return self.net(x)
def encode(p,i,o): return np.r_[PHASE_BITS[p],np.asarray(i,dtype='float32').reshape(-1),np.asarray(o,dtype='float32').reshape(-1)].astype('float32')
def rates(r):
 base=r.uniform(.04,.22); x=np.full(LANES,base); k=int(r.integers(4)); x[k*3:k*3+3]*=r.uniform(1.4,2.5); return np.clip(x,.01,.55)
def env(r): return {'phase':0,'inc':r.integers(0,3,(LANES,BINS)).astype('float32'),'out':np.zeros((LANES,BINS),dtype='float32')}
def transition(e,d,a,r,decision=10,yellow=3,clear=2,minimum=5):
 blocked=yellow+clear if a!=e['phase'] else 0; duration=minimum if blocked else decision; rew=0.
 for t in range(blocked+duration):
  if t>=blocked:
   for lane in range(a*3,a*3+3):
    n=min(int(e['inc'][lane].sum()),int(r.poisson(1.8)))
    for b in range(3):
     z=min(int(e['inc'][lane,b]),n); e['inc'][lane,b]-=z; e['out'][lane,b]+=z; n-=z
     if not n: break
  e['out'][:]=np.maximum(0,e['out']-r.binomial(1,.35,e['out'].shape)); e['inc'][:]=np.minimum(20,e['inc']+r.binomial(1,d[:,None],(LANES,BINS))); rew+=-.25*float(e['inc'].sum())
 e['phase']=a; return rew,blocked+duration
def rollout(policy,seed,horizon=180):
 r=np.random.default_rng(seed); e=env(r); d=rates(r); total=sw=elapsed=0
 while elapsed<horizon:
  a=policy(e); sw+=a!=e['phase']; x,dt=transition(e,d,a,r); total+=x; elapsed+=dt
 return {'cost':-total,'remaining':float(e['inc'].sum()+e['out'].sum()),'switches':sw}
def main():
 p=argparse.ArgumentParser(); p.add_argument('--output',type=Path,required=True); p.add_argument('--steps',type=int,default=100000); p.add_argument('--seed',type=int,default=2026); a=p.parse_args(); random.seed(a.seed); np.random.seed(a.seed); torch.manual_seed(a.seed); r=np.random.default_rng(a.seed)
 net,target=Net(),Net(); target.load_state_dict(net.state_dict()); opt=torch.optim.RMSprop(net.parameters(),lr=3e-4); mem=deque(maxlen=20000); e=env(r); d=rates(r); last=0.
 for step in range(a.steps):
  if step%300==0: d=rates(r); e=env(r)
  s=encode(e['phase'],e['inc'],e['out']); eps=max(.2,.8*.95**(step/3000));
  with torch.no_grad(): greedy=int(net(torch.from_numpy(s)).argmax())
  act=int(r.integers(4)) if r.random()<eps else greedy; rew,dt=transition(e,d,act,r); ns=encode(e['phase'],e['inc'],e['out']); mem.append((s,act,rew,ns,dt))
  if len(mem)>=512:
   b=random.sample(mem,128); ss=torch.from_numpy(np.stack([x[0] for x in b])); aa=torch.tensor([x[1] for x in b]); rr=torch.tensor([x[2] for x in b],dtype=torch.float32); nnx=torch.from_numpy(np.stack([x[3] for x in b])); dd=torch.tensor([x[4] for x in b],dtype=torch.float32); pred=net(ss).gather(1,aa[:,None]).squeeze(1)
   with torch.no_grad(): y=rr+torch.pow(torch.tensor(.8),dd/10.)*target(nnx).max(1).values
   loss=nn.functional.mse_loss(pred,y); opt.zero_grad(); loss.backward(); nn.utils.clip_grad_norm_(net.parameters(),5); opt.step(); last=float(loss)
  if step%1000==0: target.load_state_dict(net.state_dict())
 def learned(x): return int(net(torch.from_numpy(encode(x['phase'],x['inc'],x['out']))).argmax())
 def fixed(x): return (x['phase']+1)%4
 def pressure(x): return int(np.argmax([x['inc'][i*3:(i+1)*3].sum()-x['out'][i*3:(i+1)*3].sum() for i in range(4)]))
 ev={};
 for name,fn in [('fixed',fixed),('maxpressure',pressure),('presslight',learned)]:
  v=[rollout(fn,a.seed+i) for i in range(20)]; ev[name]={k:float(np.mean([z[k] for z in v])) for k in v[0]}
 a.output.parent.mkdir(parents=True,exist_ok=True); torch.save({'model':net.state_dict(),'target':target.state_dict()},a.output.with_suffix('.pt')); meta={'schema':'cross3_presslight_offline_v4_official_80d','state_dim':80,'action_dim':4,'phase_bits':8,'incoming_bins':36,'outgoing_bins':36,'phase_order':['NS_STRAIGHT','NS_LEFT','EW_STRAIGHT','EW_LEFT'],'reward':'official_presslight','reward_formula':'-0.25*sum(stopped_incoming_threshold_1s)' ,'discount':'0.8 ** (duration_seconds / 10)','training_steps':a.steps,'seed':a.seed,'eval':ev,'last_loss':last,'note':'Offline pretraining only; CARLA fine-tuning remains required.'}; a.output.with_suffix('.json').write_text(json.dumps(meta,indent=2)); print(json.dumps(meta,indent=2))
if __name__=='__main__': main()
