#!/usr/bin/env python3
"""Offline joint light/vehicle MAPPO smoke trainer.

The synthetic interface mirrors CARLA decision boundaries and is useful for
verifying the two-agent learning code before spending simulator time.
"""
import argparse,json,random
from pathlib import Path
import numpy as np, torch
from .joint_mappo import JointMAPPO, Rollout, gae

def run(seed, episodes, horizon, out):
 random.seed(seed); np.random.seed(seed); torch.manual_seed(seed); agent=JointMAPPO(); rows=[]
 for ep in range(episodes):
  lo=[];vo=[];go=[];la=[];va=[];ll=[];vl=[];rew=[];done=[];val=[]; phase=0; q=np.zeros(4)
  for t in range(horizon):
   light=np.zeros(80,np.float32); light[phase]=1; light[8:20]=np.clip(q,0,1).repeat(3); light[44:]=.1
   vehicle=np.array([q.sum(),q[phase],phase/4,t/horizon,1,0,0,0,0,0,0,0],np.float32); glob=np.r_[light[:80],vehicle[:16]].astype(np.float32)
   l,v,lp,vp,cv=agent.act(torch.tensor(light),torch.tensor(vehicle),torch.tensor(glob)); l=int(l);v=int(v); pressure=float(q[(l+1)%4]-q[l]); r=-float(q.sum())+(.3 if v==3 and pressure<0 else 0)-(.4 if v==2 and pressure>1 else 0)
   q=np.maximum(0,q+np.random.poisson(.25,4)-np.eye(4)[l]*np.random.poisson(1.2,4)); phase=l
   lo.append(light);vo.append(vehicle);go.append(glob);la.append(l);va.append(v);ll.append(float(lp));vl.append(float(vp));rew.append(r);done.append(float(t==horizon-1));val.append(float(cv))
  rt=torch.tensor(rew,dtype=torch.float32); vv=torch.tensor(val); ad,ret=gae(rt,vv,torch.tensor(done)); ro=Rollout(*(torch.tensor(x) for x in [np.array(lo),np.array(la),np.array(ll),np.array(vo),np.array(va),np.array(vl),np.array(go),rew,done]),vv,ad,ret); m=agent.update(ro); rows.append({'episode':ep,'reward':float(rt.sum()),**m})
 out.parent.mkdir(parents=True,exist_ok=True); torch.save({'light':agent.light.state_dict(),'vehicle':agent.vehicle.state_dict(),'critic':agent.critic.state_dict(),'config':{'seed':seed,'episodes':episodes,'horizon':horizon}},out.with_suffix('.pt')); out.with_suffix('.json').write_text(json.dumps({'schema':'cross3_joint_mappo_v1','rows':rows,'mean_last_reward':float(np.mean([x['reward'] for x in rows[-min(10,len(rows)):]]))},indent=2)); print(json.dumps(rows[-1],indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--episodes',type=int,default=20);p.add_argument('--horizon',type=int,default=64);p.add_argument('--seed',type=int,default=20261030);a=p.parse_args();run(a.seed,a.episodes,a.horizon,a.output)
