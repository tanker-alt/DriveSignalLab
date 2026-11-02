#!/usr/bin/env python3
"""Train continuous CAV acceleration and traffic-light policies in SUMO."""
from __future__ import annotations
import argparse, json, random
from pathlib import Path
import numpy as np, torch
from .config import GAMMA, GAE_LAMBDA, HORIZON
from .env import SumoJointEnv
from .continuous_mappo import ContinuousMAPPO, Batch
def gae(rewards, values, done):
    a=torch.zeros_like(rewards); last=torch.tensor(0.)
    for t in range(len(rewards)-1,-1,-1):
        nxt=0 if t==len(rewards)-1 else values[t+1]; delta=rewards[t]+GAMMA*nxt*(1-done[t])-values[t]; last=delta+GAMMA*GAE_LAMBDA*(1-done[t])*last; a[t]=last
    return a,a+values
def run(args):
    random.seed(args.seed); np.random.seed(args.seed); torch.manual_seed(args.seed); env=SumoJointEnv(args.sumo_cfg,args.tls_id,args.cav_id,args.horizon,args.seed,args.gui); agent=ContinuousMAPPO(); rows=[]
    for ep in range(args.episodes):
        lo=env.reset(seed=args.seed+ep); vo=env.vehicle_observation(args.cav_count); glob=np.r_[lo,vo.mean(0)]; L=[];V=[];G=[];LA=[];VA=[];LL=[];VL=[];R=[];D=[];X=[];collisions=0
        for _ in range(args.horizon):
            la,va,ll,vl,val=agent.act(torch.tensor(lo),torch.tensor(vo[0]),torch.tensor(glob)); la=int(la); va=float(va); no,r,d,info=env.step(la,[va]*args.cav_count); nvo=env.vehicle_observation(args.cav_count); ng=np.r_[no,nvo.mean(0)]
            L.append(lo); V.append(vo[0]); G.append(glob); LA.append(la); VA.append(va); LL.append(float(ll)); VL.append(float(vl)); R.append(r); D.append(float(d)); X.append(float(val)); collisions+=int(info['collision']); lo,vo,glob=no,nvo,ng
            if d: break
        rt=torch.tensor(R,dtype=torch.float32); vv=torch.tensor(X); ad,ret=gae(rt,vv,torch.tensor(D)); batch=Batch(torch.tensor(np.asarray(L),dtype=torch.float32),torch.tensor(LA),torch.tensor(LL),torch.tensor(np.asarray(V),dtype=torch.float32),torch.tensor(VA),torch.tensor(VL),torch.tensor(np.asarray(G),dtype=torch.float32),ret,ad); metrics=agent.update(batch); rows.append({'episode':ep,'reward':float(rt.sum()),'collision':collisions,**metrics})
        if (ep+1)%args.checkpoint_every==0: save(agent,args.output,rows)
    save(agent,args.output,rows); env.close(); print(json.dumps(rows[-1],ensure_ascii=False,indent=2))
def save(agent,out,rows):
    out.parent.mkdir(parents=True,exist_ok=True); torch.save({'light':agent.light.state_dict(),'vehicle':agent.vehicle.state_dict(),'critic':agent.critic.state_dict(),'config':{'gamma':GAMMA,'gae_lambda':GAE_LAMBDA}},out.with_suffix('.pt')); out.with_suffix('.json').write_text(json.dumps({'schema':'cross3_sumo_continuous_mappo_v1','rows':rows},ensure_ascii=False,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('--output',type=Path,required=True); p.add_argument('--episodes',type=int,default=500); p.add_argument('--horizon',type=int,default=HORIZON); p.add_argument('--seed',type=int,default=20261102); p.add_argument('--cav-count',type=int,default=10); p.add_argument('--sumo-cfg'); p.add_argument('--tls-id',default=''); p.add_argument('--cav-id',action='append',default=[]); p.add_argument('--gui',action='store_true'); p.add_argument('--checkpoint-every',type=int,default=10); run(p.parse_args())
