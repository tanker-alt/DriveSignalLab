"""Train/evaluate real SUMO → CARLA, optionally jointly updating LMDrive's head."""
import argparse
import hashlib
import json
from pathlib import Path
import random
import shutil
import subprocess
import numpy as np
import torch
from .core import CONTRACT
from .stages import validate_progress,sha
from .learning import Policy
from .sumo_env import SumoEnv, MAP
from .traffic_cases import demand,PROFILES

def write(path,data): path.write_text(json.dumps(data,indent=2,ensure_ascii=False,allow_nan=False)+'\n')

def run(args):
    args.output=args.output.resolve()
    if args.checkpoint: args.checkpoint=args.checkpoint.resolve()
    if args.waypoint_head: args.waypoint_head=args.waypoint_head.resolve()
    args.output.mkdir(parents=True,exist_ok=False)
    torch.set_num_threads(2); random.seed(args.seed); np.random.seed(args.seed)
    policy=Policy(args.seed, args.lr,signal_prior='none' if args.signal_prior=='auto' else args.signal_prior,
                  state_mode='extended' if args.state_mode=='auto' else args.state_mode)
    source=policy.load(args.checkpoint) if args.checkpoint else None
    if args.signal_prior!='auto' and policy.signal_prior!=args.signal_prior:
        raise ValueError('Requested signal prior differs from checkpoint')
    if args.state_mode!='auto' and policy.state_mode!=args.state_mode:
        raise ValueError('Checkpoint state mode mismatch')
    validate_progress(args,source)
    if source and source.get('map_sha256') != hashlib.sha256((MAP/'map.xodr').read_bytes()).hexdigest():
        raise ValueError('Checkpoint map fingerprint mismatch')
    if policy.loaded_head and not args.vla:
        raise ValueError('A joint checkpoint requires --vla; do not silently discard its trained internal head')
    if policy.loaded_head and source.get('initial_waypoint_head') != str(args.waypoint_head):
        raise ValueError('Resume/evaluate VLA with the same --waypoint-head initialization as training')
    for group in policy.optimizer.param_groups: group['lr']=args.lr
    provenance=dict(backend=args.backend,seed=args.seed,source_checkpoint=str(args.checkpoint),source=source,
        source_checkpoint_sha256=sha(args.checkpoint) if args.checkpoint else None,
        map_sha256=hashlib.sha256((MAP/'map.xodr').read_bytes()).hexdigest(),
        training_scope='signal_and_internal_waypoint_final_layer' if args.vla and args.train else 'signal_only' if args.train else 'frozen_evaluation',
        signal_prior=policy.signal_prior, vla_base='LMDrive local config', initial_waypoint_head=str(args.waypoint_head),
        contract=CONTRACT)
    write(args.output/'protocol.json',dict(provenance,arguments={k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()}))
    snapshot=args.output/'source'; snapshot.mkdir()
    for f in Path(__file__).parent.glob('*.py'): shutil.copyfile(f,snapshot/f.name)
    git=subprocess.run(['git','rev-parse','HEAD'],capture_output=True,text=True)
    write(args.output/'source_hashes.json',dict(git_head=git.stdout.strip(),files={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in snapshot.glob('*.py')}))
    reports=[]
    if args.train and not args.vla: policy.save(args.output/'initial.pt',dict(provenance,episode=-1))
    for ep in range(args.episodes):
        folder=args.output/('episode_%03d'%ep); seed=args.seed+ep
        profile=PROFILES[ep%len(PROFILES)] if args.demand_profile=='cycle' else args.demand_profile
        rows=demand(seed,args.arrival_seconds,args.rate,args.vla,profile)
        env=None
        if not rows: raise ValueError('No generated demand; increase --arrival-seconds')
        try:
            if args.backend=='sumo': cls=SumoEnv
            else:
                from .carla_env import CarlaEnv
                cls=CarlaEnv
            env=cls(folder,rows,seed,args.duration,policy=policy,joint=args.vla,train=args.train,
                    port=args.port,waypoint_head=args.waypoint_head, fairness_weight=args.fairness_weight,
                    max_wait_weight=args.max_wait_weight)
            write(folder/'demand.json',rows)
            obs=env.observe(); rollout=[]; total_reward=0.
            while True:
                action,logp,value=policy.act(obs,deterministic=not args.train)
                allowed=np.flatnonzero(obs['light_mask'])
                if args.controller=='fixed':
                    wanted=(env.signals.active+1)%4 if env.signals.age>=args.fixed_green else env.signals.active
                    action=int(wanted if wanted in allowed else allowed[0])
                elif args.controller=='maxpressure':
                    pressure=obs['light'][4:8]-obs['light'][8:12]
                    best=max(pressure[i] for i in allowed)
                    action=env.signals.active if env.signals.active in allowed and pressure[env.signals.active]==best else int(max(allowed,key=lambda i:pressure[i]))
                nxt,reward,terminated,truncated,info=env.step(action)
                if args.train:
                    rollout.append(dict(obs=obs,action=action,logp=logp,value=value,reward=reward,
                                        terminated=terminated,**info))
                total_reward+=reward; obs=nxt
                if terminated or truncated: break
            update={}
            if args.train:
                bootstrap=0. if terminated else policy.act(obs,True)[2]
                torch.save(dict(transitions=rollout,bootstrap=bootstrap),str(folder/'rollout.pt'))
                update=policy.update(rollout,bootstrap,args.epochs)
                if args.vla and not update['head_samples']:
                    raise RuntimeError('No actual VLA head samples; refusing to claim joint training')
                policy.save(args.output/('checkpoint_%03d.pt'%ep),dict(provenance,episode=ep))
            report=dict(env.metrics.report(),seed=seed,reward=total_reward,terminated=terminated,truncated=truncated,
                        controller=args.controller,backend=args.backend,transport=getattr(env,'transport','carla_rpc'),demand_profile=profile,signal_switches=env.signals.switches,training=update)
            write(folder/'metrics.json',report)
            write(folder/'vehicles.json',list(env.metrics.rows.values()))
            write(folder/'crossings.json',env.metrics.safety.events)
            reports.append(report); write(args.output/'results.json',reports)
            print(json.dumps(report,ensure_ascii=False),flush=True)
        except BaseException as exc:
            write(args.output/'failure.json',dict(episode=ep,error=repr(exc)))
            if env:
                write(folder/'partial_metrics.json',env.metrics.report())
            raise
        finally:
            if env: env.close()
    return reports

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--backend',choices=['sumo','carla'],required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--checkpoint',type=Path)
    p.add_argument('--metrics-acceptance',type=Path)
    p.add_argument('--sumo-acceptance',type=Path)
    p.add_argument('--carla-acceptance',type=Path)
    p.add_argument('--controller',choices=['learned','fixed','maxpressure'],default='learned')
    p.add_argument('--train',action='store_true')
    p.add_argument('--vla',action='store_true',help='Use two real LMDrive egos; --train updates its internal final waypoint layer and signal actor')
    p.add_argument('--waypoint-head',type=Path)
    p.add_argument('--episodes',type=int,default=1)
    p.add_argument('--seed',type=int,default=2026)
    p.add_argument('--arrival-seconds',type=float,default=120)
    p.add_argument('--duration',type=float,default=420)
    p.add_argument('--rate',type=float,default=.035)
    p.add_argument('--demand-profile',choices=list(PROFILES)+['cycle'],default='balanced')
    p.add_argument('--fixed-green',type=float,default=15)
    p.add_argument('--port',type=int,default=31610)
    p.add_argument('--lr',type=float,default=3e-4)
    p.add_argument('--signal-prior',choices=['auto','none','maxpressure'],default='auto')
    p.add_argument('--state-mode',choices=['auto','base','extended'],default='auto')
    p.add_argument('--epochs',type=int,default=4)
    p.add_argument('--fairness-weight',type=float,default=0.)
    p.add_argument('--max-wait-weight',type=float,default=0.)
    a=p.parse_args()
    if a.episodes<1 or a.epochs<1 or a.rate<=0 or not 0<a.arrival_seconds<=a.duration: p.error('Invalid episode, demand or horizon settings')
    if a.lr<=0: p.error('--lr must be positive')
    if a.waypoint_head and not a.vla: p.error('--waypoint-head requires --vla')
    if a.vla and a.backend!='carla': p.error('--vla requires real CARLA')
    if a.train and a.controller!='learned': p.error('Only learned controller can train')
    if a.controller=='learned' and not a.checkpoint and (not a.train or a.backend=='carla'):
        p.error('Frozen evaluation/CARLA adaptation requires a compatible SUMO checkpoint')
    run(a)
if __name__=='__main__': main()
