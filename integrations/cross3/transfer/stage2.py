"""Validate SUMO first, then transfer, adapt and test on disjoint CARLA seeds.

A failed milestone produces a report, not an acceptance receipt for stage 3.
No VLA training is launched here.
"""
import argparse
import json
import subprocess
import sys
from pathlib import Path
import numpy as np
from .audit_metrics import audit
from .stages import load_receipt,metric_sources,sha


def write(path,value): path.write_text(json.dumps(value,indent=2)+'\n')


def execute(args,folder,backend,controller='learned',checkpoint=None,train=False,episodes=1,seed=23001,profile='cycle',sumo_gate=None):
    cmd=[sys.executable,'-m','integrations.cross3.transfer.run','--backend',backend,'--controller',controller,
         '--metrics-acceptance',str(args.metrics_acceptance),'--output',str(folder),'--episodes',str(episodes),
         '--seed',str(seed),'--demand-profile',profile,'--arrival-seconds',str(args.arrival_seconds),
         '--duration',str(args.duration),'--port',str(args.port)]
    if getattr(args,'lr',None) is not None: cmd+=['--lr',str(args.lr)]
    if checkpoint: cmd+=['--checkpoint',str(checkpoint)]
    if sumo_gate: cmd+=['--sumo-acceptance',str(sumo_gate)]
    if train:
        cmd+=['--train']
        if checkpoint is None: cmd+=['--signal-prior',args.signal_prior]
    print('Stage 2:',folder.name,flush=True)
    with folder.with_suffix('.log').open('w') as stream:
        result=subprocess.run(cmd,stdout=stream,stderr=subprocess.STDOUT)
    if result.returncode: raise RuntimeError('Run failed: '+str(folder.with_suffix('.log')))
    rows=json.loads((folder/'results.json').read_text())
    if not train:
        for i in range(episodes):
            checked=audit(folder/('episode_%03d'%i))
            if not checked['passed']: raise RuntimeError('Independent metric audit failed: '+str(folder))
    return rows


def score(rows):
    return dict(unfinished=sum(r['unfinished'] for r in rows),due=sum(r['due'] for r in rows),
        collisions=sum(r['collision_pairs'] for r in rows),red_violations=sum(r['red_light_violations'] for r in rows),
        mean_wait=float(np.mean([r['mean_observed_wait_s'] for r in rows])),
        scenario_wait_std=float(np.std([r['mean_observed_wait_s'] for r in rows],ddof=1)) if len(rows)>1 else None)


def acceptable(candidate,fixed):
    return (candidate['unfinished']==0 and candidate['collisions']==0 and candidate['red_violations']==0
            and candidate['mean_wait']<=fixed['mean_wait'])


def gate(path,milestone,checkpoint,files,result):
    artifacts={str(f.resolve()):sha(f) for f in files+[checkpoint]}
    receipt=dict(stage=2,milestone=milestone,passed=True,metric_sources=metric_sources(),
                 checkpoint_sha256=sha(checkpoint),artifacts=artifacts,result=result)
    write(path,receipt)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--metrics-acceptance',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--port',type=int,default=31600);p.add_argument('--sumo-episodes',type=int,default=100)
    p.add_argument('--carla-episodes',type=int,default=20);p.add_argument('--arrival-seconds',type=float,default=40)
    p.add_argument('--signal-prior',choices=['none','maxpressure'],default='none')
    p.add_argument('--duration',type=float,default=200);p.add_argument('--eval-episodes',type=int,default=8)
    a=p.parse_args(); load_receipt(a.metrics_acceptance,1)
    if a.sumo_episodes<4 or a.carla_episodes<4 or a.eval_episodes<4: p.error('Cover all four demand families')
    a.output=a.output.resolve();a.output.mkdir(parents=True,exist_ok=False)
    protocol=dict(training_seeds=[20001,20001+a.sumo_episodes-1],validation_seed=23001,heldout_seed=24001,
        carla_training_seed=25001,carla_validation_seed=26001,carla_test_seed=27001,
        families=['balanced','east_west','north_south','left_heavy'],
        sumo_gate='all complete, zero observed collisions/red crossings, mean wait no worse than fixed; MaxPressure reported',
        carla_gate='all complete, zero collisions/red crossings; paired mean wait improvement >=5% over direct transfer on validation AND held-out test',
        arguments={k:str(v) if isinstance(v,Path) else v for k,v in vars(a).items()})
    write(a.output/'protocol.json',protocol)
    training=a.output/'sumo_train'
    execute(a,training,'sumo',train=True,episodes=a.sumo_episodes,seed=20001)
    fixed=execute(a,a.output/'sumo_fixed_validation','sumo','fixed',episodes=a.eval_episodes)
    pressure=execute(a,a.output/'sumo_pressure_validation','sumo','maxpressure',episodes=a.eval_episodes)
    initial_prior=None
    if a.signal_prior=='maxpressure':
        initial_prior=score(execute(a,a.output/'sumo_initial_prior_validation','sumo',
            checkpoint=training/'initial.pt',episodes=a.eval_episodes))
    candidates=[]
    rounds=sorted({a.sumo_episodes//4-1,a.sumo_episodes//2-1,3*a.sumo_episodes//4-1,a.sumo_episodes-1})
    for ep in rounds:
        checkpoint=training/('checkpoint_%03d.pt'%ep)
        rows=execute(a,a.output/('sumo_candidate_%03d'%ep),'sumo',checkpoint=checkpoint,episodes=a.eval_episodes)
        candidates.append(dict(checkpoint=str(checkpoint),score=score(rows)))
    best=min(candidates,key=lambda c:(c['score']['collisions']+c['score']['red_violations'],c['score']['unfinished'],c['score']['mean_wait']))
    summary=dict(stage=2,milestone='sumo_validation',passed=acceptable(best['score'],score(fixed)),
                 fixed=score(fixed),maxpressure=score(pressure),initial_prior=initial_prior,candidates=candidates,selected=best)
    write(a.output/'sumo_validation.json',summary)
    if not summary['passed']:
        print('SUMO validation did not pass; CARLA adaptation and VLA training remain gated.',flush=True);return
    checkpoint=Path(best['checkpoint'])
    held_fixed=execute(a,a.output/'sumo_fixed_test','sumo','fixed',episodes=a.eval_episodes,seed=24001)
    held=execute(a,a.output/'sumo_learned_test','sumo',checkpoint=checkpoint,episodes=a.eval_episodes,seed=24001)
    summary['heldout']=dict(fixed=score(held_fixed),learned=score(held))
    summary['passed']=acceptable(score(held),score(held_fixed));write(a.output/'sumo_validation.json',summary)
    if not summary['passed']:
        print('SUMO held-out test did not pass; do not tune against this test set.',flush=True);return
    sumo_gate=a.output/'sumo_acceptance.json'
    gate(sumo_gate,'sumo_validation',checkpoint,[a.output/'sumo_validation.json'],summary)
    before=execute(a,a.output/'carla_before','carla',checkpoint=checkpoint,episodes=a.eval_episodes,seed=26001,sumo_gate=sumo_gate)
    cf=execute(a,a.output/'carla_fixed','carla','fixed',episodes=a.eval_episodes,seed=26001)
    cp=execute(a,a.output/'carla_pressure','carla','maxpressure',episodes=a.eval_episodes,seed=26001)
    adaptation=a.output/'carla_train'
    execute(a,adaptation,'carla',checkpoint=checkpoint,train=True,episodes=a.carla_episodes,seed=25001,sumo_gate=sumo_gate)
    adapted=adaptation/('checkpoint_%03d.pt'%(a.carla_episodes-1))
    after=execute(a,a.output/'carla_after','carla',checkpoint=adapted,episodes=a.eval_episodes,seed=26001,sumo_gate=sumo_gate)
    bs,ads=score(before),score(after)
    passed=(ads['unfinished']==0 and ads['collisions']==0 and ads['red_violations']==0 and ads['mean_wait']<=bs['mean_wait']*.95)
    result=dict(stage=2,milestone='carla_adaptation',passed=passed,before=bs,after=ads,fixed=score(cf),maxpressure=score(cp))
    write(a.output/'carla_adaptation.json',result)
    if not passed:
        print('CARLA improvement criterion not met; stage 3 remains gated.',flush=True);return
    tb=execute(a,a.output/'carla_test_before','carla',checkpoint=checkpoint,episodes=a.eval_episodes,seed=27001,sumo_gate=sumo_gate)
    ta=execute(a,a.output/'carla_test_after','carla',checkpoint=adapted,episodes=a.eval_episodes,seed=27001,sumo_gate=sumo_gate)
    tbs,tas=score(tb),score(ta)
    passed=tas['unfinished']==0 and tas['collisions']==0 and tas['red_violations']==0 and tas['mean_wait']<=tbs['mean_wait']*.95
    result.update(passed=passed,heldout_before=tbs,heldout_after=tas);write(a.output/'carla_adaptation.json',result)
    if passed: gate(a.output/'carla_acceptance.json','carla_adaptation',adapted,[a.output/'carla_adaptation.json',sumo_gate],result)
    print('Stage 2 accepted.' if passed else 'Held-out CARLA improvement not confirmed; stage 3 remains gated.',flush=True)

if __name__=='__main__':main()
