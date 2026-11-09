"""Matched state/reward ablation on all four demand families; no held-out test use."""
import argparse
import json
import subprocess
import sys
import torch
from pathlib import Path
from .audit_metrics import audit
from .audit_kinematics import audit as audit_motion
from .stage2 import score
from .stages import load_receipt, metric_sources, sha

ARMS={'base_plain':('base',0.,0.),'extended_plain':('extended',0.,0.),
      'base_fair':('base',.5,.25),'extended_fair':('extended',.5,.25)}

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--metrics-acceptance',type=Path,required=True)
    a=p.parse_args();load_receipt(a.metrics_acceptance,1)
    a.output=a.output.resolve();a.output.mkdir(parents=True,exist_ok=False)
    sources=metric_sources()
    protocol=dict(arms=ARMS,train_seeds=[30001,30101],training_episodes=20,
        validation_seed=31001,validation_episodes=8,demand_profile='cycle',
        arrival_seconds=40,duration=200,prior='maxpressure',learning_rate=.0003,
        comparison='same network size/initialization; base masks eight history inputs in actor AND critic',
        scope='SUMO validation screening only; no held-out test and no CARLA acceptance',
        state='service age and stable decision-interval queue difference; no wait-history features yet',
        reward='existing direction-total-wait disparity and max-wait shaping, not revised fairness',
        metric_sources=sources)
    def write(path,obj):path.write_text(json.dumps(obj,indent=2)+'\n')
    write(a.output/'protocol.json',protocol)
    def run(name,controller='learned',seed=31001,train=False,checkpoint=None,state='extended',fw=0.,mw=0.):
        folder=a.output/name
        cmd=[sys.executable,'-m','integrations.cross3.transfer.run','--backend','sumo',
             '--metrics-acceptance',str(a.metrics_acceptance.resolve()),'--output',str(folder),
             '--episodes','20' if train else '8','--seed',str(seed),'--arrival-seconds','40',
             '--duration','200','--demand-profile','cycle','--controller',controller,
             '--state-mode',state,'--fairness-weight',str(fw),'--max-wait-weight',str(mw)]
        if train:cmd+=['--train','--signal-prior','maxpressure']
        if checkpoint:cmd+=['--checkpoint',str(checkpoint)]
        print(name,flush=True)
        with (a.output/(name+'.log')).open('w') as log:
            subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,check=True)
        rows=json.loads((folder/'results.json').read_text())
        if not train:
            for i in range(8):
                ep=folder/('episode_%03d'%i)
                checked=audit(ep);motion=audit_motion(ep/'measurements.jsonl.gz')
                if not checked['passed'] or not motion['passed']:raise RuntimeError('Audit failed: '+str(ep))
            # Compare the actual demands, not only numeric seed labels.
            if name!='fixed':
                for i in range(8):
                    ep='episode_%03d'%i
                    if (folder/ep/'demand.json').read_bytes()!=(a.output/'fixed'/ep/'demand.json').read_bytes():
                        raise RuntimeError('Unpaired traffic')
        return rows
    results={}
    for name in ('fixed','maxpressure'):
        rows=run(name,controller=name)
        results[name]=dict(score=score(rows),families={family:score([r for r in rows if r['demand_profile']==family])
                         for family in ('balanced','east_west','north_south','left_heavy')})
    for seed in protocol['train_seeds']:
        initial_reference=None
        for arm,(state,fw,mw) in ARMS.items():
            name='%s_%d'%(arm,seed)
            run(name+'_train',seed=seed,train=True,state=state,fw=fw,mw=mw)
            initial=torch.load(str(a.output/(name+'_train')/'initial.pt'),map_location='cpu')
            if initial_reference is None: initial_reference=initial
            for net in ('light','critic'):
                if any(not torch.equal(v,initial_reference[net][k]) for k,v in initial[net].items()):
                    raise RuntimeError('Unmatched initialization')
            checkpoint=a.output/(name+'_train')/'checkpoint_019.pt'
            rows=run(name+'_eval',checkpoint=checkpoint,state=state)
            results[name]=dict(score=score(rows),mean_episode_max_wait=sum(r['max_observed_wait_s'] for r in rows)/8,
                families={f:score([r for r in rows if r['demand_profile']==f]) for f in ('balanced','east_west','north_south','left_heavy')},
                paired_wait_delta_vs_pressure=[r['mean_observed_wait_s']-b['mean_observed_wait_s'] for r,b in zip(rows,json.loads((a.output/'maxpressure/results.json').read_text()))],
                checkpoint=str(checkpoint),checkpoint_sha256=sha(checkpoint))
            write(a.output/'progress.json',results)
    if metric_sources()!=sources:raise RuntimeError('Metric source changed during study')
    evidence={str(p):sha(p) for p in a.output.glob('*/episode_*/metric_audit.json')}
    evidence.update({str(p):sha(p) for p in a.output.glob('*/results.json')})
    write(a.output/'results.json',dict(protocol=protocol,results=results,evidence=evidence,
        conclusion='Screening complete; assess seed/family consistency before promotion.'))
    print('Ablation complete.',flush=True)

if __name__=='__main__':main()
