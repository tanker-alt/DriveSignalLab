"""Select an existing SUMO checkpoint; train only in CARLA, with held-out confirmation."""
import argparse
import json
from pathlib import Path
from .stage2 import execute, score, acceptable, gate, write
from .stages import load_receipt, sha
from .audit_metrics import audit
from .compare import compare


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--port',type=int,default=31600)
    a=p.parse_args();a.output=a.output.resolve();a.output.mkdir(parents=True,exist_ok=False)
    a.metrics_acceptance=Path('outputs/staged/stage1_history_fixed_retry/acceptance.json').resolve()
    load_receipt(a.metrics_acceptance,1)
    study=Path('outputs/staged/ablation_multiflow').resolve()
    records=json.loads((study/'results.json').read_text())['results']
    candidates={k:v for k,v in records.items() if 'checkpoint' in v and acceptable(v['score'],records['fixed']['score'])}
    selected=min(candidates,key=lambda k:candidates[k]['score']['mean_wait'])
    checkpoint=Path(candidates[selected]['checkpoint'])
    if sha(checkpoint)!=candidates[selected]['checkpoint_sha256']:raise ValueError('Checkpoint changed')
    # Reuse and independently audit existing SUMO measurements; never retrain SUMO.
    evidence=[study/'results.json',a.metrics_acceptance]
    for folder in (study/(selected+'_eval'),study/'fixed'):
        for ep in sorted(folder.glob('episode_*')):
            if not audit(ep)['passed']:raise ValueError('Existing SUMO audit failed')
            evidence.extend(ep/name for name in ('measurements.jsonl.gz','metrics.json','metric_audit.json'))
    a.arrival_seconds=40.;a.duration=200.;a.lr=.001;a.signal_prior='maxpressure'
    a.fairness_weight=0.;a.max_wait_weight=0.
    protocol=dict(selected=selected,checkpoint=str(checkpoint),selection=candidates[selected]['score'],
        sumo_training_episodes_this_study=0,carla_training_episodes=60,learning_rate=a.lr,
        training_seed=35001,validation_seed=36001,test_seed=37001,eval_episodes=8,
        candidate_episodes=[20,40,60],reward='original vehicle-time and collision reward',
        acceptance='>=5% lower mean wait than the SAME frozen SUMO model in CARLA; all complete; no collision/red crossings',
        selection_scope='existing SUMO validation only, not an independent test',
        arguments=dict(arrival_seconds=40,duration=200,port=a.port))
    write(a.output/'protocol.json',protocol)
    sg=a.output/'sumo_acceptance.json'
    gate(sg,'sumo_validation',checkpoint,evidence,dict(passed=True,scope='existing validation reuse explicitly requested; no SUMO retraining or new test',selected=candidates[selected]))
    before=execute(a,a.output/'carla_before','carla',checkpoint=checkpoint,episodes=8,seed=36001,sumo_gate=sg)
    pressure=execute(a,a.output/'carla_pressure','carla','maxpressure',episodes=8,seed=36001)
    train=a.output/'carla_train'
    execute(a,train,'carla',checkpoint=checkpoint,train=True,episodes=60,seed=35001,sumo_gate=sg)
    candidates=[]
    for ep in (19,39,59):
        cp=train/('checkpoint_%03d.pt'%ep);folder=a.output/('carla_candidate_%03d'%ep)
        rows=execute(a,folder,'carla',checkpoint=cp,episodes=8,seed=36001,sumo_gate=sg)
        comparison=compare([a.output/'carla_before',folder]);write(folder/'paired.json',comparison)
        candidates.append(dict(checkpoint=str(cp),score=score(rows)))
    best=min(candidates,key=lambda x:(x['score']['collisions']+x['score']['red_violations'],x['score']['unfinished'],x['score']['mean_wait']))
    bs=score(before);after=best['score']
    passed=acceptable(after,bs) and after['mean_wait']<=.95*bs['mean_wait']
    result=dict(passed=passed,before=bs,after=after,maxpressure=score(pressure),candidates=candidates,selected=best)
    write(a.output/'carla_adaptation.json',result)
    if not passed:print('Validation improvement gate not met; test remains unused.',flush=True);return
    cp=Path(best['checkpoint'])
    tb=execute(a,a.output/'carla_test_before','carla',checkpoint=checkpoint,episodes=8,seed=37001,sumo_gate=sg)
    ta=execute(a,a.output/'carla_test_after','carla',checkpoint=cp,episodes=8,seed=37001,sumo_gate=sg)
    bs,after=score(tb),score(ta)
    result.update(heldout_before=bs,heldout_after=after,passed=acceptable(after,bs) and after['mean_wait']<=.95*bs['mean_wait'])
    write(a.output/'carla_adaptation.json',result)
    if result['passed']:
        files=[a.output/'protocol.json',a.output/'carla_adaptation.json',sg,a.metrics_acceptance]
        files+=list(a.output.glob('*/episode_*/metric_audit.json'))+list(a.output.glob('*/episode_*/measurements.jsonl.gz'))
        gate(a.output/'carla_acceptance.json','carla_adaptation',cp,files,result)
    print('CARLA confirmation passed.' if result['passed'] else 'Held-out improvement failed.',flush=True)

if __name__=='__main__':main()
