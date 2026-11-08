"""Restart CARLA adaptation from an accepted study without rerunning its baselines.

This starts a new training run from the selected SUMO checkpoint. It does not
pretend that a process restart restores the RNG state of a partially saved run.
"""
import argparse
import json
from pathlib import Path
from .stage2 import execute, score, gate, write
from .stages import load_receipt, sha
from .audit_metrics import audit
from .compare import compare


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--study',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--port',type=int,default=31600)
    p.add_argument('--lr',type=float,default=3e-4)
    a=p.parse_args()
    if a.lr<=0: p.error('--lr must be positive')
    a.study=a.study.resolve(); a.output=a.output.resolve()
    protocol=json.loads((a.study/'protocol.json').read_text())
    args=protocol['arguments']
    a.metrics_acceptance=Path(args['metrics_acceptance'])
    load_receipt(a.metrics_acceptance,1)
    sumo_gate=a.study/'sumo_acceptance.json'; accepted=load_receipt(sumo_gate,2)
    if accepted['milestone']!='sumo_validation': raise ValueError('Expected accepted SUMO study')
    checkpoint=Path(accepted['result']['selected']['checkpoint'])
    if sha(checkpoint)!=accepted['checkpoint_sha256']: raise ValueError('SUMO checkpoint changed')
    a.arrival_seconds=args['arrival_seconds'];a.duration=args['duration'];a.signal_prior=args['signal_prior']
    n=args['eval_episodes']; episodes=args['carla_episodes']
    folders=[a.study/name for name in ('carla_before','carla_fixed','carla_pressure')]
    compare(folders) # identical demand, frozen policies and evaluation contracts
    baseline=[]
    for folder in folders:
        run=json.loads((folder/'protocol.json').read_text())['arguments']
        if run['backend']!='carla' or run['seed']!=26001 or run['episodes']!=n:
            raise ValueError('Unexpected baseline protocol')
        for key in ('arrival_seconds','duration'):
            if run[key]!=args[key]: raise ValueError('Baseline horizon mismatch')
        rows=json.loads((folder/'results.json').read_text())
        if len(rows)!=n: raise ValueError('Incomplete baseline')
        for i in range(n):
            checked=audit(folder/('episode_%03d'%i))
            if not checked['passed']: raise ValueError('Baseline audit failed')
        baseline.append(score(rows))
    before_args=json.loads((folders[0]/'protocol.json').read_text())['arguments']
    if sha(Path(before_args['checkpoint']))!=sha(checkpoint): raise ValueError('Wrong direct-transfer baseline')
    a.output.mkdir(parents=True,exist_ok=False)
    write(a.output/'protocol.json',dict(original_protocol=protocol,source_study=str(a.study),
        learning_rate=a.lr,
        restart='fresh training from selected SUMO model; original seeds and budget; failed run retained'))
    training=a.output/'carla_train'
    execute(a,training,'carla',checkpoint=checkpoint,train=True,episodes=episodes,seed=25001,sumo_gate=sumo_gate)
    adapted=training/('checkpoint_%03d.pt'%(episodes-1))
    after=execute(a,a.output/'carla_after','carla',checkpoint=adapted,episodes=n,seed=26001,sumo_gate=sumo_gate)
    bs,fixed,pressure=baseline; ads=score(after)
    passed=ads['unfinished']==0 and ads['collisions']==0 and ads['red_violations']==0 and ads['mean_wait']<=bs['mean_wait']*.95
    result=dict(stage=2,milestone='carla_adaptation',passed=passed,before=bs,after=ads,fixed=fixed,maxpressure=pressure)
    write(a.output/'carla_adaptation.json',result)
    write(a.output/'paired_comparison.json',compare([folders[0],a.output/'carla_after']))
    if not passed:
        print('CARLA improvement criterion not met; stage 3 remains gated.',flush=True);return
    tb=execute(a,a.output/'carla_test_before','carla',checkpoint=checkpoint,episodes=n,seed=27001,sumo_gate=sumo_gate)
    ta=execute(a,a.output/'carla_test_after','carla',checkpoint=adapted,episodes=n,seed=27001,sumo_gate=sumo_gate)
    tbs,tas=score(tb),score(ta)
    passed=tas['unfinished']==0 and tas['collisions']==0 and tas['red_violations']==0 and tas['mean_wait']<=tbs['mean_wait']*.95
    result.update(passed=passed,heldout_before=tbs,heldout_after=tas)
    write(a.output/'carla_adaptation.json',result)
    if passed:
        evidence=[a.output/'carla_adaptation.json',sumo_gate,a.metrics_acceptance,a.output/'protocol.json']
        for folder in folders+[a.output/'carla_after',a.output/'carla_test_before',a.output/'carla_test_after']:
            evidence.extend(folder.glob('episode_*/metric_audit.json'))
            evidence.extend(folder.glob('episode_*/measurements.jsonl.gz'))
            evidence.extend([folder/'protocol.json',folder/'results.json'])
        gate(a.output/'carla_acceptance.json','carla_adaptation',adapted,evidence,result)
    print('Stage 2 accepted.' if passed else 'Held-out improvement failed; stage 3 remains gated.',flush=True)


if __name__=='__main__': main()
