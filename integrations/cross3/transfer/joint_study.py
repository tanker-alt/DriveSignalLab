"""Exploratory before/after joint VLA internal-head and signal PPO experiment."""
import argparse,json,subprocess,sys
from pathlib import Path
from .stage2 import score,write,gate
from .audit_metrics import audit
from .compare import compare
from .stages import sha,load_receipt

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    b=a.output.resolve();b.mkdir(parents=True,exist_ok=False)
    metrics=Path('outputs/staged/stage1_joint_retirement/acceptance.json').resolve()
    load_receipt(metrics,1)
    cp=Path('outputs/staged/ablation_multiflow/extended_fair_30101_train/checkpoint_019.pt').resolve()
    sg=b/'sumo_acceptance.json'
    source=Path('outputs/staged/ablation_multiflow/extended_fair_30101_eval')
    evidence=[metrics,source/'results.json',source/'protocol.json']
    for ep in sorted(source.glob('episode_*')):
        if not audit(ep)['passed']:raise ValueError('SUMO evidence audit failed')
        evidence.extend(ep/name for name in ('metric_audit.json','metrics.json','measurements.jsonl.gz'))
    rows=json.loads((source/'results.json').read_text())
    if any(r['unfinished'] or r['collision_pairs'] or r['red_light_violations'] for r in rows):
        raise ValueError('Unsafe or incomplete source model')
    gate(sg,'sumo_validation',cp,evidence,dict(scope='Reaudited existing SUMO validation after CARLA retired-ego control fix; no SUMO retraining',score=score(rows)))
    head=Path('/home/zlj/Drive/models/lmdrive/cross3_waypoint_head.pt')
    protocol=dict(platform='CARLA 0.9.12',sumo_training_episodes=0,joint_training_episodes=4,
        evaluation_episodes_per_policy=4,training_seed=41001,evaluation_seed=42001,
        demand_profile='cycle',arrival_seconds=10,duration=150,starting_signal=str(cp),
        starting_signal_sha256=sha(cp),starting_vla_head=str(head),starting_vla_head_sha256=sha(head),
        trainable_vla='waypoints_predictor.2, 40970 internal parameters; upstream frozen',
        authorization='User explicitly requested joint experiment despite failed signal adaptation; not stage-2 acceptance',
        success='all complete; no collision/red crossing increase; mean wait at least 5% lower; VLA ego outcomes reported')
    write(b/'protocol.json',protocol)
    def run(name,checkpoint,train=False):
        out=b/name
        cmd=[sys.executable,'-m','integrations.cross3.transfer.run','--backend','carla','--port','31600',
             '--vla','--exploratory-joint','--waypoint-head',str(head),'--checkpoint',str(checkpoint),
             '--metrics-acceptance',str(metrics),'--sumo-acceptance',str(sg),'--output',str(out),
             '--episodes','4','--seed','41001' if train else '42001','--demand-profile','cycle',
             '--arrival-seconds','10','--duration','150','--lr','.0003']
        if train:cmd+=['--train']
        print(name,flush=True)
        with (b/(name+'.log')).open('w') as log:subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,check=True)
        rows=json.loads((out/'results.json').read_text());egos=[]
        for ep in sorted(out.glob('episode_*')):
            checked=audit(ep)
            if not checked['passed']:raise ValueError('Physical metric audit failed: '+str(ep))
            vehicles=json.loads((ep/'vehicles.json').read_text())
            egos.extend(dict(episode=ep.name,**v) for v in vehicles if v['ego'])
        write(out/'ego_results.json',egos)
        return rows,egos
    before,be=run('before',cp)
    training,te=run('train',cp,True)
    if any(r['training']['head_parameter_change_l1']<=0 or r['training']['signal_parameter_change_l1']<=0 for r in training):
        raise ValueError('Both policies must actually update')
    after,ae=run('after',b/'train/checkpoint_003.pt')
    write(b/'paired_comparison.json',compare([b/'before',b/'after']))
    bs,ads=score(before),score(after)
    def ego_summary(rows):return dict(total=len(rows),completed=sum(v['finish'] is not None for v in rows),mean_wait=sum(v['waiting'] for v in rows)/len(rows))
    result=dict(before=bs,after=ads,ego_before=ego_summary(be),ego_after=ego_summary(ae),
        improved=ads['unfinished']==0 and ads['collisions']==0 and ads['red_violations']==0 and ads['mean_wait']<=bs['mean_wait']*.95,
        training_episodes=4,sumo_training_episodes=0,scope='small exploratory comparison; no generalization or formal acceptance claim')
    write(b/'comparison.json',result);print(json.dumps(result,indent=2),flush=True)
if __name__=='__main__':main()
