"""Frozen-checkpoint probes and coverage gates. Never trains or imports CARLA."""
import argparse,json
from pathlib import Path
import numpy as np
from diagnose_pilot import weights,predict
from learning import sparse_demand,bellman
from schema import PHASE_BITS

def main():
    p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--min-observed',type=int,default=4);p.add_argument('--min-served',type=int,default=1);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    here=Path(__file__).resolve().parent
    manifest=here.parents[1]/'carla_handoff/upstream_20261023_v4/map_dataset/junction_toolkit/maps/cross3/manifest.json'
    moves=json.loads(manifest.read_text())['movements'];keys=[(m['road'],m['lane']) for m in moves]
    progress=json.loads((a.input/'progress.json').read_text());raw=[];coverage=[];probe_states=[]
    episodes=sorted(a.input.glob('train_*'))
    for folder in episodes:raw+=json.loads((folder/'samples.json').read_text())
    replay=raw[:progress['samples']]
    for i in range(32):
        arrival=sparse_demand(i)[0];initial=(i//8)%4;phase=i%4;lane=keys.index((arrival['road'],arrival['lane']))
        state=dict(cur_phase=PHASE_BITS[initial][:],coming_vehicle=[0]*36,leaving_vehicle=[0]*36)
        state['coming_vehicle'][3*lane+2]=1
        probe_states.append(dict(id=i,demand_action=phase,initial_phase=initial,road=arrival['road'],lane=arrival['lane'],state=state))
        observed=[];matched=[]
        for folder in episodes:
            demand=json.loads((folder/'demand.json').read_text())
            if len(demand)!=1 or (demand[0]['road'],demand[0]['lane'])!=(arrival['road'],arrival['lane']):continue
            first=json.loads((folder/'decisions.json').read_text())[0]['phase']
            if first!=initial:continue
            matched.append(folder.name)
            # Only transitions actually retained by the training budget count.
            rows=json.loads((folder/'samples.json').read_text())
            start=sum(len(json.loads((f/'samples.json').read_text())) for f in episodes if f.name<folder.name)
            rows=rows[:max(0,progress['samples']-start)]
            observed += [r for r in rows if sum(r['state']['coming_vehicle'][lane*3:lane*3+3])>0]
        actions=[sum(r['action']==j for r in observed) for j in range(4)]
        coverage.append(dict(id=i,episodes=matched,observed_incoming=len(observed),action_counts=actions,passed=len(observed)>=a.min_observed and actions[phase]>=a.min_served))
    probes=[];td=[];models={}
    for model in sorted((a.input/'models').glob('round_*.h5')):
        w=weights(model);q=predict(w,[x['state'] for x in probe_states]);actions=q.argmax(axis=1)
        models[model.stem]=dict(correct_single_car=int(sum(int(v)==s['demand_action'] for v,s in zip(actions,probe_states))),total=32,actions=np.bincount(actions,minlength=4).tolist())
        for s,qs,act in zip(probe_states,q,actions):
            exact=[r for r in replay if r['state']==s['state']]
            probes.append(dict(model=model.stem,probe=s,q=qs.tolist(),greedy_action=int(act),action_mask=None,exact_support=len(exact),exact_action_counts=[sum(r['action']==j for r in exact) for j in range(4)]))
    w=weights(a.input/'models'/(progress['best']+'.h5'));target=weights(a.input/'models/round_000.h5')
    # Actual regular and sparse validation observations, with real actions/rewards.
    for folder in sorted(a.input.glob('val_*')):
        rows=json.loads((folder/'samples.json').read_text());q=predict(w,[r['state'] for r in rows]);nq=predict(target,[r['next_state'] for r in rows]).max(axis=1)
        for index,(r,qs,n) in enumerate(zip(rows,q,nq)):
            y=bellman(r,float(n))
            td.append(dict(label=folder.name,index=index,state=r['state'],q=qs.tolist(),actual_action=r['action'],greedy_action=int(qs.argmax()),reward=r['reward'],average_reward=r['average_reward'],duration=r['duration'],gamma_eff=r['discount'],terminal=r['terminal'],truncated=r['truncated'],next_q_max=float(n),td_target=y,td_error=y-float(qs[r['action']]),diagnostic_target_checkpoint='round_000'))
    report=dict(training_started=False,minimum_observed=a.min_observed,minimum_served=a.min_served,coverage_passed=sum(c['passed'] for c in coverage),coverage_total=32,coverage=coverage,probe_models=models,
        features=dict(input_dim=80,direction_and_turn='distinct incoming/outgoing lane histogram slots',waiting_age=False,speeds=False,external_queue=False,action_mask=False,synthetic_probes_unique=len({json.dumps(s['state'],sort_keys=True) for s in probe_states})),
        limitations=['Synthetic probes are feature-level checks, not simulator rollouts. Correct single-car action is a demand diagnostic, not an analytically optimal Q label.','Validation TD is a final-checkpoint diagnostic using round000, not replay of historical sampled training targets.','No initial weights or best-epoch weights were saved by pilot; no causal comparison of those checkpoints is possible.','Coverage thresholds are an explicit minimum engineering gate, not a claim of statistical sufficiency.'])
    for name,data in [('audit.json',report),('probes.json',probes),('validation_td.json',td)]:
        (a.output/name).write_text(json.dumps(data,indent=2))
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
