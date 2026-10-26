"""Read existing JSON/HDF5 only; no training, simulator, or model writes."""
import argparse,json,statistics,sys
from pathlib import Path
import numpy as np
import h5py
from learning import bellman

def weights(path):
    with h5py.File(str(path),'r') as h:
        return [(np.array(h['model_weights'][n][n]['kernel:0']),np.array(h['model_weights'][n][n]['bias:0'])) for n in ('dense_0','dense_1','q_values')]

def predict(w,states):
    x=np.array([s['cur_phase']+s['coming_vehicle']+s['leaving_vehicle'] for s in states],dtype='float32')
    for i,(k,b) in enumerate(w):
        x=x.dot(k)+b
        if i<2:x=np.maximum(x,0)
    return x

def stats(values):
    x=np.asarray(values);return dict(min=float(x.min()),median=float(np.median(x)),max=float(x.max()),mean=float(x.mean()))

def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();root=a.root
    progress=json.loads((root/'progress.json').read_text());raw=[];coverage=[];train=[]
    for folder in sorted(root.glob('train_*')):
        rows=json.loads((folder/'samples.json').read_text());raw+=rows
        arrivals=json.loads((folder/'demand.json').read_text());stages=json.loads((folder/'transitions.json').read_text())
        train.append(dict(label=folder.name,samples=len(rows),switches=sum(t['stage']=='YELLOW' for t in stages)))
        if len(arrivals)==1:coverage.append(dict(label=folder.name,arrival=arrivals[0],initial_phase=json.loads((folder/'decisions.json').read_text())[0]['phase'],samples=len(rows),terminal=rows[-1]['terminal']))
    # Runner stores the first budget samples across whole episodes.
    replay=raw[:progress['samples']]
    vals=[]
    for folder in sorted(root.glob('val_*')):
        rows=json.loads((folder/'samples.json').read_text());stages=json.loads((folder/'transitions.json').read_text());s=json.loads((folder/'summary.json').read_text())
        vals.append(dict(label=folder.name,sparse='_sparse' in folder.name,rows=rows,switches=sum(t['stage']=='YELLOW' for t in stages),summary=s))
    out=dict(training_scenarios=train,coverage=coverage,replay_samples=len(replay),terminal=sum(r['terminal'] for r in replay),truncated=sum(r['truncated'] for r in replay),
             per_action={str(i):dict(samples=sum(r['action']==i for r in replay),reward=stats([r['reward']/20 for r in replay if r['action']==i])) for i in range(4)},duration=stats([r['duration'] for r in replay]),models={})
    for group in ('regular','sparse'):
        subset=[v for v in vals if v['sparse']==(group=='sparse')];rs=[r for v in subset for r in v['rows']]
        out[group]=dict(actions=[sum(r['action']==i for r in rs) for i in range(4)],switches=[v['switches'] for v in subset],completed=sum(v['summary']['completed'] for v in subset),planned=sum(v['summary']['planned'] for v in subset))
    probes=[v['rows'][-1]['state'] for v in vals if v['sparse'] and int(v['label'].split('sparse')[1])<4]
    for idx in range(4):
        name='round_%03d'%idx;w=weights(root/'models'/(name+'.h5'));q=predict(w,[r['state'] for r in replay]);targetw=weights(root/'models'/'round_000.h5');nq=predict(targetw,[r['next_state'] for r in replay]).max(axis=1)
        targets=np.array([bellman(r,float(v)) for r,v in zip(replay,nq)])
        out['models'][name]=dict(greedy_actions_on_final_replay=np.bincount(q.argmax(axis=1),minlength=4).tolist(),q_by_action=[stats(q[:,i]) for i in range(4)],probe_q=predict(w,probes).tolist(),bellman_target_using_round000=stats(targets),bootstrap=stats([0. if r['terminal'] else r['discount']*float(v) for r,v in zip(replay,nq)]))
    # Four diagnostic single-car observations: final state and observed pressure.
    out['probes']=[dict(label=v['label'],state=v['rows'][-1]['state'],last_decision=json.loads((root/v['label']/'decisions.json').read_text())[-1]) for v in vals if v['sparse'] and int(v['label'].split('sparse')[1])<4]
    # Compute time reward formula and actual decision durations independently.
    errors=[]
    for v in vals:
        for r in v['rows']:
            errors.append(abs(r['discount']-.8**(r['duration']/10)))
    out['max_discount_error']=max(errors)
    out['target_schedule']=['initial network copy','round_000','round_000','round_000']
    out['note']='Round 000 pre-update initial target weights were not saved. Targets here are a diagnostic on final replay, not the historical sampled training targets.'
    (root/'diagnosis.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
if __name__=='__main__':main()
