#!/usr/bin/env python3
"""Official PressLight CARLA training, validation selection and held-out comparison."""
import argparse,hashlib,json,os,random,subprocess,sys,time,shutil
from pathlib import Path
import numpy as np
from environment import Environment,demand
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]

class Worker:
    def __init__(self,root,seed,checkpoint=None):
        self.log=(root/'official_worker.log').open('a' if checkpoint else 'w')
        env=dict(os.environ,CUDA_VISIBLE_DEVICES='-1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',TF_CPP_MIN_LOG_LEVEL='2')
        self.p=subprocess.Popen([os.environ['PRESSLIGHT_PYTHON'],'-u',str(HERE/'worker.py')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.log,universal_newlines=True,env=env)
        self.config=self.call(op='init',models=str(root/'models'),seed=seed,checkpoint=checkpoint,resume_replay=bool(checkpoint))
    def call(self,**data):
        self.p.stdin.write(json.dumps(data)+'\n');self.p.stdin.flush()
        line=self.p.stdout.readline()
        if not line:raise RuntimeError('Official worker exited; inspect official_worker.log')
        result=json.loads(line)
        if 'error' in result:raise RuntimeError(result['error'])
        return result
    def close(self):
        if self.p.poll() is None:
            self.p.stdin.write('{"op":"quit"}\n');self.p.stdin.flush();self.p.wait(timeout=30)
        self.log.close()

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--port',type=int,default=31610)
    p.add_argument('--rounds',type=int,default=60);p.add_argument('--smoke',action='store_true');p.add_argument('--resume',action='store_true')
    p.add_argument('--drain-seconds',type=int,default=300);a=p.parse_args()
    if a.rounds<1 or a.drain_seconds<1:p.error('rounds and drain-seconds must be positive')
    root=a.output.resolve();root.mkdir(parents=True,exist_ok=a.resume)
    if a.resume and (root/'comparison.json').exists():raise RuntimeError('Final test already exposed: do not tune further on this test set')
    previous=json.loads((root/'progress.json').read_text()) if a.resume else None
    official=ROOT/'Drive/repos/PressLight_official'
    provenance=dict(commit=subprocess.check_output(['git','-C',str(official),'rev-parse','HEAD']).decode().strip(),
        source='https://github.com/wingsweihua/presslight',sha256={n:hashlib.sha256((official/n).read_bytes()).hexdigest() for n in ('simple_dqn_agent.py','network_agent.py','agent.py','config.py')},
        adapter='Original SimpleDQNAgent and NetworkAgent executed by Python 3.6 worker; CARLA environment replaces CityFlow',
        adaptations={'lane_length':150,'histogram_bins':[0,50,100,150],'phase_bits':8,'incoming_histogram':36,'outgoing_histogram':36,'actions':4,
            'decision_seconds':10,'yellow_seconds':3,'clearance_seconds':2,'clearance_interlock':'until controlled cars and rears leave junction, then 5s green','right_turn':'always-green','queue_threshold_mps':.1,
            'reward':'-.25 * abs(total incoming stopped - total outgoing stopped), interval average','terminal':'official updater bootstraps last state; no added terminal mask'},
        adapter_sha256={n:hashlib.sha256((HERE/n).read_bytes()).hexdigest() for n in ('environment.py','worker.py','schema.py','train.py','audit_results.py')},
        rounds=a.rounds,arrival_horizon=120,drain_seconds=a.drain_seconds,
        phase_bit_order=['WL','WS','EL','ES','NL','NS','SL','SS'],
        action_order=['NS_STRAIGHT','NS_LEFT','EW_STRAIGHT','EW_LEFT'],
        imported_source_sha256={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest()
                               for path in (HERE.parent/'run_cross3.py',
                               HERE.parents[1]/'carla_handoff/upstream_20261023_v4/control_four_phases.py')},
        checkpoint_selection='lowest unfinished fraction, then scenario mean wait; validation only',
        target={'fixed_wait_reduction':.10,'beat_maxpressure':True,'completion_no_worse':True,'zero_collisions':True},
        validation=[{'seed':s,'kind':k} for s,k in ((4101,'balanced'),(4102,'east_west'),(4103,'north_south'),(4104,'left_heavy'))],
        test=[{'seed':s,'kind':k} for s,k in ((7101,'balanced'),(7102,'east_west'),(7103,'north_south'),(7104,'left_heavy'),(7105,'balanced'),(7106,'east_west'))])
    if a.resume:
        if json.loads((root/'protocol.json').read_text())!=provenance:raise RuntimeError('Resume protocol differs')
    else:
        (root/'protocol.json').write_text(json.dumps(provenance,indent=2))
        (root/'source').mkdir()
        for name in provenance['adapter_sha256']:shutil.copy2(HERE/name,root/'source'/name)
    results=previous['results'] if previous else [];history=previous['training'] if previous else []
    best=previous['best'] if previous else None;best_score=previous['best_validation_wait'] if previous else float('inf');fixed_green=previous['fixed_green'] if previous else 20
    best_rank=tuple(previous['best_validation_rank']) if previous else (float('inf'),float('inf'))
    worker=Worker(root,2026,history[-1]['checkpoint'] if history else None)
    if not a.resume:(root/'official_config.json').write_text(json.dumps(worker.config,indent=2))
    def save():
        data=dict(results=results,training=history,best=best,best_validation_wait=best_score,
                  best_validation_rank=best_rank,fixed_green=fixed_green)
        tmp=root/'progress.json.tmp';tmp.write_text(json.dumps(data,indent=2));tmp.replace(root/'progress.json')
    try:
        env=Environment(a.port,root)
        def run(label,seed,kind,controller,epsilon=0.,horizon=120,green=20):
            arrivals=demand(seed,kind,horizon)
            cached=next((r for r in results if r['label']==label),None)
            if cached:
                if cached['demand_sha256']!=hashlib.sha256(json.dumps(arrivals).encode()).hexdigest():raise RuntimeError('Cached demand differs')
                return cached,json.loads((root/label/'samples.json').read_text())
            fixed_step=[0]
            def select(state,pressures,current,t):
                if controller=='fixed':
                    action=(fixed_step[0]//(green//10))%4;fixed_step[0]+=1;return action
                if controller=='maxpressure':
                    m=max(pressures);return current if pressures[current]==m else pressures.index(m)
                return worker.call(op='act',state=state,epsilon=epsilon)['action']
            summary,samples=env.run(root/label,arrivals,select,seed,limit=horizon+a.drain_seconds)
            summary.update(label=label,controller=controller,kind=kind,demand_sha256=hashlib.sha256(json.dumps(arrivals).encode()).hexdigest())
            results.append(summary);save();print(json.dumps(summary),flush=True)
            return summary,samples
        if a.smoke:
            for controller in ('fixed','maxpressure','presslight'):
                summary,samples=run('smoke_'+controller,3100,'balanced',controller,horizon=20)
                if controller=='presslight':worker.call(op='train',samples=samples,round=0)
            return
        # Fixed-time tuning uses validation only, before the final test is exposed.
        tuning={}
        for green in (10,20,30):
            vals=[run('val_fixed%d_%d'%(green,c['seed']),c['seed'],c['kind'],'fixed',green=green)[0] for c in provenance['validation']]
            tuning[green]=sum(v['mean_wait_seconds'] for v in vals)/len(vals) if all(v['completed']==v['planned'] for v in vals) else float('inf')
        fixed_green=min(tuning,key=tuning.get);save()
        for c in provenance['validation']:run('val_maxpressure_%d'%c['seed'],c['seed'],c['kind'],'maxpressure')
        for r in range(len(history),a.rounds):
            kind=('balanced','east_west','north_south','left_heavy')[r%4];epsilon=max(.2,.8*.95**r)
            summary,samples=run('train_%03d'%r,10000+r,kind,'presslight',epsilon)
            info=worker.call(op='train',samples=samples,round=r);history.append(dict(info,epsilon=epsilon,mean_wait=summary['mean_wait_seconds']));save()
            if (r+1)%10==0 or r==a.rounds-1:
                vals=[run('val_round%03d_%d'%(r,c['seed']),c['seed'],c['kind'],'presslight')[0] for c in provenance['validation']]
                score=sum(v['mean_wait_seconds'] for v in vals)/len(vals)
                rank=(sum(v['planned']-v['completed'] for v in vals)/sum(v['planned'] for v in vals),score)
                if rank<best_rank:best_rank=rank;best_score=score;best=info['checkpoint'];save()
                print(json.dumps(dict(validation_round=r,unfinished_fraction=rank[0],mean_wait=score,best=best)),flush=True)
        if best is None:raise RuntimeError('No valid checkpoint; test withheld')
        worker.call(op='load',checkpoint=best)
        tests={c:[] for c in ('fixed','maxpressure','presslight')}
        for c in provenance['test']:
            for controller in tests:
                tests[controller].append(run('test_%s_%d'%(controller,c['seed']),c['seed'],c['kind'],controller,green=fixed_green)[0])
        aggregate={k:dict(mean_wait=sum(v['mean_wait_seconds'] for v in rows)/len(rows),completed=sum(v['completed'] for v in rows),planned=sum(v['planned'] for v in rows),collisions=sum(len(v['collisions']) for v in rows)) for k,rows in tests.items()}
        pl=aggregate['presslight'];fixed=aggregate['fixed'];mp=aggregate['maxpressure']
        fixed_gain=1-pl['mean_wait']/fixed['mean_wait'];mp_gain=1-pl['mean_wait']/mp['mean_wait']
        report=dict(checkpoint=best,aggregate=aggregate,fixed_wait_reduction=fixed_gain,maxpressure_wait_reduction=mp_gain,
            achieved=fixed_gain>=.1 and mp_gain>0 and all(v['completed']==v['planned'] and v['collisions']==0 for v in aggregate.values()),
            validation_unfinished_fraction=best_rank[0],
            scope='Six held-out demands, one training seed; no general convergence claim')
        (root/'comparison.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
        from audit_results import audit
        print(json.dumps(audit(root)),flush=True)
    finally:worker.close()

if __name__=='__main__':main()
