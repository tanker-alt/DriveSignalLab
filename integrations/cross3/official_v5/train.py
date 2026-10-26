#!/usr/bin/env python3
"""Isolated v5 experiment. One variant/seed per invocation; never resumes v4."""
import argparse, hashlib, json, os, random, shutil, subprocess, sys
from pathlib import Path
from learning import epsilon, sparse_demand
HERE=Path(__file__).resolve().parent

class Worker:
    def __init__(self, root, seed, variant, warmup, target_lag=5):
        self.log=(root/'worker.log').open('w')
        env=dict(os.environ,CUDA_VISIBLE_DEVICES='-1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',TF_CPP_MIN_LOG_LEVEL='2')
        self.p=subprocess.Popen([os.environ['PRESSLIGHT_PYTHON'],'-u',str(HERE/'worker.py')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.log,universal_newlines=True,env=env)
        self.config=self.call(op='init',models=str(root/'models'),seed=seed,variant=variant,warmup=warmup,target_lag_updates=target_lag)
    def call(self, **data):
        self.p.stdin.write(json.dumps(data)+'\n');self.p.stdin.flush()
        line=self.p.stdout.readline()
        if not line:raise RuntimeError('Worker exited; see worker.log')
        result=json.loads(line)
        if 'error' in result:raise RuntimeError(result['error'])
        return result
    def close(self):
        if self.p.poll() is None:
            self.p.stdin.write('{"op":"quit"}\n');self.p.stdin.flush();self.p.wait(timeout=30)
        self.log.close()

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--variant',choices=['legacy','smdp','smdp_coverage'],default='smdp_coverage')
    p.add_argument('--seed',type=int,choices=[2026,2027,2028],default=2026)
    p.add_argument('--port',type=int,default=31610)
    p.add_argument('--samples',type=int,default=18000)
    p.add_argument('--training-horizon',type=int,default=600)
    p.add_argument('--update-every',type=int,default=300)
    p.add_argument('--warmup',type=int,default=1000)
    p.add_argument('--target-lag-updates',type=int,default=5)
    p.add_argument('--decay-samples',type=int,default=12000)
    p.add_argument('--validate-every',type=int,default=3000)
    p.add_argument('--final-test',action='store_true',help='Expose fresh test only after protocol and hyperparameters are frozen')
    p.add_argument('--plan-only',action='store_true')
    p.add_argument('--coverage-plan',type=Path)
    a=p.parse_args()
    if min(a.samples,a.update_every,a.warmup,a.decay_samples,a.validate_every,a.training_horizon,a.target_lag_updates)<=0:p.error('Budgets must be positive')
    if not a.plan_only:
        from coverage_quota import require_coverage
        require_coverage(a.coverage_plan)
    root=a.output.resolve();root.mkdir(parents=True,exist_ok=False)
    protocol=dict(version=5,variant=a.variant,seed=a.seed,budget=a.samples,update_every=a.update_every,
                  warmup=a.warmup,epsilon_decay_samples=a.decay_samples,validate_every=a.validate_every,
                  target_lag_updates=a.target_lag_updates,epoch_selection='minimum finite fit validation loss; restore best weights',
                  target='discounted tick integral / 10 / 20 + .8^(duration/10) * Qnext; true terminal masked',
                  reward='unchanged -.25 * abs(incoming stopped - outgoing stopped)',
                  training_horizon=a.training_horizon,drain_seconds=300,
                  coverage='every fifth episode is late single-car; 32 phase/approach/initial-phase combinations' if a.variant=='smdp_coverage' else 'standard four traffic types',
                  validation_seeds=[8401,8402,8403,8404],test_seeds=[9401,9402,9403,9404,9405,9406],
                  modified_official='Original network/optimizer; adapter replaces replay Bellman target. legacy keeps v4 target for ablation.',
                  source_sha256={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in HERE.glob('*.py')})
    (root/'protocol.json').write_text(json.dumps(protocol,indent=2))
    shutil.copytree(HERE,root/'source',ignore=shutil.ignore_patterns('__pycache__'))
    if a.plan_only:return
    # Separate process lock prevents two v5 runners from sharing a simulator.
    # load_map additionally refuses a synchronous or occupied world. This does
    # not stop or take over any pre-existing training process.
    import fcntl
    lock=open('/tmp/presslight_cross3_port_%d.lock'%a.port,'w')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    from environment import Environment,demand
    random.seed(a.seed)
    worker=Worker(root,a.seed,'legacy' if a.variant=='legacy' else 'smdp',a.warmup,a.target_lag_updates)
    results=[];history=[];best=None;best_rank=(float('inf'),float('inf'));count=0;episode=0;update=0
    kinds=['balanced','east_west','north_south','left_heavy']
    def save():
        data=dict(samples=count,episodes=episode,training=history,results=results,best=best,best_rank=best_rank)
        tmp=root/'progress.tmp';tmp.write_text(json.dumps(data,indent=2));tmp.replace(root/'progress.json')
    try:
        env=Environment(a.port,root)
        (root/'worker_config.json').write_text(json.dumps(worker.config,indent=2))
        def run(label,arrivals,seed,controller='presslight',training=False,initial=0,slot=20,training_offset=0,limit=420):
            steps=[0]
            def select(state,pressures,current,t):
                if controller=='fixed':
                    action=(steps[0]//(slot//10))%4;steps[0]+=1;return action
                if controller=='maxpressure':
                    return current if pressures[current]==max(pressures) else pressures.index(max(pressures))
                return act(state,steps) if training else worker.call(op='act',state=state,epsilon=0.)['action']
            def act(state,steps):
                action=worker.call(op='act',state=state,epsilon=epsilon(count+training_offset+steps[0],a.decay_samples))['action']
                steps[0]+=1;return action
            summary,rows=env.run(root/label,arrivals,select,seed,limit=limit,initial_phase=initial)
            summary.update(label=label,controller=controller,action_counts=[sum(r['action']==i for r in rows) for i in range(4)])
            results.append(summary);print(json.dumps(summary),flush=True)
            return summary,rows
        def validation(checkpoint):
            vals=[run('val_%s_%d'%(checkpoint,8401+i),demand(8401+i,k),8401+i)[0] for i,k in enumerate(kinds)]
            vals += [run('val_%s_sparse%d'%(checkpoint,i),sparse_demand(i),8600+i,initial=(i//8)%4)[0] for i in range(32)]
            return (sum(v['planned']-v['completed'] for v in vals)/sum(v['planned'] for v in vals),sum(v['mean_wait_seconds'] for v in vals)/len(vals))
        next_validation=a.validate_every
        while count<a.samples:
            rows=[]
            while len(rows)<min(a.update_every,a.samples-count):
                seed=20000+episode
                sparse=a.variant=='smdp_coverage' and episode%5==4
                index=episode//5
                arrivals=sparse_demand(index) if sparse else demand(seed,kinds[(episode-episode//5)%4] if a.variant=='smdp_coverage' else kinds[episode%4],a.training_horizon)
                _,batch=run('train_%05d'%episode,arrivals,seed,training=True,initial=(index//8)%4 if sparse else 0,training_offset=len(rows),limit=420 if sparse else a.training_horizon+300)
                # Equal replay budgets, including the final partial episode.
                rows.extend(batch[:a.samples-count-len(rows)]);episode+=1
            info=worker.call(op='train',samples=rows,round=update);count+=len(rows);update+=1
            history.append(dict(info,samples=count,epsilon=epsilon(count,a.decay_samples)));save()
            if (count>=next_validation or count>=a.samples) and info['trained']:
                rank=validation(info['checkpoint'])
                if rank<best_rank:best_rank=rank;best=info['checkpoint']
                next_validation=count+a.validate_every;save()
        if a.final_test:
            if best is None:raise RuntimeError('No trained checkpoint')
            # Tune fixed slots only on validation; same safety and arrivals.
            tuning={}
            for slot in (10,20,30):
                vals=[run('fixed_val_%d_%d'%(slot,i),demand(8401+i,k),8401+i,'fixed',slot=slot)[0] for i,k in enumerate(kinds)]
                tuning[slot]=(sum(v['planned']-v['completed'] for v in vals),sum(v['mean_wait_seconds'] for v in vals))
            slot=min(tuning,key=tuning.get);worker.call(op='load',checkpoint=best)
            for i in range(6):
                for ctrl in ('fixed','maxpressure','presslight'):
                    run('test_%s_%d'%(ctrl,9401+i),demand(9401+i,kinds[i%4]),9401+i,ctrl,slot=slot)
            save()
    finally:
        worker.close();lock.close()

if __name__=='__main__':main()
