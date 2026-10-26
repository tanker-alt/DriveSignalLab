"""Independent integrity checks and paired uncertainty for completed benchmarks."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np

def audit(root):
    progress=json.loads((root/'progress.json').read_text());report=json.loads((root/'comparison.json').read_text())
    protocol=json.loads((root/'protocol.json').read_text())
    rows=progress['results'];errors=[];paired={}
    for name,digest in protocol.get('adapter_sha256',{}).items():
        if hashlib.sha256((root/'source'/name).read_bytes()).hexdigest()!=digest:errors.append('source_hash '+name)
    for row in rows:
        folder=root/row['label'];arrivals=json.loads((folder/'demand.json').read_text())
        if hashlib.sha256(json.dumps(arrivals).encode()).hexdigest()!=row['demand_sha256']:errors.append('demand_hash '+row['label'])
        if row['label'].startswith('test_'):
            paired.setdefault(row['seed'],{})[row['controller']]=row
    expected={c['seed'] for c in protocol['test']}
    if set(paired)!=expected:errors.append('test_seed_set')
    for seed,controllers in paired.items():
        if len(controllers)!=3:errors.append('missing_controller '+str(seed))
        if len({r['demand_sha256'] for r in controllers.values()})!=1:errors.append('unpaired_demand '+str(seed))
        for r in controllers.values():
            if r['collisions'] or r['signal_mismatches'] or r['completed']!=r['planned']:errors.append('failed_case '+r['label'])
    differences={}
    rng=np.random.RandomState(82)
    for baseline in ('fixed','maxpressure'):
        delta=np.array([c[baseline]['mean_wait_seconds']-c['presslight']['mean_wait_seconds'] for c in paired.values()
                        if baseline in c and 'presslight' in c])
        if not len(delta):
            errors.append('no_paired_cases '+baseline);continue
        means=np.mean(rng.choice(delta,(10000,len(delta)),replace=True),axis=1)
        differences[baseline]=dict(mean_seconds=float(delta.mean()),bootstrap_95_interval=np.percentile(means,[2.5,97.5]).tolist(),
                                  wins=int((delta>0).sum()),cases=len(delta))
    result=dict(integrity_passed=not errors,errors=errors,paired_wait_improvement=differences,
                target_achieved=report['achieved'] and not errors,
                note='Bootstrap over held-out demand cases; one training seed, not a training-seed robustness claim')
    (root/'audit.json').write_text(json.dumps(result,indent=2));return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();print(json.dumps(audit(a.root),indent=2))
