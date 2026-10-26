"""Report multi-seed validation and optional frozen final-test results."""
import argparse
import json
import statistics
from pathlib import Path

def main():
    p=argparse.ArgumentParser();p.add_argument('suite',type=Path);a=p.parse_args()
    rows=[]
    for path in sorted(a.suite.glob('*/progress.json')):
        data=json.loads(path.read_text());protocol=json.loads((path.parent/'protocol.json').read_text())
        best=data['best']
        vals=[r for r in data['results'] if best and r['label'].startswith('val_'+best+'_')]
        groups={}
        for name,subset in [('regular',[r for r in vals if '_sparse' not in r['label']]),('sparse',[r for r in vals if '_sparse' in r['label']])]:
            if subset:
                groups[name]=dict(mean_wait=statistics.mean(r['mean_wait_seconds'] for r in subset),
                    completed=sum(r['completed'] for r in subset),planned=sum(r['planned'] for r in subset),
                    action_counts=[sum(r['action_counts'][i] for r in subset) for i in range(4)])
        tests={}
        for ctrl in ('fixed','maxpressure','presslight'):
            subset=[r for r in data['results'] if r['label'].startswith('test_'+ctrl+'_')]
            if subset:tests[ctrl]=dict(mean_wait=statistics.mean(r['mean_wait_seconds'] for r in subset),completed=sum(r['completed'] for r in subset),planned=sum(r['planned'] for r in subset))
        rows.append(dict(variant=protocol['variant'],seed=protocol['seed'],samples=data['samples'],best=best,validation=groups,test=tests))
    aggregate={}
    for variant in ('legacy','smdp','smdp_coverage'):
        subset=[r for r in rows if r['variant']==variant]
        aggregate[variant]={}
        for group in ('regular','sparse'):
            values=[r['validation'][group]['mean_wait'] for r in subset if group in r['validation']]
            if values:aggregate[variant][group]=dict(seeds=len(values),mean=statistics.mean(values),stdev=statistics.stdev(values) if len(values)>1 else None)
    report=dict(runs=rows,validation_across_seeds=aggregate,warning='Validation selects checkpoints; it is not independent test evidence. Partial runs are included with sample counts.')
    (a.suite/'summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))

if __name__=='__main__':main()
