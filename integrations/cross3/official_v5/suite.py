"""Sequential, equal-budget 3 variants x 3 seeds; no concurrent CARLA ownership."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

HERE=Path(__file__).resolve().parent

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--samples',type=int,default=18000);p.add_argument('--port',type=int,default=31610)
    p.add_argument('--plan-only',action='store_true');p.add_argument('--coverage-plan',type=Path);a=p.parse_args()
    if not a.plan_only:
        from coverage_quota import require_coverage
        require_coverage(a.coverage_plan)
    root=a.output.resolve();root.mkdir(parents=True,exist_ok=False)
    jobs=[dict(variant=v,seed=s,status='pending') for s in (2026,2027,2028) for v in ('legacy','smdp','smdp_coverage')]
    def save():
        temp=root/'suite.tmp';temp.write_text(json.dumps(dict(samples_per_job=a.samples,jobs=jobs),indent=2));temp.replace(root/'suite.json')
    save()
    if a.plan_only:return
    for job in jobs:
        label='%s_%d'%(job['variant'],job['seed'])
        command=[sys.executable,str(HERE/'train.py'),'--output',str(root/label),'--variant',job['variant'],'--seed',str(job['seed']),'--samples',str(a.samples),'--port',str(a.port),'--coverage-plan',str(a.coverage_plan.resolve())]
        job['status']='running';save()
        with (root/(label+'.log')).open('w') as log:
            result=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
        job['exit_code']=result.returncode;job['status']='finished' if result.returncode==0 else 'failed';save()
        if result.returncode:raise RuntimeError('Stopped scheduling after failed job '+label)

if __name__=='__main__':main()
