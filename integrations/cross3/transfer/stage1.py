"""Run analytic tests and independent audits in both real simulators before training."""
import argparse
import json
import os
import subprocess
import sys
from pathlib import Path
from .audit_metrics import audit
from .stages import metric_sources,sha


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--port',type=int,default=31600)
    a=p.parse_args();a.output=a.output.resolve();a.output.mkdir(parents=True,exist_ok=False)
    sources=metric_sources()
    with (a.output/'unit_tests.log').open('w') as stream:
        test=subprocess.run([sys.executable,'-m','unittest','integrations.cross3.transfer.test_transfer',
            'integrations.cross3.transfer.test_metrics_stage1','integrations.cross3.transfer.test_stages','-v'],stdout=stream,stderr=subprocess.STDOUT)
    if test.returncode: raise RuntimeError('Stage 1 analytic tests failed')
    cases=[]; artifacts={str(a.output/'unit_tests.log'):sha(a.output/'unit_tests.log')}
    for backend in ('sumo','carla'):
        for controller,seed,duration in [('fixed',12001,150),('fixed',12002,150),
                                         ('maxpressure',12001,150),('maxpressure',12002,150),('fixed',12003,8)]:
            name='%s_%s_%d_%d'%(backend,controller,seed,duration)
            out=a.output/name
            cmd=[sys.executable,'-m','integrations.cross3.transfer.run','--backend',backend,
                 '--controller',controller,'--seed',str(seed),'--arrival-seconds','6' if duration==8 else '10',
                 '--duration',str(duration),'--output',str(out),'--port',str(a.port)]
            print('Stage 1:',name,flush=True)
            with (a.output/(name+'.log')).open('w') as stream:
                proc=subprocess.run(cmd,stdout=stream,stderr=subprocess.STDOUT)
            if proc.returncode: raise RuntimeError('Physical run failed; inspect '+str(a.output/(name+'.log')))
            folder=out/'episode_000'; result=audit(folder)
            report=json.loads((folder/'metrics.json').read_text())
            case=dict(name=name,audit_passed=result['passed'],frames=result['frames'],completed=report['completed'],
                      unfinished=report['unfinished'],red_light_violations=report['red_light_violations'],
                      collision_pairs=report['collision_pairs'])
            cases.append(case)
            for filename in ('demand.json','metrics.json','vehicles.json','measurements.jsonl.gz','metric_audit.json','vehicle_audit.csv'):
                path=folder/filename;artifacts[str(path)]=sha(path)
            passed=result['passed'] and (report['unfinished']>0 if duration==8 else report['unfinished']==0)
            # A valid detector may find actual violations. Baselines must also be safe to train against.
            passed=passed and report['red_light_violations']==0 and report['collision_pairs']==0
            (a.output/'progress.json').write_text(json.dumps(dict(cases=cases,latest_passed=passed),indent=2))
            if not passed: raise RuntimeError('Stage 1 not accepted: '+name+'; retain evidence and fix the cause')
    if sources!=metric_sources(): raise RuntimeError('Source changed while stage 1 was running')
    receipt=dict(stage=1,passed=True,metric_sources=sources,artifacts=artifacts,cases=cases,
                 scope='analytic safety cases + independent per-vehicle integration and physical baselines',
                 limitations=['Crossing TTC is a constant-velocity/fixed-heading risk proxy, not future ground truth.'])
    (a.output/'acceptance.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print('Stage 1 accepted:',a.output/'acceptance.json',flush=True)

if __name__=='__main__':main()
