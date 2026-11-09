"""Evaluate the last available joint checkpoint after an interrupted exploratory study.

This preserves episode failures instead of discarding them from a success claim.
"""
import json,subprocess,sys
from pathlib import Path
from integrations.cross3.transfer.audit_metrics import audit
from integrations.cross3.transfer.stage2 import write,score
from integrations.cross3.transfer.compare import compare
b=Path('outputs/staged/joint_small_noise').resolve();out=b/'after_two_updates';out.mkdir();rows=[];failures=[]
for i,profile in enumerate(('balanced','east_west','north_south','left_heavy')):
 folder=out/('run_%03d'%i)
 cmd=[sys.executable,'-m','integrations.cross3.transfer.run','--backend','carla','--port','31600','--vla','--exploratory-joint','--waypoint-head','/home/zlj/Drive/models/lmdrive/cross3_waypoint_head.pt','--checkpoint',str(b/'train/run_001/checkpoint_000.pt'),'--metrics-acceptance','outputs/staged/stage1_joint_retirement/acceptance.json','--sumo-acceptance',str(b/'sumo_acceptance.json'),'--output',str(folder),'--episodes','1','--seed',str(42001+i),'--demand-profile',profile,'--arrival-seconds','10','--duration','150']
 print('evaluate',i,flush=True)
 with (out/('run_%03d.log'%i)).open('w') as log:r=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT)
 if r.returncode:
  failures.append(dict(seed=42001+i,failure=json.loads((folder/'failure.json').read_text())));write(out/'failures.json',failures);continue
 if not audit(folder/'episode_000')['passed']:raise ValueError('Audit failed')
 rows+=json.loads((folder/'results.json').read_text());write(out/'results.json',rows)
 write(folder/'paired.json',compare([b/'before'/('run_%03d'%i),folder]))
before=json.loads((b/'before/results.json').read_text());matched=[r for r in before if r['seed'] in {x['seed'] for x in rows}]
result=dict(actual_joint_updates=2,planned_joint_updates=4,third_training_episode_failed=json.loads((b/'train/run_002/failure.json').read_text()),before=score(before),after=score(rows) if rows else None,matched_before=score(matched) if matched else None,evaluation_failures=failures,improved=False,scope='Failed exploratory training; evaluate last available checkpoint without hiding failed scenarios')
write(b/'comparison_two_updates.json',result);print(json.dumps(result,indent=2),flush=True)
