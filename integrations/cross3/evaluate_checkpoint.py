#!/usr/bin/env python3
"""Run repeated frozen CARLA evaluations and aggregate summaries."""
import argparse,json,subprocess,sys
from pathlib import Path
def main():
 p=argparse.ArgumentParser();p.add_argument('--controller',required=True);p.add_argument('--checkpoint',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--episodes',type=int,default=10);p.add_argument('--seed',type=int,default=8401);p.add_argument('--port',type=int,default=31600);p.add_argument('--background-per-lane',type=int,default=2);p.add_argument('--duration',type=int,default=180);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
 for i in range(a.episodes):
  out=a.output/('run_%02d'%i); cmd=[sys.executable,'integrations/cross3/run_cross3.py','--controller',a.controller,'--egos','0','--background-per-lane',str(a.background_per_lane),'--duration',str(a.duration),'--seed',str(a.seed+i),'--output',str(out),'--port',str(a.port)]
  if a.checkpoint: cmd += ['--checkpoint',str(a.checkpoint)]
  env=dict(__import__('os').environ); env['CROSS3_PYTHON']=env.get('CROSS3_PYTHON',sys.executable); subprocess.run(cmd,check=True,env=env)
 rows=[]
 for pth in sorted(a.output.glob('run_*/summary.json')):
  d=json.loads(pth.read_text());d=d[0] if isinstance(d,list) else d; times=[v['completion_seconds'] for v in d.get('vehicles',[]) if v.get('completed') and 'completion_seconds' in v]; rows.append(dict(completed=d.get('completed',0),planned=d.get('planned',0),collisions=len(d.get('collisions',[])),signal_mismatches=d.get('signal_mismatches',0),elapsed=d.get('elapsed'),mean_completion_s=sum(times)/len(times) if times else None))
 out={'controller':a.controller,'checkpoint':str(a.checkpoint) if a.checkpoint else None,'seed_start':a.seed,'episodes':rows,'completion_rate':sum(x['completed'] for x in rows)/max(1,sum(x['planned'] for x in rows)),'collision_episodes':sum(bool(x['collisions']) for x in rows),'mean_completion_s':sum(x['mean_completion_s'] for x in rows if x['mean_completion_s'] is not None)/len([x for x in rows if x['mean_completion_s'] is not None])};(a.output/'evaluation.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
if __name__=='__main__':main()
