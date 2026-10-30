#!/usr/bin/env python3
import argparse,json,statistics
from pathlib import Path

def main():
 p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args(); rows=[]
 for s in sorted(a.root.glob('*/summary.json')):
  data=json.loads(s.read_text()); data=data[0] if isinstance(data,list) else data
  sig=s.parent/'episode_000'/'signals.jsonl'; cost=stopped=0.; frames=0
  if sig.exists():
   for line in sig.open():
    o=json.loads(line).get('observation',{}); stopped+=float(o.get('stopped_incoming',0)); cost+=-.25*abs(float(o.get('stopped_incoming',0))-float(o.get('stopped_outgoing',0))); frames+=1
  times=[v['completion_seconds'] for v in data.get('vehicles',[]) if v.get('completed') and 'completion_seconds' in v]
  rows.append(dict(run=s.parent.name,status=data.get('status'),completed=data.get('completed',0),planned=data.get('planned',0),collisions=len(data.get('collisions',[])),signal_mismatches=data.get('signal_mismatches',0),phases=data.get('phases_seen',[]),mean_completion_s=statistics.mean(times) if times else None,presslight_reward=cost,mean_stopped=stopped/frames if frames else None))
 out={'runs':rows,'aggregate':{}}
 if rows:
  for k in ('completed','planned','collisions','signal_mismatches','mean_completion_s','presslight_reward','mean_stopped'):
   vals=[r[k] for r in rows if isinstance(r[k],(int,float))]
   out['aggregate'][k]=statistics.mean(vals) if vals else None
  out['aggregate']['completion_rate']=sum(r['completed'] for r in rows)/max(1,sum(r['planned'] for r in rows))
 print(json.dumps(out,ensure_ascii=False,indent=2)); (a.root/'summary_aggregate.json').write_text(json.dumps(out,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
