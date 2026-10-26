"""Small live contract check; refuses an occupied CARLA world via load_map."""
import argparse
import json
from pathlib import Path
from environment import Environment
from learning import sparse_demand
from train import Worker

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--port',type=int,default=31610)
    a=p.parse_args();root=a.output.resolve();root.mkdir(parents=True,exist_ok=False)
    worker=Worker(root,2026,'smdp',4)
    try:
        env=Environment(a.port,root);rows=[];summaries=[]
        for i in range(4):
            arrivals=sparse_demand(i);arrivals[0]['time']=5.
            def select(state,pressure,current,t):return int(t//10)%4
            summary,batch=env.run(root/('movement_%d'%i),arrivals,select,30000+i,limit=180,initial_phase=(i+1)%4)
            assert summary['completed']==summary['planned']==1,summary
            assert not summary['collisions'] and summary['signal_mismatches']==0
            assert batch[-1]['terminal'] and not batch[-1]['truncated']
            assert all(r['duration']>0 and 0<r['discount']<=1 for r in batch)
            rows.extend(batch);summaries.append(summary)
        info=worker.call(op='train',samples=rows,round=0)
        assert info['trained'],info
        worker.call(op='load',checkpoint=info['checkpoint'])
        action=worker.call(op='act',state=rows[-1]['state'],epsilon=0.)
        report=dict(passed=True,scenarios=summaries,samples=len(rows),training=info,reloaded_action=action)
        (root/'verification.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
    finally:worker.close()

if __name__=='__main__':main()
