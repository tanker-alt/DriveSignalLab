"""Stress episode reset/native-light synchronization without traffic confounding."""
import argparse
import json
from pathlib import Path
from environment import Environment


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--port',type=int,default=31610);p.add_argument('--repeats',type=int,default=40)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    env=Environment(args.port,args.output);rows=[]
    for i in range(args.repeats):
        # Future arrival keeps an empty world alive through yellow/red/new green.
        summary,_=env.run(args.output/('reset_%03d'%i),[dict(time=100,road=1,lane=-2)],
                          lambda s,p,c,t:i%4,i,limit=6)
        assert not summary['collisions'] and summary['signal_mismatches']==0,summary
        rows.append(summary)
    result=dict(passed=True,episodes=len(rows),initial_actions=[0,1,2,3],
                max_initial_settle_frames=max(r['initial_signal_settle_frames'] for r in rows),
                transition_lag_frames=sum(r['signal_lag_frames'] for r in rows))
    (args.output/'regression.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)


if __name__=='__main__':main()
