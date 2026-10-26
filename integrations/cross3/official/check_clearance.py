"""Live CARLA regression: old collision actions and all directed phase switches."""
import argparse
import json
from pathlib import Path
from environment import Environment,demand


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--port',type=int,default=31610)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    root=Path(__file__).resolve().parents[4]
    old=json.loads((root/'Drive/outputs/cross3/official_training_v2/train_019/decisions.json').read_text())
    env=Environment(args.port,args.output);results=[]
    # Euler tour of the complete directed four-phase graph: every ordered pair.
    cycle=[0,1,0,2,0,3,1,2,1,3,2,3]
    for name,seed,kind in [('collision_replay',10019,'left_heavy'),
                            ('all_switches',4102,'east_west')]:
        counter=[0]
        def select(state,pressures,current,t):
            i=counter[0];counter[0]+=1
            if name=='collision_replay' and i<len(old):return old[i]['action']
            return cycle[i%len(cycle)]
        summary,_=env.run(args.output/name,demand(seed,kind),select,seed,limit=420)
        assert summary['completed']==summary['planned'] and not summary['collisions'],summary
        assert summary['signal_mismatches']==0,summary
        stages=json.loads((args.output/name/'transitions.json').read_text())
        greens=[s for s in stages if s['stage']=='GREEN']
        for green in greens[1:]:
            assert not green['blockers'] and green['switch_age']>=5,green
        if name=='all_switches':
            pairs={(a['phase'],b['phase']) for a,b in zip(greens,greens[1:])}
            assert {(a,b) for a in range(4) for b in range(4) if a!=b}<=pairs,pairs
        summary['green_releases_checked']=len(greens)-1;results.append(summary)
        print(json.dumps(dict(case=name,summary=summary)),flush=True)
    (args.output/'regression.json').write_text(json.dumps(dict(passed=True,cases=results),indent=2))


if __name__=='__main__':main()
