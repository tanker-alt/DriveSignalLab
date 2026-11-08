"""Detect impossible position jumps in raw physical snapshots.

The 0.5 m margin accommodates discrete contact dynamics and body-centre rotation;
this is a data-quality check, not a collision predictor.
"""
import argparse
import gzip
import json
import math
from pathlib import Path


def check_frames(frames,margin=.5):
    pairs=0;errors=[];previous={}
    def check(a,b,dt,time,vid,kind):
        nonlocal pairs
        pairs+=1
        distance=math.hypot(a['x']-b['x'],a['y']-b['y'])
        bound=max(a['speed'],b['speed'])*dt+margin
        if distance>bound:
            errors.append(dict(time=time,vehicle=vid,kind=kind,displacement=distance,bound=bound))
    for frame in frames:
        for vid,v in frame['before'].items():
            if vid in previous: check(previous[vid],v,0.,frame['time']-frame['dt'],vid,'between_snapshots')
            if vid in frame['after']: check(v,frame['after'][vid],frame['dt'],frame['time'],vid,'during_tick')
        previous=frame['after']
    return dict(passed=not errors,position_pairs=pairs,error_count=len(errors),errors=errors[:100],margin_m=margin)


def audit(path):
    with gzip.open(path,'rt') as stream:
        result=check_frames(json.loads(line) for line in stream)
    return dict(path=str(path),**result)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('root',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    files=[a.root] if a.root.is_file() else sorted(a.root.rglob('measurements.jsonl.gz'))
    if not files: p.error('No physical measurements found')
    results=[audit(path) for path in files]
    result=dict(passed=all(r['passed'] for r in results),runs=results)
    a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(passed=result['passed'],runs=len(results),errors=sum(r['error_count'] for r in results))))
    raise SystemExit(0 if result['passed'] else 1)
