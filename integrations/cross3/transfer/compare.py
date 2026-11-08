"""Paired evaluation report; reject mismatched traffic and training rollouts."""
import argparse
import json
from pathlib import Path
import numpy as np

def compare(paths):
    runs=[]
    for path in paths:
        protocol=json.loads((path/'protocol.json').read_text())
        args=protocol['arguments']
        if args['train']: raise ValueError('Use frozen evaluation, not training rollouts: '+str(path))
        rows=json.loads((path/'results.json').read_text())
        demands={r['seed']:json.loads((path/('episode_%03d'%i)/'demand.json').read_text()) for i,r in enumerate(rows)}
        runs.append((path,protocol,{r['seed']:r for r in rows},demands))
    reference=runs[0]; output=[]
    for path,protocol,rows,demands in runs:
        if protocol['contract']!=reference[1]['contract']: raise ValueError('Contract mismatch')
        if demands!=reference[3]: raise ValueError('Demand/seed mismatch: '+str(path))
        for key in ('duration','arrival_seconds','rate','vla','waypoint_head'):
            if protocol['arguments'][key]!=reference[1]['arguments'][key]: raise ValueError('Evaluation protocol mismatch: '+key)
        stats={}
        for key in ('completion_rate','mean_observed_wait_s','observed_time_per_due_vehicle_s','throughput_veh_per_hour'):
            values=[r[key] for r in rows.values() if r[key] is not None]
            deltas=[rows[s][key]-reference[2][s][key] for s in rows
                    if rows[s][key] is not None and reference[2][s][key] is not None]
            stats[key]=dict(mean=float(np.mean(values)) if values else None,
                paired_delta_vs_first=float(np.mean(deltas)) if deltas else None,
                paired_delta_std=float(np.std(deltas,ddof=1)) if len(deltas)>1 else None)
        output.append(dict(path=str(path),backend=protocol['arguments']['backend'],seeds=list(rows),
            collision_pairs=sum(r['collision_pairs'] for r in rows.values()),unfinished=sum(r['unfinished'] for r in rows.values()),metrics=stats))
    return dict(runs=output,limitations=['Small pilot samples do not establish improvement.',
        'Completed-trip means exclude censored trips; compare completion and observed vehicle time together.',
        'Red-light and crossing-conflict metrics are unavailable, not zero.',
        'SUMO/CARLA share topology and demand; driving dynamics and stop-line geometry remain domain differences.'])

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('runs',type=Path,nargs='+');p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    a.output.write_text(json.dumps(compare(a.runs),indent=2)+'\n')
