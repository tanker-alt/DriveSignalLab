"""Explain service deficits using measured lamp durations and per-vehicle outcomes."""
import argparse
import gzip
import json
from pathlib import Path

GROUPS={'NS_STRAIGHT':('2/-2','4/-2','2/-3','4/-3'),
        'NS_LEFT':('2/-1','4/-1'),'EW_STRAIGHT':('1/-2','3/-2','1/-3','3/-3'),
        'EW_LEFT':('1/-1','3/-1')}


def diagnose(folder):
    vehicles=json.loads((folder/'vehicles.json').read_text())
    metrics=json.loads((folder/'metrics.json').read_text())
    green={key:0. for keys in GROUPS.values() for key in keys}
    with gzip.open(folder/'measurements.jsonl.gz','rt') as stream:
        for line in stream:
            record=json.loads(line)
            for key,color in record['physical_signals'].items():
                if color=='GREEN': green[key]+=record['dt']
    groups={}
    for group,keys in GROUPS.items():
        due=[r for r in vehicles if '%d/%d'%(r['road'],r['lane']) in keys and r['time']<=metrics['duration_s']]
        groups[group]=dict(due=len(due),completed=sum(r['finish'] is not None for r in due),
            unfinished=sum(r['finish'] is None for r in due),
            mean_wait_s=sum(r['waiting'] for r in due)/len(due) if due else None,
            max_wait_s=max((r['waiting'] for r in due),default=None),
            green_seconds_by_lane={k:green[k] for k in keys},
            no_green_despite_demand=bool(due) and all(green[k]==0 for k in keys))
    result=dict(groups=groups,collisions=metrics['collision_pairs'],red_crossings=metrics['red_light_violations'],
        warnings=['Unfinished trips are censored; this report does not prove which model component caused a delay.'])
    (folder/'service_diagnosis.json').write_text(json.dumps(result,indent=2)+'\n')
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('folder',type=Path);a=p.parse_args()
    print(json.dumps(diagnose(a.folder),indent=2))
