"""Read frozen replay and output missing quotas; never collect or fit."""
import argparse,json
from pathlib import Path
from coverage_quota import combination,counts,deficit,gate,schedule,DEFAULT_QUOTAS

def main():
    p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    manifest=Path(__file__).resolve().parents[2]/'carla_handoff/upstream_20261023_v4/map_dataset/junction_toolkit/maps/cross3/manifest.json'
    keys=[(m['road'],m['lane']) for m in json.loads(manifest.read_text())['movements']]
    budget=json.loads((a.input/'progress.json').read_text())['samples'];retained=0;episodes=[]
    for folder in sorted(a.input.glob('train_*')):
        rows=json.loads((folder/'samples.json').read_text());take=rows[:max(0,budget-retained)];retained+=len(rows)
        d=json.loads((folder/'demand.json').read_text());phase=json.loads((folder/'decisions.json').read_text())[0]['phase']
        episodes.append((d,phase,take,folder.name))
    entries=[]
    for index in range(32):
        spec=combination(index);rows=[];labels=[]
        for arrivals,phase,take,label in episodes:
            if len(arrivals)==1 and phase==spec['initial_phase'] and (arrivals[0]['road'],arrivals[0]['lane'])==(spec['road'],spec['lane']):
                rows+=take;labels.append(label)
        actual=counts(rows,spec,keys);entries.append(dict(spec,counts=actual,deficit=deficit(actual),episodes=labels))
    result=dict(status='offline plan only',source=str(a.input.resolve()),quotas=DEFAULT_QUOTAS,
                required_combinations=32,passed=sum(not any(e['deficit'].values()) for e in entries),
                gate_passed=gate(entries),entries=entries,next_cycle=schedule(entries),
                total_deficits={k:sum(e['deficit'][k] for e in entries) for k in DEFAULT_QUOTAS},
                warning='Deficits overlap: observed/served/alternative are not independent sample totals. Passing is necessary screening only.')
    (a.output/'coverage_plan.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
if __name__=='__main__':main()
