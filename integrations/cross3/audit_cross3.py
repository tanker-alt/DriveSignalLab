#!/usr/bin/env python3
"""Audit saved Cross3 runs; incomplete driving never counts as acceptance."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys


def audit(folder):
    config=json.loads((folder/'config.json').read_text())
    load=json.loads((folder/'map_load.json').read_text())
    manifest_path=Path(__file__).resolve().parents[1]/'carla_handoff/upstream_20261023_v4/map_dataset/junction_toolkit/maps/cross3/manifest.json'
    manifest=json.loads(manifest_path.read_text())
    phase_lanes={'NS_STRAIGHT':{(2,-2),(4,-2)},'NS_LEFT':{(2,-1),(4,-1)},
                 'EW_STRAIGHT':{(1,-2),(3,-2)},'EW_LEFT':{(1,-1),(3,-1)}}
    episodes=[]
    for path in sorted(folder.glob('episode_*/summary.json')):
        d=json.loads(path.read_text())
        mismatches=0
        applying=0
        semantic_errors=0
        frames=0
        for line in (path.parent/'signals.jsonl').read_text().splitlines():
            r=json.loads(line)
            frames+=1
            stage=r['stage'][9:] if r['stage'].startswith('APPLYING_') else r['stage']
            active=phase_lanes.get(r['phase'],set())
            expected={}
            for sid,cfg in manifest['signals'].items():
                road,lane=cfg['road_id'],cfg['incoming_lanes'][0]
                if lane==-3 and config['right_turn']=='always-green':
                    color='Green'
                elif ((road,lane) in active or (lane==-3 and config['right_turn']=='with-straight'
                        and (road,-2) in active)) and stage in ('GREEN','YELLOW'):
                    color=stage.title()
                else:
                    color='Red'
                expected[sid]=color
            semantic_errors+=expected!=r['desired']
            if r['stage'].startswith('APPLYING_'):
                applying+=1
            else:
                mismatches+=r['desired']!=r['actual']
        vehicles=d['vehicles']
        episodes.append(dict(episode=path.parent.name,status=d['status'],
            planned=len(vehicles),completed=sum(v['completed'] for v in vehicles),
            ego_completed=sum(v['completed'] for v in vehicles if v['role'].startswith('hero')),
            wrong_exits=sum(v['route_violation'] for v in vehicles),
            collision_callbacks=len(d['collisions']),frames=frames,signal_mismatches=mismatches,
            application_lag_frames=applying,
            phase_semantic_errors=semantic_errors,
            phases_seen=d.get('phases_seen',[]),
            accepted=d['status']=='finished' and bool(vehicles) and all(v['completed'] and not v['route_violation'] for v in vehicles)
                     and not d['collisions'] and mismatches==0 and semantic_errors==0))
    calls=Counter()
    adaptations=[]
    for f in folder.glob('lmdrive/*/model_audit.jsonl'):
        for line in f.read_text().splitlines():
            r=json.loads(line)
            if r['type']=='model_output': calls[str(r['ego_index'])]+=1
    for f in folder.glob('lmdrive/*/experiment_config.json'):
        d=json.loads(f.read_text())
        adaptations.append(d.get('waypoint_head'))
    passed=(len(episodes)==config['episodes'] and all(e['accepted'] for e in episodes)
            and (config['egos']==0 or all(calls[str(i)]>0 for i in range(config['egos']))))
    return dict(accepted=passed,vla_accepted=passed and config['egos']==2 and not config.get('teacher',False),
                driver='BasicAgent_expert' if config.get('teacher') else 'LMDrive' if config['egos'] else 'background_only',
                config=config,map=load,episodes=episodes,
                model_calls=dict(calls),waypoint_heads=adaptations,
                limits='Harness endpoints and manifest exits; no official Leaderboard score or complete red-light certification')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run',type=Path)
    a=p.parse_args()
    report=audit(a.run)
    (a.run/'audit.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    sys.exit(0 if report['accepted'] else 1)
