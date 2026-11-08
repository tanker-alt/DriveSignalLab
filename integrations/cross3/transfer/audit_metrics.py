"""Independent trip/event accounting from raw physics snapshots, never Metrics.report()."""
import argparse
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path
import numpy as np


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def audit(folder):
    demand=json.loads((folder/'demand.json').read_text())
    reported=json.loads((folder/'metrics.json').read_text())
    saved={r['id']:r for r in json.loads((folder/'vehicles.json').read_text())}
    schedule={r['id']:r for r in demand}
    with gzip.open(folder/'measurements.jsonl.gz','rt') as stream:
        frames=[json.loads(line) for line in stream]
    errors=[]
    def check(condition,message):
        if not condition: errors.append(message)
    def equal(a,b,label,tolerance=1e-5):
        check((a is None and b is None) or (a is not None and b is not None and abs(a-b)<=tolerance),label)
    check(bool(frames),'Empty evidence')
    if not frames: return dict(passed=False,errors=errors)
    spawns={}; finishes={}; crossed={}; collisions=set(); previous=0.; geometry_checks=0
    for frame in frames:
        now,dt=frame['time'],frame['dt']
        check(dt>0 and abs(now-dt-previous)<1e-6,'Noncontiguous timestamp')
        before,after=frame['before'],frame['after']
        for vid,v in {**before,**after}.items():
            check(vid in schedule,'Unknown vehicle '+vid)
            if vid not in schedule: continue
            spawn=v['spawn_time']
            if vid in spawns: equal(spawns[vid],spawn,'Changing spawn '+vid)
            spawns[vid]=spawn
            check(spawn>=schedule[vid]['time']-1e-6 and spawn<=now+1e-6,'Invalid spawn '+vid)
            equal(v['speed'],math.sqrt(v['vx']**2+v['vy']**2+v.get('vz',0)**2),'Speed/velocity '+vid,1e-5)
            if frame['backend']=='sumo' and v['approach']:
                equal(v['front_stop_distance'],144-v['raw_position'],'SUMO stop line '+vid)
                geometry_checks+=1
            elif frame['backend']=='carla' and v['approach']:
                front=(v['x']-v['stop_x'])*v['stop_dx']+(v['y']-v['stop_y'])*v['stop_dy']
                heading=math.atan2(v['stop_dy'],v['stop_dx'])
                support=v['half_length']*abs(math.cos(v['yaw']-heading))+v['half_width']*abs(math.sin(v['yaw']-heading))
                equal(v['front_stop_distance'],-front-support,'CARLA stop line '+vid)
                geometry_checks+=1
        for vid in frame['completed']:
            check(vid not in finishes,'Duplicate exit '+vid)
            check(vid in spawns,'Exit without spawn '+vid)
            v=after.get(vid,before.get(vid))
            check(v is not None and v.get('outgoing'),'Exit on wrong road '+vid)
            if v is not None and v.get('exit_front_distance') is not None:
                if vid in after: check(v['exit_front_distance']<=1e-4,'Premature exit '+vid)
                else: check(v['exit_front_distance']<=v['speed']*dt+.5,'Unverified SUMO exit '+vid)
            finishes[vid]=now
        for vid,v in before.items():
            physical=frame['physical_signals'].get('%d/%d'%(v['road'],v['lane']))
            check(frame['colors'].get(vid)==physical,'Signal source mismatch '+vid)
            post=after.get(vid)
            if (post and vid not in crossed and v['approach'] and
                    v['front_stop_distance']>0>=post['front_stop_distance']):
                fraction=v['front_stop_distance']/(v['front_stop_distance']-post['front_stop_distance'])
                crossed[vid]=dict(time=now-dt+fraction*dt,color=physical)
        for pair in frame['collisions']: collisions.add(tuple(sorted(map(str,pair))))
        previous=now
    end=frames[-1]['time']; rows=[]; rear_min=None; rear_exposure=0.; physical_max=external_max=0
    waits={vid:0. for vid in schedule}
    seen=set()
    for frame in frames:
        a,b=frame['time']-frame['dt'],frame['time']
        for vid,v in frame['before'].items():
            low=max(a,spawns[vid]); high=min(b,finishes.get(vid,end))
            if v['speed']<.1: waits[vid]+=max(0.,high-low)
            closing=v['speed']-v['leader_speed']
            if v['gap'] is not None and closing>1e-9:
                ttc=max(0.,v['gap'])/closing
                rear_min=ttc if rear_min is None else min(rear_min,ttc)
                if ttc<1.5: rear_exposure+=frame['dt']
        for vid,t in spawns.items():
            if a-1e-9<=t<b and vid not in frame['before']:
                waits[vid]+=max(0.,min(b,finishes.get(vid,end))-t)
        physical_max=max(physical_max,sum(v['speed']<.1 and finishes.get(vid,float('inf'))>b for vid,v in frame['after'].items()))
        # Spawn visibility is causal: a vehicle in the next frame's `before`
        # did not exist in this frame's `after`, even at the same timestamp.
        seen.update(frame['before']); seen.update(frame['after'])
        external_max=max(external_max,sum(r['time']<=b and vid not in seen for vid,r in schedule.items()))
    due=[r for r in demand if r['time']<=end]
    for item in due:
        vid=item['id']; finish=finishes.get(vid); spawn=spawns.get(vid)
        external=max(0.,min(end,spawn if spawn is not None else end)-item['time'])
        observed=(finish if finish is not None else end)-item['time']
        waiting=external+waits[vid]
        row=dict(vehicle=vid,road=item['road'],lane=item['lane'],scheduled=item['time'],spawn=spawn,
                 stop_line_time=crossed.get(vid,{}).get('time'),stop_line_color=crossed.get(vid,{}).get('color'),
                 finish=finish,completed=finish is not None,observed_time=observed,
                 external_wait=external,in_sim_wait=waits[vid],waiting=waiting)
        rows.append(row)
        for key,value in [('spawn',spawn),('finish',finish),('waiting',waiting),('observed_time',observed)]:
            equal(saved[vid].get(key),value,'Per-vehicle %s: %s'%(key,vid))
    completed=[r for r in rows if r['completed']]
    trips=[r['observed_time'] for r in completed]
    expected=dict(duration_s=end,planned=len(demand),due=len(due),completed=len(completed),
        unfinished=len(due)-len(completed),future_demand=len(demand)-len(due),
        censored_vehicle_time_s=sum(r['observed_time'] for r in rows),
        waiting_vehicle_seconds=sum(r['waiting'] for r in rows),
        external_waiting_vehicle_seconds=sum(r['external_wait'] for r in rows),
        in_sim_waiting_vehicle_seconds=sum(r['in_sim_wait'] for r in rows),
        completed_travel_time_mean_s=float(np.mean(trips)) if trips else None,
        completed_travel_time_p95_s=float(np.percentile(trips,95)) if trips else None,
        throughput_veh_per_hour=len(completed)*3600/end,
        red_light_violations=sum(r['color']=='RED' for r in crossed.values()),
        stop_line_crossings=len(crossed),collision_pairs=len(collisions),
        rear_end_min_ttc_s=rear_min,rear_end_ttc_below_1_5_vehicle_seconds=rear_exposure,
        max_queue_vehicles=physical_max,max_external_queue_vehicles=external_max)
    for key,value in expected.items(): equal(reported.get(key),value,'Aggregate '+key)
    check(geometry_checks>0,'No physical stop-line geometry evidence')
    result=dict(schema='cross3_independent_metric_audit_v1',passed=not errors,errors=errors[:100],
        error_count=len(errors),backend=frames[0]['backend'],frames=len(frames),geometry_checks=geometry_checks,
        independently_recomputed=expected,
        coverage='trip times, censoring, queues, waits, TTC arithmetic, red crossings, signal association and exits',
        crossing_ttc_validation='analytic fixtures and independent static-overlap tests; constant-velocity proxy',
        evidence_sha256={name:digest(folder/name) for name in ['demand.json','metrics.json','vehicles.json','measurements.jsonl.gz']})
    (folder/'metric_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    if rows:
        with (folder/'vehicle_audit.csv').open('w') as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('folder',type=Path);args=parser.parse_args()
    result=audit(args.folder);print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed'] else 1)
