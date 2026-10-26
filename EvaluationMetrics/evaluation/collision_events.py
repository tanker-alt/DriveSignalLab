"""Deterministic pair-contact episodes from preserved CARLA callbacks.

Unordered actor pair + adjacent simulation-time gap. Not responsibility,
multi-party crash reconstruction, or proof of physical separation.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
from datetime import datetime, timezone
from uuid import uuid4


def validate_gap(seconds):
    if not math.isfinite(seconds) or seconds < 0:
        raise ValueError('Collision merge gap must be finite and nonnegative')


def aggregate(records, merge_gap_seconds):
    validate_gap(merge_gap_seconds)
    callbacks=[];unresolved=[];attached={};coverage_rows={};ends=[];failures=[];closed=False
    for number,record in enumerate(records,1):
        kind=record.get('event')
        if kind=='collision_callback':
            time=float(record['sim_time']);frame=int(record['frame'])
            if not math.isfinite(time) or time<0 or frame<0:
                raise ValueError('Invalid collision time/frame at source line '+str(number))
            magnitude=float(record['impulse_magnitude'])
            if not math.isfinite(magnitude) or magnitude<0:
                raise ValueError('Invalid collision impulse at source line '+str(number))
            item=dict(record,source_line=number,sim_time=time,frame=frame)
            a,b=int(record['vehicle_id']),int(record['other_actor_id'])
            if min(a,b)<=0 or a==b:
                unresolved.append(dict(source_line=number,reason='unresolved_or_self_actor_identity',callback=record))
            else:
                callbacks.append((tuple(sorted((a,b))),item))
        elif kind=='sensor_attached':
            attached[int(record['vehicle_id'])]=int(record.get('earliest_eligible_frame',int(record['frame'])+1))
        elif kind=='coverage_frame':
            frame=int(record['frame'])
            if frame in coverage_rows:
                raise ValueError('Duplicate collision coverage frame')
            coverage_rows[frame]=record
        elif kind=='coverage_ended':
            ends.append(record)
        elif kind in ('sensor_attach_failed','sensor_listen_failed','sensor_cleanup_error'):
            failures.append(record)
        elif kind=='recording_closed':
            closed=True
    callbacks.sort(key=lambda pair:(pair[0],pair[1]['sim_time'],pair[1]['frame'],pair[1]['source_line']))
    groups=[]
    for pair,item in callbacks:
        if (not groups or groups[-1]['actor_ids']!=list(pair) or
                item['sim_time']-groups[-1]['callbacks'][-1]['sim_time']>merge_gap_seconds+1e-9):
            groups.append(dict(actor_ids=list(pair),callbacks=[]))
        groups[-1]['callbacks'].append(item)
    events=[]
    for group in groups:
        rows=group['callbacks'];reporters=sorted({int(r['vehicle_id']) for r in rows})
        types={str(actor):set() for actor in group['actor_ids']}
        for row in rows:
            types[str(row['vehicle_id'])].add(row.get('vehicle_type','vehicle.unspecified'))
            types[str(row['other_actor_id'])].add(row.get('other_actor_type','unknown'))
        all_types={t for ts in types.values() for t in ts}
        category=('vehicle_pedestrian' if any(t.startswith('walker.') for t in all_types) else
                  'vehicle_vehicle' if all(all(t.startswith('vehicle.') for t in ts) for ts in types.values()) else
                  'vehicle_static' if any(t.startswith(('static.','traffic.')) for t in all_types) else 'vehicle_other_or_unknown')
        events.append(dict(actor_ids=group['actor_ids'],start_frame=min(r['frame'] for r in rows),
            end_frame=max(r['frame'] for r in rows),start_sim_time=rows[0]['sim_time'],end_sim_time=rows[-1]['sim_time'],
            callback_count=len(rows),reporting_vehicle_ids=reporters,
            bidirectional_callbacks=len(reporters)==2,category=category,
            actor_types={actor:sorted(ts) for actor,ts in types.items()},
            peak_callback_impulse=max(float(r['impulse_magnitude']) for r in rows),
            source_lines=[r['source_line'] for r in rows],
            locations=[dict(source_line=r['source_line'],sensor_location=r['sensor_location'])
                       for r in (rows[0],rows[-1]) if 'sensor_location' in r],
            position_definition='sensor position at callback, not contact point',
            responsibility='not_assigned'))
    events.sort(key=lambda e:(e['start_sim_time'],e['actor_ids']))
    for number,event in enumerate(events,1):
        event['collision_event_id']=number
    observed_count=covered_count=0;uncovered=[]
    for frame,row in sorted(coverage_rows.items()):
        observed=set(map(int,row['observed_vehicle_ids']))
        listening=set(map(int,row['listening_vehicle_ids']))
        covered={vid for vid in observed&listening if vid in attached and frame>=attached[vid]}
        observed_count+=len(observed);covered_count+=len(covered)
        if observed-covered:
            uncovered.append(dict(frame=frame,vehicle_ids=sorted(observed-covered)))
    frames=sorted(coverage_rows)
    gaps=[dict(after=a,before=b,missing_frames=b-a-1) for a,b in zip(frames,frames[1:]) if b>a+1]
    summary=dict(schema_version='collision_events.1',merge_gap_seconds=merge_gap_seconds,
        definition='unordered_actor_pair; adjacent_callback_sim_time_gap; no_multi_party_merge',
        raw_callback_count=len(callbacks)+len(unresolved),resolved_callback_count=len(callbacks),
        pair_event_count=len(events),unresolved_callback_count=len(unresolved),unresolved_callbacks=unresolved,
        observed_any_collision_callback=bool(callbacks or unresolved),
        bidirectional_pair_event_count=sum(e['bidirectional_callbacks'] for e in events),
        categories={kind:sum(e['category']==kind for e in events) for kind in sorted({e['category'] for e in events})},
        coverage=dict(audited_frames=len(frames),observed_vehicle_frames=observed_count,
            listening_eligible_vehicle_frames=covered_count,
            listening_eligible_ratio=covered_count/observed_count if observed_count else None,
            uncovered_vehicle_frames=uncovered,frame_gaps=gaps,coverage_ends=ends,errors=failures,
            status='unknown' if not frames else 'gaps_or_errors' if uncovered or gaps or failures else 'audited_listener_health',
            callback_delivery_complete=False,recording_closed=closed),
        limits=['Counts are observed pair contacts, not reconstructed multi-party accidents.',
                'A time gap does not prove physical separation; threshold is configurable.',
                'No callbacks does not prove no collision; pre-attachment and late callbacks may be absent.',
                'All callbacks from attached sensors are counted, including outside collection radius.',
                'No responsibility assignment, impulse summation, episode rate or distance rate.'])
    return events,summary


def summarize(raw_path, output, merge_gap_seconds=1.0, accident_config=None):
    raw_path=Path(raw_path);output=Path(output)
    raw=raw_path.read_bytes()
    records=[json.loads(line) if line.strip() else {} for line in raw.decode('utf-8-sig').splitlines()]
    events,summary=aggregate(records,merge_gap_seconds)
    if accident_config is None:
        metadata_path = raw_path.parent/'metadata.json'
        if metadata_path.exists():
            metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
            if 'region_center_xy' in metadata and 'radius_m' in metadata:
                accident_config = dict(center=metadata['region_center_xy'], radius=metadata['radius_m'], gap=merge_gap_seconds)
    accidents = []
    if accident_config is not None:
        from .collision_accidents import aggregate_accidents, load_contact_frames
        contact_path=raw_path.parent/'vehicles.csv'
        contact_frames=None
        contact_audit=dict(status='unavailable',reason='No sampled geometry')
        if contact_path.exists():
            try:
                ids={int(r[k]) for r in records if r.get('event')=='collision_callback'
                     for k in ('vehicle_id','other_actor_id')}
                contact_frames=load_contact_frames(contact_path,ids)
                contact_audit=dict(status='loaded',source_path=str(contact_path),
                    source_sha256=hashlib.sha256(contact_path.read_bytes()).hexdigest(),frames=len(contact_frames),
                    definition='continuous_world_bbox_XY_intersection_and_Z_overlap; not collision_mesh_contact')
            except ImportError as exc:
                contact_audit=dict(status='unavailable',reason=str(exc))
        accidents, summary['accidents'] = aggregate_accidents(records, contact_frames=contact_frames, **accident_config)
        summary['accidents']['contact_evidence']=contact_audit
    else:
        summary['accidents'] = dict(accident_count=None, status='unavailable', reason='Evaluation region not supplied')
    summary.update(source_path=str(raw_path),source_sha256=hashlib.sha256(raw).hexdigest())
    output.mkdir(parents=True,exist_ok=True)
    with (output/'collision_events.jsonl').open('x',encoding='utf-8') as handle:
        for event in events:
            handle.write(json.dumps(event,ensure_ascii=False)+'\n')
    with (output/'collision_accidents.jsonl').open('x',encoding='utf-8') as handle:
        for event in accidents:
            handle.write(json.dumps(event,ensure_ascii=False)+'\n')
    with (output/'collision_summary.json').open('x',encoding='utf-8') as handle:
        json.dump(summary,handle,ensure_ascii=False,indent=2)
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--merge-gap-seconds',type=float,default=1.0)
    args=parser.parse_args()
    validate_gap(args.merge_gap_seconds)
    output=args.run/('collision_analysis_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ_')+uuid4().hex[:6])
    summary=summarize(args.run/'collisions.jsonl',output,args.merge_gap_seconds)
    print(output)
    print(json.dumps({key:summary[key] for key in ('raw_callback_count','pair_event_count','unresolved_callback_count')},indent=2))
