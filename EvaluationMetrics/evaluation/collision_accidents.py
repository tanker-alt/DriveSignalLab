"""Spatial/temporal connected collision callbacks; no responsibility inference."""
import math
import csv
import json
import hashlib


def aggregate_accidents(records, center, radius=120.0, gap=1.0, distance=15.0, contact_frames=None):
    if len(center) != 2 or not all(math.isfinite(float(v)) for v in center):
        raise ValueError('Finite XY center required')
    if not all(math.isfinite(v) and v > 0 for v in (radius, distance)) or not math.isfinite(gap) or gap < 0:
        raise ValueError('Invalid accident thresholds')
    rows, excluded, unknown = [], [], []
    for line, r in enumerate(records, 1):
        if r.get('event') != 'collision_callback':
            continue
        p = r.get('sensor_location', {})
        try:
            x, y = float(p['x']), float(p['y'])
            t = float(r['sim_time'])
            a, b = int(r['vehicle_id']), int(r['other_actor_id'])
            if not all(math.isfinite(v) for v in (x, y, t)) or t < 0 or a <= 0 or b < 0 or a == b or (b == 0 and not r.get('other_actor_type', '').startswith('static.')):
                raise ValueError()
        except (KeyError, ValueError, TypeError, OverflowError):
            unknown.append(line)
            continue
        # Radius uses callback sensor position, not an unavailable physical contact point.
        inside = math.hypot(x-center[0], y-center[1]) <= radius
        vehicles = {a}
        if r.get('other_actor_type', '').startswith('vehicle.'):
            vehicles.add(b)
        rows.append(dict(line=line, t=t, frame=r.get('frame'), xy=(x,y), actors=({a,b} if b else {a}), unidentified_static=(b==0), vehicles=vehicles, inside=inside))
        if not inside:
            excluded.append(line)
    rows.sort(key=lambda r:(r['t'], r['line']))
    parent=list(range(len(rows)))
    def find(i):
        while parent[i] != i:
            parent[i]=parent[parent[i]]
            i=parent[i]
        return i
    active=[]
    for i,r in enumerate(rows):
        active=[j for j in active if r['t']-rows[j]['t'] <= gap+1e-9]
        for j in active:
            q=rows[j]
            if r['vehicles'] & q['vehicles'] and math.hypot(r['xy'][0]-q['xy'][0],r['xy'][1]-q['xy'][1]) <= distance:
                parent[find(i)]=find(j)
        active.append(i)
    # A sensor can be quiet while bodies remain in contact. Bridge only an
    # already observed pair, with complete per-frame geometry, never proximity alone.
    bridges=[]
    last_pair={}
    for i,r in enumerate(rows):
        pair=tuple(sorted(r['actors']))
        if len(pair)!=2:
            continue
        j=last_pair.get(pair)
        if j is not None:
            q=rows[j]
            if (r['t']-q['t'] > gap+1e-9 and
                    math.hypot(r['xy'][0]-q['xy'][0], r['xy'][1]-q['xy'][1]) <= distance and
                    continuous_contact(contact_frames, pair, q, r)):
                parent[find(i)]=find(j)
                bridges.append(dict(actor_ids=list(pair),from_source_line=q['line'],
                                    to_source_line=r['line'],start_frame=q['frame'],end_frame=r['frame']))
        last_pair[pair]=i
    groups={}
    for i,r in enumerate(rows):
        groups.setdefault(find(i),[]).append(r)
    events=[]
    for group in groups.values():
        if not any(r['inside'] for r in group):
            continue
        events.append(dict(start_sim_time=min(r['t'] for r in group),
            end_sim_time=max(r['t'] for r in group),
            actor_ids=sorted(set().union(*(r['actors'] for r in group))),
            vehicle_ids=sorted(set().union(*(r['vehicles'] for r in group))),
            actor_pairs=sorted({tuple(sorted(r['actors'])) for r in group if len(r['actors'])==2}),
            unidentified_static_source_lines=sorted(r['line'] for r in group if r['unidentified_static']),
            source_lines=sorted(r['line'] for r in group),
            in_region_source_lines=sorted(r['line'] for r in group if r['inside']),
            callback_count=len(group), responsibility='not_assigned'))
    events.sort(key=lambda e:(e['start_sim_time'],e['actor_ids'],e['source_lines']))
    for i,e in enumerate(events,1):
        e['accident_id']=i
    return events, dict(accident_count=len(events),
        status='observed_accidents_with_coverage_audit',
        definition='shared_vehicle_temporal_spatial_callbacks_with_continuous_body_contact',
        continuous_contact_bridges=bridges,
        region=dict(shape='circle',center_xy=list(center),radius_m=radius,
                    membership='at_least_one_callback_sensor_position_inside'),
        config=dict(merge_gap_seconds=gap,merge_distance_m=distance,threshold_status='provisional'),
        unknown_source_lines=unknown, outside_callback_source_lines=excluded,
        limits=['Sensor position is not contact point; boundary attribution is approximate.',
                'Outside callbacks can extend a connected accident with an inside callback.',
                'No callbacks does not prove absence of collision.',
                'Thresholds define observed episodes, not proven causal accident reconstruction.'])


def continuous_contact(frames, pair, before, after):
    if not frames or not isinstance(before['frame'], int) or not isinstance(after['frame'], int):
        return False
    first,last=before['frame'],after['frame']
    if last <= first or last-first+1 > len(frames):
        return False
    previous_time=None
    for frame in range(first,last+1):
        data=frames.get(frame)
        if not data or any(vid not in data['bodies'] for vid in pair):
            return False
        t=data['time']
        if previous_time is not None and t <= previous_time:
            return False
        if frame==first and abs(t-before['t'])>1e-5 or frame==last and abs(t-after['t'])>1e-5:
            return False
        previous_time=t
        a,b=(data['bodies'][vid] for vid in pair)
        if min(a[2],b[2]) < max(a[1],b[1]) or not a[0].intersects(b[0]):
            return False
    return True


def load_contact_frames(path, vehicle_ids):
    """Read authoritative sampled world bounding boxes; no inferred missing frames."""
    from shapely.geometry import MultiPoint
    frames={}
    with path.open(encoding='utf-8-sig', newline='') as handle:
        for row in csv.DictReader(handle):
            try:
                vid=int(row['vehicle_id'])
                if vid not in vehicle_ids:
                    continue
                frame=int(row['frame']); t=float(row['sim_time'])
                vertices=json.loads(row['bbox_vertices_world_json'])
                if len(vertices)!=8 or not math.isfinite(t) or not all(len(v)==3 and all(math.isfinite(float(x)) for x in v) for v in vertices):
                    continue
                hull=MultiPoint([(v[0],v[1]) for v in vertices]).convex_hull
                if not hull.is_valid or hull.area<=0:
                    continue
                data=frames.setdefault(frame,dict(time=t,bodies={},invalid=set()))
                if abs(data['time']-t)>1e-5 or vid in data['bodies'] or vid in data['invalid']:
                    data['invalid'].add(vid);data['bodies'].pop(vid,None)
                    continue
                data['bodies'][vid]=(hull,min(v[2] for v in vertices),max(v[2] for v in vertices))
            except (ValueError,KeyError,TypeError):
                continue
    return frames
