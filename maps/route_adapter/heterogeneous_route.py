"""Merge before a shrinking lane, traverse the real bridge, then select turn lane."""
import carla


def plan(m,d,start_road=None,start_lane=None):
    try:
        from .network_itinerary import itinerary
        from .continuous_route_planner import follow,waypoint_record
    except ImportError:
        from network_itinerary import itinerary
        from continuous_route_planner import follow,waypoint_record
    selected=itinerary(d,start_road=start_road,start_lane=start_lane);points=[];changes=[]
    rings={r['cell_id']:r for r in d.get('roundabouts',[])}
    physical_order=[];ring_sequences={}
    lengths={r:l for j in d['junctions'] for r,l in zip(j['roads'],j['lengths'])}
    seams={r:s for s in d['seams'] for r in (s['road_a'],s['road_b'])}
    current=m.get_waypoint_xodr(selected[0]['road'],selected[0]['lane'],25.);first=current
    def add_path(path):
        points.extend(w.transform.location for w in path)
        for w in path:
            if w.is_junction and (not physical_order or physical_order[-1]!=w.junction_id):physical_order.append(w.junction_id)
            for jid,ring in rings.items():
                sequence=ring_sequences.setdefault(str(jid),[])
                if w.road_id in ring['ring_roads'] and (not sequence or sequence[-1]!=w.road_id):sequence.append(w.road_id)
    def change(start,target):
        while start.lane_id!=target:
            old=start.lane_id;new=old+(1 if target>old else -1)
            if old*new<=0:raise ValueError('Opposing lane change')
            direction=1 if old<0 else -1;station=start.s;rid=start.road_id
            for step in range(36):
                a=m.get_waypoint_xodr(rid,old,station+direction*step)
                b=m.get_waypoint_xodr(rid,new,station+direction*step)
                if a is None or b is None or a.is_junction or b.is_junction:raise ValueError('Lane change outside approach')
                allowed=carla.LaneChange.Left if (new-old)*old<0 else carla.LaneChange.Right
                if not int(a.lane_change)&int(allowed):raise ValueError('Prohibited lane change %s/%s at %s'%(rid,old,a.s))
                t=step/35.;f=10*t**3-15*t**4+6*t**5;pa,pb=a.transform.location,b.transform.location
                points.append(carla.Location(x=pa.x+(pb.x-pa.x)*f,y=pa.y+(pb.y-pa.y)*f,z=pa.z))
            changes.append(dict(road=rid,from_lane=old,to_lane=new,start_s=station,end_s=b.s,reason='merge/turn preparation'))
            start=b
        return start
    for i,move in enumerate(selected):
        current=change(current,move['lane'])
        target=move['target_road']
        ring=rings.get(move['junction_id'])
        if ring:
            # Complete one full circuit before taking the chosen network exit.
            arm=move['road']-(ring['cell_id']-10)-1
            for k in range(5):
                road=ring['ring_roads'][(arm+k)%4]
                segment=follow(current,road,move['lane'],5.)
                if any(w.road_id not in ring['ring_roads'] and not w.is_junction and w.road_id!=move['road'] for w in segment):
                    raise ValueError('Full lap left the ring')
                add_path(segment);current=segment[-1]
        path=follow(current,target,move['target_lane'],lengths[target]-25.)
        crossed={w.junction_id for w in path if w.is_junction}
        if ring:
            if not crossed <= set(ring['junction_ids']):raise ValueError('Ring exit leaves its cell')
        elif crossed!={move['junction_id']}:raise ValueError('Unexpected junction in movement')
        add_path(path);current=path[-1]
        if i+1<len(selected):
            seam=seams[target];through=min(move['target_lane'],seam['through_lanes'])
            current=change(current,through)
            path=follow(current,selected[i+1]['road'],-through,25.)
            if any(w.is_junction for w in path):raise ValueError('Bridge traversal unexpectedly enters junction')
            if seam['connector_road'] not in {w.road_id for w in path}:raise ValueError('Route bypasses bridge')
            add_path(path);current=path[-1]
        else:
            path=follow(current,target,move['target_lane'],65.);add_path(path);current=path[-1]
    gaps=[a.distance(b) for a,b in zip(points,points[1:])]
    if max(gaps)>1.25:raise ValueError('Discontinuous planned route')
    required={j for cell in d['junctions'] for j in cell.get('opendrive_junction_ids',[cell['id']])}
    if not required<=set(physical_order):raise ValueError('Route omits physical junctions')
    return points,first,dict(junction_order=physical_order,logical_junction_order=[v['junction_id'] for v in selected],ring_sequences=ring_sequences,start=waypoint_record(first),
                            destination=waypoint_record(current),movements=selected,lane_changes=changes,
                            length_m=sum(gaps),max_sample_gap_m=max(gaps))


def audit(report,route,design):
    bridges={s['connector_road']:s for s in design['seams']}
    observed=[];rows=report['road_transitions']
    for a,b,c in zip(rows,rows[1:],rows[2:]):
        if b['road'] in bridges:observed.append([a['road'],b['road'],c['road']])
    expected=[]
    for a,b in zip(route['movements'],route['movements'][1:]):
        seam=next(s for s in design['seams'] if {s['road_a'],s['road_b']}=={a['target_road'],b['road']})
        expected.append([a['target_road'],seam['connector_road'],b['road']])
    report['observed_bridge_crossings']=observed;report['expected_bridge_crossings']=expected
    report['seam_sequence_passed']=observed==expected
    ring_ok=True
    for ring in design.get('roundabouts',[]):
        sequence=[]
        for row in rows:
            if row['road'] in ring['ring_roads'] and (not sequence or sequence[-1]!=row['road']):sequence.append(row['road'])
        expected_ring=route['ring_sequences'][str(ring['cell_id'])]
        report.setdefault('ring_crossings',[]).append(dict(cell=ring['cell_id'],observed=sequence,expected=expected_ring))
        ring_ok=ring_ok and sequence==expected_ring and set(ring['ring_roads'])<=set(sequence)
    report['ring_sequence_passed']=ring_ok
    return observed==expected and ring_ok
