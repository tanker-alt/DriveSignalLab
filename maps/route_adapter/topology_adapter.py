"""Topology-first map adapter for traffic-flow and scenario buckets.

The adapter consumes OpenDRIVE plus optional generator manifests. Coordinates are
only emitted as resolved evidence; consumers use road/lane/s features.
"""
import argparse
import hashlib
import json
from pathlib import Path

import carla
from .errors import AdapterError


def lane_s_direction(road_map, road, lane, s):
    """Resolve OpenDRIVE s orientation using the live lane forward vector."""
    a = road_map.get_waypoint_xodr(road, lane, s)
    b = road_map.get_waypoint_xodr(road, lane, s + .1)
    offset = .1
    if b is None:
        b = road_map.get_waypoint_xodr(road, lane, s - .1)
        offset = -.1
    if a is None or b is None:
        raise AdapterError('lane_direction_unresolved', '{} / {} / {}'.format(road, lane, s))
    p, q = a.transform.location, b.transform.location
    f = a.transform.get_forward_vector()
    dot = ((q.x-p.x)*f.x + (q.y-p.y)*f.y) / offset
    if abs(dot) < .001:
        raise AdapterError('lane_direction_unresolved', 'Degenerate lane geometry')
    return 1 if dot > 0 else -1


def bypass_route(road_map, route, obstacle_s, adjacent_lane, change_m=20., clearance_m=8., obstacle_road=None):
    """Reference feasibility path, using live lane permission in both directions."""
    import math
    from copy import deepcopy
    anchor = next(p for p in route if obstacle_road is None or p['road_id']==obstacle_road)
    road, lane = anchor['road_id'], anchor['lane_id']
    direction = lane_s_direction(road_map, road, lane, obstacle_s)
    result = deepcopy(route)
    for p in result:
        if p['road_id'] != road:
            continue
        u = direction * (p['s'] - obstacle_s)
        if u < -clearance_m-change_m or u > clearance_m+change_m:
            continue
        a = road_map.get_waypoint_xodr(road, lane, p['s'])
        b = road_map.get_waypoint_xodr(road, adjacent_lane, p['s'])
        if a is None or b is None or a.is_junction or b.is_junction:
            raise ValueError('Bypass window leaves the continuous approach')
        f, g = a.transform.get_forward_vector(), b.transform.get_forward_vector()
        if f.x*g.x+f.y*g.y < .99:
            raise ValueError('Bypass lane is not in the same direction')
        source, dest = (a,b) if u < 0 else (b,a)
        left = source.get_left_lane()
        required = 'Left' if left and left.lane_id == dest.lane_id else 'Right'
        neighbor = source.get_left_lane() if required == 'Left' else source.get_right_lane()
        if neighbor is None or neighbor.lane_id != dest.lane_id or str(source.lane_change) not in (required,'Both'):
            raise ValueError('Bypass crosses a prohibited lane marking')
        blend = min(1., max(0., (u+clearance_m+change_m)/change_m)) if u < 0 else min(1., max(0., (clearance_m+change_m-u)/change_m))
        blend = blend*blend*(3-2*blend)
        for axis in ('x','y','z'):
            p[axis] = getattr(a.transform.location,axis)*(1-blend)+getattr(b.transform.location,axis)*blend
    for a,b in zip(result,result[1:]):
        a['yaw'] = math.degrees(math.atan2(b['y']-a['y'],b['x']-a['x']))
    return result


def transition_route(road_map, route, road, source_lane, target_lane, start_s, end_s):
    """One legal same-direction transition; retain the target movement downstream."""
    import math
    from copy import deepcopy
    direction = lane_s_direction(road_map, road, target_lane, start_s)
    length = direction * (end_s-start_s)
    if length < 10:
        raise ValueError('Insufficient lane-change length')
    result = deepcopy(route)
    for p in result:
        if p['road_id'] != road or direction*(p['s']-end_s)>0:
            continue
        a=road_map.get_waypoint_xodr(road,source_lane,p['s'])
        b=road_map.get_waypoint_xodr(road,target_lane,p['s'])
        if not a or not b or a.is_junction or b.is_junction:
            raise ValueError('Lane transition leaves non-junction road')
        f,g=a.transform.get_forward_vector(),b.transform.get_forward_vector()
        left=a.get_left_lane(); side='Left' if left and left.lane_id==target_lane else 'Right'
        neighbor=a.get_left_lane() if side=='Left' else a.get_right_lane()
        if not neighbor or neighbor.lane_id!=target_lane or f.x*g.x+f.y*g.y<.99:
            raise ValueError('Transition requires adjacent same-direction lane')
        u=max(0.,min(1.,direction*(p['s']-start_s)/length))
        if u>0 and str(a.lane_change) not in (side,'Both'):
            raise ValueError('Transition crosses prohibited marking')
        blend=u*u*(3-2*u)
        for axis in ('x','y','z'):
            p[axis]=getattr(a.transform.location,axis)*(1-blend)+getattr(b.transform.location,axis)*blend
        p['lane_id']=source_lane if u<.5 else target_lane
    for a,b in zip(result,result[1:]):
        a['yaw']=math.degrees(math.atan2(b['y']-a['y'],b['x']-a['x']))
    return result


def _read(path, default=None):
    p=Path(path)
    return json.loads(p.read_text(encoding='utf-8-sig')) if p.exists() else default


def adapt(map_dir):
    folder=Path(map_dir)
    from .adapter import read_map
    raw, _, _ = read_map(folder)
    from .contract import validate_opendrive, mapping_diagnostics
    root = validate_opendrive(raw)
    road_map=carla.Map(folder.name,raw.decode('utf-8-sig'))
    manifest=_read(folder/'manifest.json',{}) or {}
    config=manifest.get('config',{})
    junctions=manifest.get('junctions') or []
    if not junctions:
        ids=sorted({w.junction_id for w in road_map.generate_waypoints(5.0) if w.is_junction})
        junctions=[{'id':int(j),'roads':sorted({w.road_id for w in road_map.generate_waypoints(5.0) if w.junction_id==j})} for j in ids]
    kind=config.get('type') or ('roundabout' if any('round' in str(j).lower() for j in junctions) else None)
    if not kind:
        kind='t' if any(len(j.get('roads',[]))==3 for j in junctions) else ('cross' if junctions else 'corridor')
    signals=manifest.get('signals',{})
    safety=_read(folder/'safety_rules.json',{}) or {}
    stops=safety.get('stop_lines',[])
    movements=manifest.get('movements',[])
    if not movements:
        for i,(a,b) in enumerate(road_map.get_topology()):
            if a.road_id!=b.road_id:
                movements.append(dict(id='topology_{}'.format(i),from_road=a.road_id,from_lane=a.lane_id,
                    to_road=b.road_id,to_lane=b.lane_id,from_s=a.s,to_s=b.s))
    entries=sorted({(m.get('road_id',m.get('from_road',m.get('road'))),m.get('lane',m.get('from_lane'))) for m in movements if m.get('road_id',m.get('from_road',m.get('road'))) is not None})
    exits=sorted({(m.get('target_road',m.get('to_road')),m.get('target_lane',m.get('to_lane'))) for m in movements if m.get('target_road',m.get('to_road')) is not None})
    od=[]
    for m in movements:
        o=m.get('road_id',m.get('from_road',m.get('road')));d=m.get('target_road',m.get('to_road'))
        if o is not None and d is not None: od.append(dict(movement_id=m.get('id'),origin_road=o,origin_lane=m.get('lane',m.get('from_lane')),destination_road=d,destination_lane=m.get('target_lane',m.get('to_lane'))))
    signal_rows=[]
    for sid,cfg in signals.items():
        for lane in cfg.get('incoming_lanes',[]):
            signal_rows.append(dict(signal_id=str(sid),junction_id=cfg.get('junction_id'),road_id=cfg.get('road_id'),lane_id=lane,reference_s=cfg.get('reference_s')))
    if not stops:
        stops=[dict(id='stop_'+x['signal_id'],junction_id=x['junction_id'],signal_opendrive_id=x['signal_id'],
                    road_id=x['road_id'],lane_id=x['lane_id'],s_m=x['reference_s'],position_source='manifest.signals.reference_s')
               for x in signal_rows if x['reference_s'] is not None]
    from .route_topology import LaneGraph
    graph=LaneGraph(road_map)
    routes=graph.catalog()
    topology_checks=[dict(route_id=r['route_id'],**graph.audit(r['route'],(r['destination'],r['destination_lane']))) for r in routes]
    od=[dict(movement_id=r['route_id'],route_id=r['route_id'],origin_road=r['origin'],origin_lane=r['lane'],
             destination_road=r['destination'],destination_lane=r['destination_lane']) for r in routes]
    entries=sorted({(r['origin'],r['lane']) for r in routes})
    exits=sorted({(r['destination'],r['destination_lane']) for r in routes})
    digest=hashlib.sha256(raw).hexdigest()
    diagnostics=mapping_diagnostics(road_map,root,signal_rows,stops,safety,digest)
    return dict(schema_version='map_adapter.2',map_name=folder.name,opendrive_sha256=hashlib.sha256(raw).hexdigest(),
        source=str(folder),classification=dict(kind=kind,junction_count=len(junctions),road_count=len(root.findall('road'))),
        junctions=junctions,entries=[dict(road_id=r,lane_id=l) for r,l in entries],exits=[dict(road_id=r,lane_id=l) for r,l in exits],topology_checks=topology_checks,
        od_connections=od,route_catalog=routes,signals=signal_rows,stop_lines=stops,
        diagnostics=diagnostics,
        passed=bool(routes) and all(c['passed'] for c in topology_checks) and not any(d['severity']=='error' for d in diagnostics),
        capabilities=dict(junction_identification=True,od_extraction=bool(od),signal_stop_mapping=bool(signal_rows and stops) and not any(d['severity']=='error' for d in diagnostics),special_right_of_way=False,dynamic_validation=False,
                          topology_source='Offline CARLA lane graph; next() successors and boundary OD enumeration'))


def main():
    p=argparse.ArgumentParser();p.add_argument('map_dir',type=Path);p.add_argument('--output',type=Path);a=p.parse_args()
    result=adapt(a.map_dir);out=a.output or a.map_dir/'map_adapter.json';out.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8');print(out)


if __name__=='__main__':main()
