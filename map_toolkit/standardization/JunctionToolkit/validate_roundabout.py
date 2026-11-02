"""Offline CARLA topology checks for the experimental roundabout."""
import argparse,collections,hashlib,json,math,xml.etree.ElementTree as ET
from pathlib import Path
import carla


def validate(folder):
    folder=Path(folder);d=json.loads((folder/'manifest.json').read_text());raw=(folder/'map.xodr').read_bytes()
    if hashlib.sha256(raw).hexdigest()!=d['opendrive_sha256']:raise ValueError('Map hash differs')
    m=carla.Map('roundabout_candidate',raw.decode());errors=[];connections=[];routes=[];geometry=[]
    tree=ET.fromstring(raw)
    ring_lanes=d['config']['circulating_lanes'];expected_signals=16 if d['config'].get('internal_signals') else 8
    if len(d['signals'])!=expected_signals:errors.append(dict(error='Signal count differs',expected=expected_signals))
    owned={(cfg['road_id'],lane) for cfg in d['signals'].values() for lane in cfg['incoming_lanes']}
    wanted={(r,l) for r in range(1,5) for l in (-1,-2)}
    if d['config'].get('internal_signals'):wanted|={(r,-l) for r in d['ring_roads'] for l in range(1,ring_lanes+1)}
    if owned!=wanted or len(owned)!=len(d['signals']):errors.append(dict(error='Lane signal coverage differs'))
    for sid,cfg in d['signals'].items():
        sig=tree.find('./road[@id="'+str(cfg['road_id'])+'"]/signals/signal[@id="'+sid+'"]')
        valid=sig.findall('validity') if sig is not None else []
        if len(valid)!=1 or any(int(v.get(k))!=cfg['incoming_lanes'][0] for v in valid for k in ('fromLane','toLane')):
            errors.append(dict(signal=sid,error='Signal lane ownership differs'))
    for rid in d['ring_roads']+d['ring_continuations']:
        lanes=tree.findall('./road[@id="'+str(rid)+'"]/lanes/laneSection/right/lane')
        if {int(l.get('id')) for l in lanes}!=set(range(-ring_lanes,0)):errors.append(dict(road=rid,error='Ring lane count differs'))
    polylines=[]
    for rid,spec in d['road_geometry'].items():
        rid=int(rid)
        for lane in ((-2,-1,1,2) if rid<10 else tuple(range(-ring_lanes,0)) if rid in d['ring_roads']+d['ring_continuations'] else (-1,)):
            points=[]
            for k in range(201):
                wp=m.get_waypoint_xodr(rid,lane,.001+(spec['length']-.002)*k/200)
                if wp:points.append([wp.transform.location.x,-wp.transform.location.y])
            polylines.append(dict(road_id=rid,lane_id=lane,points=points))
    (folder/'waypoint_preview.json').write_text(json.dumps(polylines))
    for link in d['connections']:
        rid=link['road'];length=d['road_geometry'][str(rid)]['length'];wp=m.get_waypoint_xodr(rid,link.get('lane',-1),length-.05)
        ns=wp.next(.1) if wp else []
        good=bool(ns) and all((n.road_id,n.lane_id)==(link['target'],link['target_lane']) for n in ns)
        gap=max([wp.transform.location.distance(n.transform.location) for n in ns] or [999])
        item=dict(link,passed=good and gap<.3,endpoint_gap_m=gap);connections.append(item)
        if not item['passed']:errors.append(item)
    for segment in polylines:
        rid=segment['road_id'];lane=segment['lane_id'];spec=d['road_geometry'][str(rid)]
        if rid<10:continue
        samples=[m.get_waypoint_xodr(rid,lane,.001+(spec['length']-.002)*i/200) for i in range(201)]
        maximum=0
        for a,b in zip(samples,samples[1:]):
            if a is None or b is None:errors.append(dict(road=rid,error='Missing sample'));break
            distance=a.transform.location.distance(b.transform.location)
            turn=abs((b.transform.rotation.yaw-a.transform.rotation.yaw+180)%360-180)*math.pi/180
            if distance>.0001:maximum=max(maximum,turn/distance)
        geometry.append(dict(road=rid,lane=lane,minimum_sampled_turn_radius_m=1/maximum if maximum else None))
        if maximum>1/8:errors.append(dict(road=rid,error='Sampled radius below prototype minimum 8m'))
        if rid in d['ring_roads']+d['ring_continuations']:
            for wp in samples:
                p=wp.transform.location;f=wp.transform.get_forward_vector()
                # CARLA Y is inverted: CCW OpenDRIVE => negative XY cross product.
                if p.x*f.y-p.y*f.x>=0:errors.append(dict(road=rid,error='Wrong circulation'));break
    for arm in range(1,5):
        for lane in (-1,-2):
            start=m.get_waypoint_xodr(arm,lane,d['config']['arm_length_m']-10)
            todo=collections.deque([start]);seen=set();exits=set();maxgap=0;loop=False
            while todo and len(seen)<12000:
                wp=todo.popleft();key=(wp.road_id,wp.lane_id,round(wp.s))
                if key in seen:
                    if wp.road_id in d['ring_roads']:loop=True
                    continue
                seen.add(key)
                if wp.road_id in range(1,5) and wp.lane_id>0:
                    exits.add((wp.road_id,wp.lane_id));continue
                ns=wp.next(1.)
                if not ns:errors.append(dict(road=wp.road_id,lane=wp.lane_id,s=wp.s,error='Dead end before exit'))
                for n in ns:
                    maxgap=max(maxgap,wp.transform.location.distance(n.transform.location));todo.append(n)
            wanted={(r,l) for r in range(1,5) for l in ((-lane,) if ring_lanes==2 else (1,2))}
            item=dict(road=arm,lane=lane,reachable_exits=sorted(exits),ring_cycle_reachable=loop,max_step_gap_m=maxgap,passed=exits==wanted and loop and maxgap<1.3 and not todo)
            routes.append(item)
            if not item['passed']:errors.append(item)
    result=dict(passed=not errors,signal_count=len(d['signals']),connections=connections,entry_routes=routes,geometry=geometry,errors=errors,
                limits='Offline waypoint checks only. No live mesh, collision, signal stopping or traffic-flow validation.')
    (folder/'topology_validation.json').write_text(json.dumps(result,indent=2))
    print('Roundabout topology:',result['passed'],'signals:',len(d['signals']),'connections:',len(connections),'entry lanes:',len(routes),'errors:',len(errors))
    if errors:print(json.dumps(errors[:5],indent=2))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('map_dir');a=p.parse_args()
    if not validate(a.map_dir)['passed']:raise SystemExit(1)
