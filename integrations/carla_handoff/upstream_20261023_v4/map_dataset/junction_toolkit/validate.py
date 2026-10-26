"""Offline CARLA waypoint validation; no server or map switch required (Python 3.7)."""
import argparse
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
import carla
from layout import ARM_NAMES, active_arms, arm_turns, is_t


def validate(folder):
    folder=Path(folder);d=json.loads((folder/'manifest.json').read_text())
    raw=(folder/'map.xodr').read_bytes()
    assert hashlib.sha256(raw).hexdigest()==d['opendrive_sha256'],'Map hash differs'
    m=carla.Map('generated',raw.decode('utf-8'));checks=[];errors=[]
    tree=ET.fromstring(raw)
    roads={int(r.get('id')):r for r in tree.findall('road')}
    junction_ids={int(j.get('id')) for j in tree.findall('junction')}
    for rid,road in roads.items():
        for link in road.findall('./link/*'):
            targets=roads if link.get('elementType')=='road' else junction_ids
            if int(link.get('elementId')) not in targets:
                errors.append(dict(road=rid,error='Dangling road link'))
    expected={(v['road'],v['lane'],v['target_road'],v['target_lane'],v['movement']) for v in d['movements']}
    resolved=arm_turns(d['config']);from_config=set()
    for j in d['junctions']:
        base=j['id']-10;active=active_arms(d['config'])
        if set(j['roads']) != {base+i+1 for i in active}:
            errors.append(dict(error='Active arm list differs from config',junction=j['id']))
        for i in active:
            for lane,turns in enumerate(resolved[i],1):
                for turn in turns:
                    dest=(i+{'left':-1,'straight':2,'right':1}[turn])%4
                    from_config.add((base+i+1,-lane,base+dest+1,lane,turn))
            road=roads.get(base+i+1)
            if road is None:
                errors.append(dict(error='Missing active arm'));continue
            g=road.find('./planView/geometry');length=dict(zip(j['roads'],j['lengths']))[base+i+1]
            distance=length+d['config']['junction_radius_m']
            desired=(j['center'][0]-distance*math.cos(i*math.pi/2),j['center'][1]-distance*math.sin(i*math.pi/2))
            if math.hypot(float(g.get('x'))-desired[0],float(g.get('y'))-desired[1])>.01:
                errors.append(dict(error='Arm compass/centre mismatch',road=base+i+1))
        if is_t(d['config']):
            absent=next(i for i in range(4) if i not in active)+base+1
            if absent in roads or any(cfg['road_id']==absent for cfg in d['signals'].values()):
                errors.append(dict(error='Missing arm still present',road=absent))
    if expected != from_config:
        errors.append(dict(error='Manifest movements differ from configuration'))
    signal_ids={s.get('id') for s in tree.findall('./road/signals/signal')}
    if signal_ids != set(d['signals']):errors.append(dict(error='Signal IDs differ from manifest'))
    if d.get('signal_policy')=='one_signal_per_incoming_lane':
        coverage=[];slots=set()
        for sid,cfg in d['signals'].items():
            node=roads[cfg['road_id']].find('./signals/signal[@id="'+sid+'"]')
            lanes=cfg['incoming_lanes'];validity=node.findall('validity') if node is not None else []
            if len(lanes)!=1 or len(validity)!=1 or any(int(v.get(k))!=lanes[0] for v in validity for k in ('fromLane','toLane')):
                errors.append(dict(error='Signal must control exactly its assigned lane',signal=sid))
            if node is None or node.find('positionInertial') is None:
                errors.append(dict(error='Missing explicit lamp pose',signal=sid))
            coverage.extend((cfg['road_id'],lane) for lane in lanes)
            slot=(cfg['junction_id'],cfg['phase_slot'])
            if slot in slots:errors.append(dict(error='Duplicate phase slot',signal=sid))
            slots.add(slot)
        wanted={(rid,-lane) for j in d['junctions'] for rid in j['roads'] for lane in range(1,d['config']['incoming_lanes']+1)}
        if set(coverage)!=wanted or len(coverage)!=len(wanted):
            errors.append(dict(error='Incoming lane signal coverage differs'))
    lengths={rid:l for j in d['junctions'] for rid,l in zip(j['roads'],j['lengths'])}
    for move in d['movements']:
        start=m.get_waypoint_xodr(move['road'],move['lane'],lengths[move['road']]-10)
        if start is None:
            errors.append(dict(move,error='Incoming lane waypoint missing'));continue
        # A lane may branch into several permitted turns; inspect all alternatives.
        todo=[(start,False,0.,0)];found=False;actual_targets=set()
        while todo:
            w,entered,gap,steps=todo.pop()
            if steps>500:continue
            for nxt in w.next(1.):
                distance=w.transform.location.distance(nxt.transform.location)
                newgap=max(gap,distance)
                if entered and not nxt.is_junction:
                    actual_targets.add((nxt.road_id,nxt.lane_id))
                    if (nxt.road_id,nxt.lane_id)==(move['target_road'],move['target_lane']):
                        found=True
                        f=start.transform.get_forward_vector();v=nxt.transform.location-start.transform.location
                        cross=f.x*v.y-f.y*v.x
                        ok=newgap<1.25 and (move['movement']=='straight' or (cross<0 if move['movement']=='left' else cross>0))
                        checks.append(dict(move,max_gap_m=newgap,passed=ok))
                        if not ok:errors.append(dict(move,error='Discontinuous or wrong turn'))
                    continue
                todo.append((nxt,entered or nxt.is_junction,newgap,steps+1))
        if not found:errors.append(dict(move,error='No permitted route to target'))
        expected_targets={(v['target_road'],v['target_lane']) for v in d['movements'] if (v['road'],v['lane'])==(move['road'],move['lane'])}
        if actual_targets!=expected_targets:errors.append(dict(move,error='Actual allowed turns differ from config',actual=sorted(actual_targets),expected=sorted(expected_targets)))
    seams=[]
    for seam in d['seams']:
        for src,dst in [(seam['road_a'],seam['road_b']),(seam['road_b'],seam['road_a'])]:
            for lane in range(1,d['config']['incoming_lanes']+1):
                w=m.get_waypoint_xodr(src,lane,2.)
                path=[w]
                for _ in range(6):
                    ns=w.next(1.)
                    if len(ns)!=1:break
                    w=ns[0];path.append(w)
                gap=max(a.transform.location.distance(b.transform.location) for a,b in zip(path,path[1:])) if len(path)>1 else 999
                ok=w.road_id==dst and w.lane_id==-lane and gap<1.25
                item=dict(source=src,target=dst,lane=lane,max_gap_m=gap,passed=ok);seams.append(item)
                if not ok:errors.append(item)
    result=dict(passed=not errors,junctions=len(d['junctions']),movement_checks=checks,seam_checks=seams,errors=errors,
                limits='Topology only. Does not certify meshes, collisions, signal behavior or agent performance.')
    (folder/'topology_validation.json').write_text(json.dumps(result,indent=2))
    print('Topology:',result['passed'],'movements:',len(checks),'seam directions/lanes:',len(seams),'errors:',errors)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('map_dir');a=p.parse_args()
    if not validate(a.map_dir)['passed']:raise SystemExit(1)
