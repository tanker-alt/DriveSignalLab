"""CARLA waypoint/geometry gates for heterogeneous junctions and taper roads."""
import argparse
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
import carla
from network_route_check import follow
from safety_rules import load_rules
from validate_standard import audit


def validate(folder):
    p=Path(folder);raw=(p/'map.xodr').read_bytes();d=json.loads((p/'manifest.json').read_text())
    assert hashlib.sha256(raw).hexdigest()==d['opendrive_sha256']
    root=ET.fromstring(raw);roads={int(r.get('id')):r for r in root.findall('road')};js={int(j.get('id')) for j in root.findall('junction')}
    assert len(roads)==len(root.findall('road')) and js=={jid for j in d['junctions'] for jid in j.get('opendrive_junction_ids',[j['id']])}
    rings={r['cell_id']:r for r in d.get('roundabouts',[])}
    from roundabout_links import verify
    for ring in rings.values():verify(root,ring['connections'])
    errors=[];movement_checks=[];seam_checks=[];geometry=[]
    m=carla.Map('heterogeneous',raw.decode())
    ring_checks=[]
    for ring in rings.values():
        for link in ring['connections']+ring.get('sidewalk_connections',[]):
            rid=link['road'];length=float(roads[rid].get('length'))
            start=m.get_waypoint_xodr(rid,link['lane'],.05)
            end=m.get_waypoint_xodr(rid,link['lane'],length-.05)
            before=start.previous(.1) if start else []
            after=end.next(.1) if end else []
            ok=bool(before) and bool(after)
            ok=ok and all((w.road_id,w.lane_id)==(link['source'],link['source_lane']) and w.transform.location.distance(start.transform.location)<.3 for w in before)
            ok=ok and all((w.road_id,w.lane_id)==(link['target'],link['target_lane']) and w.transform.location.distance(end.transform.location)<.3 for w in after)
            ring_checks.append(dict(link,passed=ok))
            if not ok:errors.append('Broken CARLA ring connector %s'%rid)
    load_rules(p,required=True)
    for r in roads.values():
        for link in r.findall('./link/*'):
            if int(link.get('elementId')) not in (roads if link.get('elementType')=='road' else js):errors.append('Dangling road link')
    for j in d['junctions']:
        cfg=j['template_config'];part=dict(config=cfg,junctions=[j])
        if j['id'] not in rings:
            surface=audit(root,part,m)
            errors.extend(surface['errors'])
        for rid in j['roads']:
            lanes=roads[rid].findall("./lanes/laneSection/right/lane[@type='driving']")
            if len(lanes)!=j['incoming_lanes']:errors.append('Template lane count changed')
    lengths={r:l for j in d['junctions'] for r,l in zip(j['roads'],j['lengths'])}
    for move in d['movements']:
        try:
            start=m.get_waypoint_xodr(move['road'],move['lane'],lengths[move['road']]-10.)
            path=follow(start,move['target_road'],move['target_lane'],lengths[move['target_road']]-10.)
            gap=max(a.transform.location.distance(b.transform.location) for a,b in zip(path,path[1:]))
            crossed={w.junction_id for w in path if w.is_junction}
            ring=rings.get(move['junction_id'])
            ok=gap<1.25 and (bool(crossed) and crossed<=set(ring['junction_ids']) if ring else crossed=={move['junction_id']})
            movement_checks.append(dict(move,passed=ok,max_gap_m=gap))
            if not ok:errors.append('Discontinuous junction movement')
        except (ValueError,AttributeError) as e:errors.append(str(e))
    for seam in d['seams']:
        bridge=roads[seam['connector_road']];sections=bridge.findall('./lanes/laneSection')
        # Match ALL physical endpoint lanes, including lane 3 at the wide end.
        for arm,section,tag,mirror in [(seam['road_a'],sections[0],'predecessor',True),(seam['road_b'],sections[-1],'successor',False)]:
            count=seam['lanes_a'] if mirror else seam['lanes_b']
            for sign in (-1,1):
                for k in range(1,count+1):
                    lid=sign*k;ln=section.find("./*/lane[@id='%s']"%lid)
                    ref=ln.find('./link/'+tag) if ln is not None else None
                    if ref is None or int(ref.get('id'))!=(-lid if mirror else lid):errors.append('Incorrect bridge endpoint lane link')
        for src,dst in [(seam['road_a'],seam['road_b']),(seam['road_b'],seam['road_a'])]:
            for lane in range(1,seam['through_lanes']+1):
                try:
                    path=follow(m.get_waypoint_xodr(src,lane,5.),dst,-lane,5.)
                    gap=max(a.transform.location.distance(b.transform.location) for a,b in zip(path,path[1:]))
                    ok=gap<1.25 and seam['connector_road'] in {w.road_id for w in path}
                    seam_checks.append(dict(source=src,target=dst,lane=lane,passed=ok,max_gap_m=gap))
                    if not ok:errors.append('Bridge gap/route mismatch')
                except (ValueError,AttributeError) as e:errors.append(str(e))
        for i,section in enumerate(sections):
            s=float(section.get('s'));end=float(sections[i+1].get('s')) if i+1<len(sections) else float(bridge.get('length'))
            for lane in section.findall('./*/lane'):
                lid=int(lane.get('id'))
                if not lid:continue
                poly=lane.find('width');coeff=[float(poly.get(k,'0')) for k in ('a','b','c','d')]
                values=[sum(v*(t*(end-s))**k for k,v in enumerate(coeff)) for t in [q/20. for q in range(21)]]
                if min(values)<-1e-6 or max(values)>3.500001:errors.append('Invalid taper width')
                geometry.append(dict(road=seam['connector_road'],section_s=s,lane=lid,start_width=values[0],end_width=values[-1]))
    report=dict(passed=not errors,opendrive_sha256=d['opendrive_sha256'],junctions=len(d['junctions']),
                movement_checks=movement_checks,seam_checks=seam_checks,ring_connector_checks=ring_checks,taper_geometry=geometry,errors=errors,
                limits='Through lanes validated bidirectionally. Outer dropping lane requires a legal merge before zero width; no many-to-one teleport links.')
    (p/'topology_validation.json').write_text(json.dumps(report,indent=2))
    print('Heterogeneous topology',report['passed'],'movements',len(movement_checks),'bridge traversals',len(seam_checks),'errors',errors[:10])
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder');a=p.parse_args()
    if not validate(a.folder)['passed']:raise SystemExit(1)
