"""Offline admission checks for a map intended for controlled experiments."""
import argparse, collections, hashlib, json
from pathlib import Path
import xml.etree.ElementTree as ET
import carla

def audit(folder):
    folder=Path(folder); raw=(folder/'map.xodr').read_bytes()
    d=json.loads((folder/'manifest.json').read_text()); root=ET.fromstring(raw)
    m=carla.Map('admission',raw.decode('utf-8'));errors=[];paths=[]
    if hashlib.sha256(raw).hexdigest()!=d['opendrive_sha256']: errors.append('Map hash mismatch')
    roads={r.get('id'):r for r in root.findall('road')};js={j.get('id') for j in root.findall('junction')}
    for r in roads.values():
        for l in r.findall('./link/*'):
            if l.get('elementId') not in (roads if l.get('elementType')=='road' else js):errors.append('Dangling road link '+r.get('id'))
    for sid,spec in d['signals'].items():
        for lane in spec['incoming_lanes']:
            rid=spec['road_id'];length=float(roads[str(rid)].get('length'))
            start=m.get_waypoint_xodr(rid,lane,max(.01,length-1))
            pending=collections.deque([(start,False)]);seen=set();targets=set()
            while pending:
                w,entered=pending.popleft()
                if w is None:continue
                key=(w.road_id,w.section_id,w.lane_id,round(w.s,1))
                if key in seen:continue
                seen.add(key)
                entered=entered or w.is_junction
                if entered and not w.is_junction:
                    targets.add((w.road_id,w.lane_id));continue
                if len(seen)>5000:errors.append('Traversal overflow');break
                for n in w.next(2.):pending.append((n,entered))
            paths.append(dict(signal=sid,road=rid,lane=lane,targets=sorted(targets)))
            if not targets:errors.append('No exit for signal %s road %s lane %s'%(sid,rid,lane))
            if d['config'].get('type') != 'roundabout':
                expected={(move['target_road'],move['target_lane']) for move in d['movements']
                          if move['road']==rid and move['lane']==lane}
                if targets != expected:
                    errors.append('Exit mismatch for road %s lane %s: %s != %s'%(rid,lane,targets,expected))
    seams=[]
    for s in d.get('seams',[]):
        a,b=s['road_a'],s['road_b']
        for src,dst in ((a,b),(b,a)):
            ls=roads[str(src)].find('./lanes/laneSection')
            for l in ls.findall('./left/lane'):
                if l.get('type')!='driving':continue
                lid=int(l.get('id')); w=m.get_waypoint_xodr(src,lid,.5); ns=w.next(1.) if w else []
                if not any(n.road_id==dst and n.lane_id==-lid and n.transform.location.distance(w.transform.location)<1.05 for n in ns):errors.append('Broken seam %s %s lane %s'%(src,dst,lid))
        seams.append(s)
    if len(d['junctions']) > 1 and d['config'].get('type') != 'roundabout':
        owners={r:j['id'] for j in d['junctions'] for r in j['roads']}
        adjacency={j['id']:set() for j in d['junctions']}
        for seam in seams:
            a,b=owners[seam['road_a']],owners[seam['road_b']]
            adjacency[a].add(b);adjacency[b].add(a)
        visited=set();pending=[next(iter(adjacency))]
        while pending:
            node=pending.pop()
            if node not in visited:
                visited.add(node);pending.extend(adjacency[node]-visited)
        if len(visited)!=len(adjacency):errors.append('Disconnected junction graph')
    report=dict(passed=not errors,opendrive_sha256=d['opendrive_sha256'],errors=errors,signal_lane_paths=paths,seams_checked=len(seams),topology_edges=len(m.get_topology()),scope='Every controlled incoming lane must reach an outgoing non-junction lane; all seam lanes checked bidirectionally; seam junction graph connected. Offline validation only.')
    (folder/'topology_validation.json').write_text(json.dumps(report,indent=2))
    (folder/'admission_topology.json').write_text(json.dumps(report,indent=2));print(folder.name,report['passed'],errors,flush=True)
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder');a=p.parse_args();raise SystemExit(0 if audit(a.folder)['passed'] else 1)
