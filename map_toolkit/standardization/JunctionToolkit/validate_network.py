import json,sys,math
from pathlib import Path
import xml.etree.ElementTree as E
import carla
from safety_rules import load_rules
p=Path(sys.argv[1]); raw=(p/'map.xodr').read_bytes(); root=E.fromstring(raw); d=json.loads((p/'manifest.json').read_text())
rs={r.get('id'):r for r in root.findall('road')}; js={j.get('id'):j for j in root.findall('junction')}; cs={c.get('id'):c for c in root.findall('controller')}
assert len(rs)==len(root.findall('road')) and len(js)==len(root.findall('junction'))
for r in rs.values():
 for l in r.findall('./link/*'): assert l.get('elementId') in (rs if l.get('elementType')=='road' else js),(r.get('id'),l.attrib)
for j in js.values():
 for c in j.findall('connection'):
  assert c.get('incomingRoad') in rs and c.get('connectingRoad') in rs
  assert rs[c.get('connectingRoad')].get('junction')==j.get('id')
 for c in j.findall('controller'): assert c.get('id') in cs
ids=[s.get('id') for s in root.findall('./road/signals/signal')]
assert len(ids)==len(set(ids))==len(d['signals'])
assert set(ids)=={c.get('signalId') for c in root.findall('./controller/control')}
load_rules(p,required=True)
m=carla.Map('audit',raw.decode()); seams=[]
for seam in d['seams']:
 a,b=seam['road_a'],seam['road_b']
 for src,dst in ((a,b),(b,a)):
  for lane in (1,2,3):
   w=m.get_waypoint_xodr(src,lane,0.5); nxt=w.next(1.0)
   assert any(n.road_id==dst and n.lane_id==-lane for n in nxt),(src,dst,lane,[(n.road_id,n.lane_id) for n in nxt])
   n=next(n for n in nxt if n.road_id==dst)
   assert w.transform.location.distance(n.transform.location)<1.02
 seams.append(seam)
# Logical junction graph connectivity from the actual verified seam traversals.
adj={j['id']:set() for j in d['junctions']}
for s in seams:
 a=s['road_a']//10000*10000+10;b=s['road_b']//10000*10000+10
 adj[a].add(b);adj[b].add(a)
seen=set();todo=[next(iter(adj))]
while todo:
 x=todo.pop()
 if x not in seen: seen.add(x);todo.extend(adj[x]-seen)
assert len(seen)==len(adj),(len(seen),len(adj))
report=dict(passed=True,scope='XML references, signal controllers, safety rules, all seam lanes in both directions and junction graph connectivity',junctions=len(js),signals=len(ids),seams=len(seams),topology_edges=len(m.get_topology()),pavement_roads=sum(r.get('name')=='rounded_junction_pavement' for r in rs.values()),opendrive_sha256=d['opendrive_sha256'])
(p/'topology_validation.json').write_text(json.dumps(report,indent=2));print(report)
