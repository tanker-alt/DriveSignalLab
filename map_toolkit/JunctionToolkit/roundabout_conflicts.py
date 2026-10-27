"""Offline centreline crossing audit, not a complete swept-volume conflict matrix."""
import argparse,json,math
from pathlib import Path


def run(folder):
    folder=Path(folder);d=json.loads((folder/'manifest.json').read_text());samples=json.loads((folder/'waypoint_preview.json').read_text())
    lines={(s['road_id'],s['lane_id']):s['points'] for s in samples};conflicts=[]
    def cross(a,b):return a[0]*b[1]-a[1]*b[0]
    def sub(a,b):return (a[0]-b[0],a[1]-b[1])
    def intersections(a,b):
        found=[]
        for p,q in zip(a,a[1:]):
            r=sub(q,p)
            for x,y in zip(b,b[1:]):
                s=sub(y,x);den=cross(r,s)
                if abs(den)<1e-8:continue
                u=cross(sub(x,p),s)/den;v=cross(sub(x,p),r)/den
                if 0<u<1 and 0<v<1:
                    z=(p[0]+u*r[0],p[1]+u*r[1])
                    if min(math.hypot(z[0]-t[0],z[1]-t[1]) for t in (a[0],a[-1],b[0],b[-1]))>1:
                        if not any(math.hypot(z[0]-t[0],z[1]-t[1])<1 for t in found):found.append(z)
        return found
    paths=d['connections']
    for i,a in enumerate(paths):
        for b in paths[i+1:]:
            if d['road_geometry'][str(a['road'])]['junction']!=d['road_geometry'][str(b['road'])]['junction']:continue
            hits=intersections(lines[(a['road'],a.get('lane',-1))],lines[(b['road'],b.get('lane',-1))])
            if not hits:continue
            groups=[]
            for path in (a,b):
                matches=[s for s in d['signals'].values() if s['road_id']==path['source'] and path['source_lane'] in s['incoming_lanes']]
                groups.append(matches[0]['signal_group'])
            conflicts.append(dict(paths=[a,b],groups=groups,locations_opendrive=hits))
    plan=json.loads((folder/'phase_example.json').read_text());phases=[]
    for p in plan['phases']:
        ids=[i for i,c in enumerate(conflicts) if all(g in p['green_groups'] for g in c['groups'])]
        phases.append(dict(name=p['name'],simultaneous_crossing_indices=ids,clear_of_detected_crossings=not ids))
    result=dict(conflicts=conflicts,phase_checks=phases,safe_for_traffic=False,
                limits='Centreline intersections only; excludes vehicle width, merging conflicts, queues and clearance times. No detected crossings is not proof of safety.')
    (folder/'conflict_audit.json').write_text(json.dumps(result,indent=2));print('Crossings:',len(conflicts),'phase checks:',phases)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('map_dir');run(p.parse_args().map_dir)
