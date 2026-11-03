"""Sample real OpenDRIVE lane boundaries for a static layout (no CARLA)."""
import argparse,hashlib,json,math,xml.etree.ElementTree as ET
from pathlib import Path

def sample(folder):
    folder=Path(folder);raw=(folder/'map.xodr').read_bytes();root=ET.fromstring(raw)
    manifest=json.loads((folder/'manifest.json').read_text())
    digest=hashlib.sha256(raw).hexdigest()
    if digest!=manifest['opendrive_sha256']:raise ValueError('Map hash mismatch')
    strips=[]
    for road in root.findall('road'):
        sections=road.findall('./lanes/laneSection');geometries=road.findall('./planView/geometry')
        for i,section in enumerate(sections):
            begin=float(section.get('s'));end=float(sections[i+1].get('s')) if i+1<len(sections) else float(road.get('length'))
            for side,sign in [('left',1),('right',-1)]:
                lanes=sorted(section.findall('./'+side+'/lane'),key=lambda l:abs(int(l.get('id'))))
                curves={l.get('id'):([],[]) for l in lanes}
                count=max(2,int((end-begin)/1.5)+1)
                for k in range(count+1):
                    station=begin+(end-begin)*k/count
                    g=[g for g in geometries if float(g.get('s'))<=station+1e-7][-1]
                    ds=station-float(g.get('s'));x=float(g.get('x'));y=float(g.get('y'));h=float(g.get('hdg'))
                    arc=g.find('arc');poly=g.find('paramPoly3')
                    if arc is not None:
                        c=float(arc.get('curvature'));x+=(math.sin(h+c*ds)-math.sin(h))/c;y-=(math.cos(h+c*ds)-math.cos(h))/c;h+=c*ds
                    elif poly is not None:
                        t=ds/float(g.get('length')) if poly.get('pRange')=='normalized' else ds
                        u=sum(float(poly.get(a+'U'))*t**j for j,a in enumerate('abcd'));v=sum(float(poly.get(a+'V'))*t**j for j,a in enumerate('abcd'))
                        du=sum(j*float(poly.get(a+'U'))*t**(j-1) for j,a in enumerate('abcd') if j);dv=sum(j*float(poly.get(a+'V'))*t**(j-1) for j,a in enumerate('abcd') if j)
                        x,y=x+u*math.cos(h)-v*math.sin(h),y+u*math.sin(h)+v*math.cos(h);h+=math.atan2(dv,du)
                    elif g.find('spiral') is not None:
                        sp=g.find('spiral');c=float(sp.get('curvStart'));rate=(float(sp.get('curvEnd'))-c)/float(g.get('length'))
                        steps=max(20,int(ds/.2));step=ds/steps
                        for q in range(steps):
                            t=(q+.5)*step;angle=h+c*t+.5*rate*t*t
                            x+=step*math.cos(angle);y+=step*math.sin(angle)
                        h+=c*ds+.5*rate*ds*ds
                    elif g.find('line') is not None:x+=ds*math.cos(h);y+=ds*math.sin(h)
                    else:raise ValueError('Unsupported geometry')
                    offset=0.
                    for lane in lanes:
                        records=[w for w in lane.findall('width') if float(w.get('sOffset'))<=station-begin+1e-7]
                        if not records:continue
                        w=records[-1];t=station-begin-float(w.get('sOffset'));width=sum(float(w.get(a))*t**j for j,a in enumerate('abcd'))
                        a,b=curves[lane.get('id')]
                        a.append([x-sign*offset*math.sin(h),y+sign*offset*math.cos(h)])
                        offset+=max(0,width);b.append([x-sign*offset*math.sin(h),y+sign*offset*math.cos(h)])
                for lane in lanes:
                    a,b=curves[lane.get('id')]
                    if a:strips.append(dict(kind=lane.get('type'),points=a+b[::-1]))
    labels=[dict(x=j['center'][0],y=j['center'][1],text=str(i+1)) for i,j in enumerate(manifest['junctions'])]
    result=dict(name=folder.name,sha256=digest,strips=strips,labels=labels)
    (folder/'schematic_geometry.json').write_text(json.dumps(result))
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder');sample(p.parse_args().folder)
