"""XML and logical-route checks only: never imports or contacts CARLA."""
import hashlib
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from network_itinerary import itinerary
from roundabout_links import verify


def require(ok,message):
    if not ok:raise ValueError(message)


def lanes(section):
    return {int(x.get('id')):x for x in section.findall('./*/lane')}


def widths(section,s,side):
    result=[]
    for lane in section.findall('./'+side+'/lane'):
        records=[w for w in lane.findall('width') if float(w.get('sOffset'))<=s+1e-8]
        w=records[-1];t=s-float(w.get('sOffset'))
        value=sum(float(w.get(k))*t**i for i,k in enumerate(('a','b','c','d')))
        require(math.isfinite(value) and value>=-1e-7,'Invalid lane width')
        result.append(value)
    return sum(result)


def validate(folder):
    folder=Path(folder);root=ET.parse(folder/'map.xodr').getroot()
    d=json.loads((folder/'manifest.json').read_text())
    digest=hashlib.sha256((folder/'map.xodr').read_bytes()).hexdigest()
    require(digest==d['opendrive_sha256'],'Manifest hash mismatch')
    tables={}
    for tag,path in [('road','road'),('junction','junction'),('controller','controller'),('signal','./road/signals/signal')]:
        items=root.findall(path);table={x.get('id'):x for x in items}
        require(len(table)==len(items),'Duplicate '+tag+' ID');tables[tag]=table
    roads=tables['road']
    for road in roads.values():
        if road.get('junction')!='-1':require(road.get('junction') in tables['junction'],'Missing road junction')
        for link in road.findall('./link/*'):
            require(link.get('elementId') in tables[link.get('elementType')],'Missing road endpoint')
    for j in tables['junction'].values():
        for co in j.findall('connection'):
            incoming=roads[co.get('incomingRoad')];connector=roads[co.get('connectingRoad')]
            require(connector.get('junction')==j.get('id'),'Wrong connecting junction')
            incoming_ids={x.get('id') for x in incoming.findall('./lanes/laneSection/*/lane')}
            section=connector.findall('./lanes/laneSection')[0 if co.get('contactPoint')=='start' else -1]
            for ll in co.findall('laneLink'):
                require(ll.get('from') in incoming_ids and int(ll.get('to')) in lanes(section),'Invalid junction laneLink')
    require(set(d['signals'])==set(tables['signal']),'Signal manifest mismatch')
    for ctrl in tables['controller'].values():
        for co in ctrl.findall('control'):require(co.get('signalId') in tables['signal'],'Missing controller signal')
    for cell in d['junctions']:
        require(all(str(j) in tables['junction'] for j in cell['opendrive_junction_ids']),'Missing physical junction')
    require(len(d['roundabouts'])<=1,'Multiple roundabouts')
    for ring in d['roundabouts']:
        verify(root,ring['connections'])
        if ring.get('sidewalk_connections'):verify(root,ring['sidewalk_connections'])
    for seam in d['seams']:
        road=roads[str(seam['connector_road'])];sections=road.findall('./lanes/laneSection')
        for i,section in enumerate(sections):
            length=(float(sections[i+1].get('s')) if i+1<len(sections) else float(road.get('length')))-float(section.get('s'))
            for side in ('left','right'):
                edge=section.findall('./'+side+"/lane[@type='sidewalk']")
                require(len(edge)==2,'Missing curb/sidewalk')
                require([float(x.find('width').get('a')) for x in edge]==[.2,2.],'Wrong sidewalk widths')
                for k in range(21):widths(section,length*k/20,side)
                if i+1<len(sections):require(abs(widths(section,length,side)-widths(sections[i+1],0,side))<1e-6,'Sidewalk edge discontinuity')
            for tag,step in [('predecessor',-1),('successor',1)]:
                if 0<=i+step<len(sections):
                    other=sections[i+step];backtag='successor' if step==-1 else 'predecessor'
                else:
                    endpoint=road.find('./link/'+tag);parent=roads[endpoint.get('elementId')]
                    other=parent.findall('./lanes/laneSection')[0 if endpoint.get('contactPoint')=='start' else -1]
                    backtag='predecessor' if endpoint.get('contactPoint')=='start' else 'successor'
                    back=parent.find('./link/'+backtag)
                    require(back is not None and back.get('elementId')==road.get('id') and back.get('contactPoint')==('start' if step==-1 else 'end'),'Nonreciprocal road seam')
                for lane in lanes(section).values():
                    if lane.get('id')=='0':continue
                    link=lane.find('./link/'+tag)
                    if link is None:
                        require(lane.get('type')=='driving','Unlinked sidewalk');continue
                    target=lanes(other).get(int(link.get('id')))
                    require(target is not None,'Missing seam lane')
                    back=target.find('./link/'+backtag)
                    require(back is not None and back.get('id')==lane.get('id'),'Nonreciprocal seam lane')
    route=itinerary(d)
    require({m['junction_id'] for m in route}=={j['id'] for j in d['junctions']},'Incomplete logical itinerary')
    report=dict(passed=True,opendrive_sha256=digest,logical_cells=len(d['junctions']),physical_junctions=len(tables['junction']),transitions=len(d['seams']),roundabouts=len(d['roundabouts']),logical_itinerary=[m['junction_id'] for m in route],carla_validation_performed=False,driving_acceptance='not_run')
    (folder/'static_validation.json').write_text(json.dumps(report,indent=2))
    return report


if __name__=='__main__':print(json.dumps(validate(sys.argv[1]),indent=2))
