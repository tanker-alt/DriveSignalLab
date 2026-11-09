"""Real two-lane T + three-lane cross tiles joined by xodr LaneDef roads."""
import argparse
import copy
import hashlib
import json
import math
import random
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from scenariogeneration import xodr
from generate import generate, check
from layout import ARM_NAMES, active_arms, arm_turns, TURN_OFFSETS
from network_plan import connected
from network_itinerary import itinerary
from runtime_config import map_output
from safety_rules import write_default_rules
from transition_sidewalks import stitch_edges, add_edges


def ring_position(config,rows,cols):
    enabled=config.get('include_roundabout',False)
    if type(enabled) is not bool:raise ValueError('include_roundabout must be boolean')
    if not enabled:
        if config.get('roundabout_position') is not None:raise ValueError('roundabout_position requires include_roundabout')
        return None
    pos=config.get('roundabout_position',[rows//2,cols//2])
    if not isinstance(pos,(list,tuple)) or len(pos)!=2 or any(type(v)!=int for v in pos):
        raise ValueError('roundabout_position must be [row,column]')
    if not (0<=pos[0]<rows and 0<=pos[1]<cols):raise ValueError('roundabout_position outside grid')
    return pos[0]*cols+pos[1]


def sample(config):
    if config.get('t_probability',.5)!=.5 or config.get('cross_probability',.5)!=.5:
        raise ValueError('This generator uses equal T/cross proposal probabilities')
    rows=config.get('rows',3);cols=config.get('columns',3)
    if type(rows)!=int or type(cols)!=int or not 1<=rows<=6 or not 1<=cols<=6 or rows*cols<2:
        raise ValueError('Dimensions must be 1..6, with at least two junctions')
    rng=random.Random(config.get('seed',42));reject=dict(homogeneous=0,disconnected=0,no_route=0)
    ring=ring_position(config,rows,cols)
    fixed=config.get('layout')
    for attempt in range(1,10001):
        cells=copy.deepcopy(fixed) if fixed else [dict(type='roundabout' if i==ring else ('t' if rng.random()<.5 else 'cross')) for i in range(rows*cols)]
        if len(cells)!=rows*cols:raise ValueError('layout size differs from dimensions')
        ring_cells=[i for i,v in enumerate(cells) if v.get('type')=='roundabout']
        if ring_cells!=([ring] if ring is not None else []):raise ValueError('Layout must contain exactly the configured ring (at most one)')
        for cell in cells:
            if cell['type'] not in ('t','cross','roundabout'):raise ValueError('Unknown junction type')
            if cell['type']=='t':cell.setdefault('missing_arm',rng.choice(ARM_NAMES))
            if cell.get('missing_arm') not in (None,)+ARM_NAMES:raise ValueError('Invalid missing arm')
            if cell['type']!='t' and cell.get('missing_arm') is not None:raise ValueError('Only a T can have a missing arm')
        missing={'%d,%d'%divmod(i,cols):v['missing_arm'] for i,v in enumerate(cells) if v['type']=='t'}
        reason=None
        if rows*cols-(ring is not None)>=2 and not {'t','cross'}<={v['type'] for v in cells}:reason='homogeneous'
        elif not connected(rows,cols,missing):reason='disconnected'
        d=dict(junctions=[],movements=[],seams=[])
        for i,v in enumerate(cells):
            if v['type']=='roundabout':
                base=i*10000
                d['junctions'].append(dict(id=base+10,arms={name:base+k+1 for k,name in enumerate(ARM_NAMES)}))
                for a in range(4):
                    for b in range(4):
                        for lane in (1,2):
                            d['movements'].append(dict(road=base+a+1,lane=-lane,target_road=base+b+1,target_lane=lane,movement='roundabout',junction_id=base+10))
                continue
            c=check(dict(type=v['type'],**({'missing_arm':v['missing_arm']} if v['type']=='t' else {})))
            base=i*10000;arms={ARM_NAMES[a]:base+a+1 for a in active_arms(c)}
            d['junctions'].append(dict(id=base+10,arms=arms))
            for a,lanes in arm_turns(c).items():
                for lane,turns in enumerate(lanes,1):
                    for turn in turns:d['movements'].append(dict(road=base+a+1,lane=-lane,target_road=base+(a+TURN_OFFSETS[turn])%4+1,target_lane=lane,movement=turn,junction_id=base+10))
        for i,j in enumerate(d['junctions']):
            r,col=divmod(i,cols)
            for valid,k,arm,other in [(col+1<cols,i+1,'east','west'),(r+1<rows,i+cols,'north','south')]:
                if valid and arm in j['arms'] and other in d['junctions'][k]['arms']:
                    d['seams'].append(dict(road_a=j['arms'][arm],road_b=d['junctions'][k]['arms'][other]))
        if not reason:
            try:itinerary(d)
            except ValueError:reason='no_route'
        if reason:
            reject[reason]+=1
            if fixed:raise ValueError('Fixed layout rejected: '+reason)
            continue
        return cells,d,dict(attempts=attempt,rejected=reject,proposal=dict(t=.5,cross=.5,missing_directions={s:.25 for s in ARM_NAMES}),
                            ring_cell=ring,
                            acceptance='both ordinary types present when at least two ordinary cells; connected, legal logical-cell itinerary; accepted frequencies are conditional')
    raise ValueError('No feasible sampled layout in 10000 attempts')


def bridge(root, a, b, rid, na, nb, width):
    ga=a.find('./planView/geometry');gb=b.find('./planView/geometry')
    x,y=float(ga.get('x')),float(ga.get('y'));tx,ty=float(gb.get('x')),float(gb.get('y'))
    length=math.hypot(tx-x,ty-y);heading=math.atan2(ty-y,tx-x)
    if length<100:raise ValueError('Spacing must leave at least 100m for transition')
    def defs(sign):
        return na if na==nb else [xodr.LaneDef(20.,length-20.,na,nb,sign*3)]
    r=xodr.create_road(xodr.Line(length),rid,left_lanes=defs(1),right_lanes=defs(-1),lane_width=width)
    r.planview.set_start_point(x,y,heading);r.planview.adjust_geometries()
    # Proxy approach objects provide the template lane endpoints to the official
    # lane-link API; only their generated links are copied onto the real tiles.
    pa=xodr.create_road(xodr.Line(150),int(a.get('id')),left_lanes=na,right_lanes=na,lane_width=width)
    pb=xodr.create_road(xodr.Line(150),int(b.get('id')),left_lanes=nb,right_lanes=nb,lane_width=width)
    pa.add_predecessor(xodr.ElementType.road,rid,xodr.ContactPoint.start)
    r.add_predecessor(xodr.ElementType.road,pa.id,xodr.ContactPoint.start)
    r.add_successor(xodr.ElementType.road,pb.id,xodr.ContactPoint.start)
    pb.add_predecessor(xodr.ElementType.road,rid,xodr.ContactPoint.end)
    ids_a=list(range(-na,0))+list(range(1,na+1));ids_b=list(range(-nb,0))+list(range(1,nb+1))
    xodr.create_lane_links_from_ids(pa,r,ids_a,[-n for n in ids_a])
    xodr.create_lane_links_from_ids(r,pb,ids_b,ids_b)
    for real,proxy in ((a,pa),(b,pb)):
        xml=proxy.get_element();link=real.find('link')
        for old in list(link.findall('predecessor')):link.remove(old)
        link.insert(0,copy.deepcopy(xml.find('./link/predecessor')))
        for lane in real.findall('./lanes/laneSection/*/lane'):
            lid=int(lane.get('id'))
            if not lid:continue
            ll=lane.find('link')
            if ll is not None:
                for old in list(ll.findall('predecessor')):ll.remove(old)
            source=xml.find("./lanes/laneSection/*/lane[@id='%s']/link/predecessor"%lid)
            if source is not None:
                if ll is None:ll=ET.Element('link');lane.insert(0,ll)
                ll.insert(0,copy.deepcopy(source))
    xml=r.get_element();xml.set('name','heterogeneous_transition_%s_to_%s'%(na,nb))
    for section in xml.findall('./lanes/laneSection'):
        section[:]=section.findall('left')+section.findall('center')+section.findall('right')
        for lane in section.findall('./*/lane'):
            for mark in lane.findall('roadMark'):
                mark.set('laneChange','both' if mark.get('type')=='broken' else 'none')
    stitch_edges(xml,a,b)
    root.append(xml)
    return dict(road_a=int(a.get('id')),road_b=int(b.get('id')),connector_road=rid,
                lanes_a=na,lanes_b=nb,through_lanes=min(na,nb),length_m=length,
                policy='outer lane tapers; merge before narrowing, no many-to-one teleport')


def build(config,out):
    out=map_output(out)
    if out.exists() and any(out.iterdir()):raise ValueError('Output must be empty')
    c=dict(config);c.setdefault('rows',3);c.setdefault('columns',3);c.setdefault('seed',42);c.setdefault('spacing_m',500.)
    c.setdefault('include_roundabout',False)
    ring=ring_position(c,c['rows'],c['columns'])
    if ring is not None:c['roundabout_position']=list(divmod(ring,c['columns']))
    minimum=450 if ring is not None else 445
    if not math.isfinite(c['spacing_m']) or c['spacing_m']<minimum:raise ValueError('spacing_m must be >=%s'%minimum)
    cells,abstract,sampling=sample(c);c.update(type='heterogeneous',name=c.get('name',out.name),layout=cells,lane_width_m=3.5,arm_length_m=150.,t_probability=.5,cross_probability=.5)
    root=ET.Element('OpenDRIVE');h=ET.SubElement(root,'header',revMajor='1',revMinor='4',name=c['name'],version='1',north='0',south='0',east='0',west='0')
    ET.SubElement(h,'geoReference').text='+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs'
    d=dict(schema_version=1,config=c,junctions=[],movements=[],seams=[],signals={},roundabouts=[],signal_policy='one_signal_per_incoming_lane',sampling=sampling,status='generated_requires_validation')
    out.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(dir=str(out.parent)) as td:
        for i,cell in enumerate(cells):
            cfg=dict(type=cell['type'],name='tile',rows=1,columns=1,arm_length_m=150.,roadside_props=False)
            if cell['type']=='t':cfg['missing_arm']=cell['missing_arm']
            folder=Path(td)/str(i)
            if cell['type']=='roundabout':
                from roundabout import generate as generate_ring
                local=generate_ring(folder,radius=30.,arm_length=120.,width=3.5)
            else:local=generate(cfg,folder)
            tile=ET.parse(folder/'map.xodr').getroot()
            if cell['type']=='roundabout':
                from roundabout_sidewalks import add_perimeter
                local['sidewalk_connections']=add_perimeter(tile,local)
            base=i*10000;r,col=divmod(i,c['columns']);dx,dy=col*c['spacing_m'],r*c['spacing_m']
            for road in tile.findall('road'):
                road.set('id',str(base+int(road.get('id'))))
                if road.get('junction')!='-1':road.set('junction',str(base+int(road.get('junction'))))
                for g in road.findall('./planView/geometry'):
                    g.set('x',str(float(g.get('x'))+dx));g.set('y',str(float(g.get('y'))+dy))
                for link in road.findall('./link/*'):link.set('elementId',str(base+int(link.get('elementId'))))
                for sig in road.findall('./signals/signal'):
                    sig.set('id',str(base+int(sig.get('id'))));pos=sig.find('positionInertial')
                    if pos is not None:pos.set('x',str(float(pos.get('x'))+dx));pos.set('y',str(float(pos.get('y'))+dy))
                for obj in road.findall('./objects/object'):obj.set('id',str(base+int(obj.get('id'))))
                root.append(road)
            for j in tile.findall('junction'):
                j.set('id',str(base+int(j.get('id'))))
                for co in j.findall('connection'):
                    for key in ('incomingRoad','connectingRoad'):co.set(key,str(base+int(co.get(key))))
                for ctrl in j.findall('controller'):ctrl.set('id',str(base+int(ctrl.get('id'))))
                root.append(j)
            for ctrl in tile.findall('controller'):
                ctrl.set('id',str(base+int(ctrl.get('id'))))
                for co in ctrl.findall('control'):co.set('signalId',str(base+int(co.get('signalId'))))
                root.append(ctrl)
            if cell['type']=='roundabout':
                # One logical cell contains four real OpenDRIVE access junctions.
                # Retain every road/connection/controller, not just the first T.
                for arm in range(1,5):add_edges(root.find("./road[@id='%s']"%(base+arm)))
                j=dict(id=10,roads=[1,2,3,4],lengths=[120.]*4,arms={name:k+1 for k,name in enumerate(ARM_NAMES)},missing_arm=None)
                ring_metadata=dict(cell_id=base+10,center=[dx,dy],junction_ids=[base+x['id'] for x in local['junctions']],
                                   ring_roads=[base+x for x in local['ring_roads']],ring_continuations=[base+x for x in local['ring_continuations']],connections=[])
                for original in local['connections']:
                    entry=dict(original)
                    for key in ('road','source','target'):entry[key]+=base
                    ring_metadata['connections'].append(entry)
                d['roundabouts'].append(ring_metadata)
                ring_metadata['sidewalk_connections']=[]
                for original in local['sidewalk_connections']:
                    entry=dict(original)
                    for key in ('road','source','target'):entry[key]+=base
                    ring_metadata['sidewalk_connections'].append(entry)
                local['movements']=[dict(move,road=move['road']-base,target_road=move['target_road']-base,junction_id=10) for move in abstract['movements'] if move['junction_id']==base+10]
            else:j=copy.deepcopy(local['junctions'][0])
            j.update(id=base+10,center=[dx,dy],type=cell['type'],incoming_lanes=local['config']['incoming_lanes'],template_config=local['config'])
            j['opendrive_junction_ids']=[base+x['id'] for x in local['junctions']]
            j['roads']=[base+x for x in j['roads']];j['arms']={k:base+v for k,v in j['arms'].items()};d['junctions'].append(j)
            for move in local['movements']:
                for key in ('road','target_road','junction_id'):move[key]+=base
                d['movements'].append(move)
            for sid,spec in local['signals'].items():
                spec['road_id']+=base;spec['junction_id']+=base;d['signals'][str(base+int(sid))]=spec
    roads={int(r.get('id')):r for r in root.findall('road')}
    owners={rid:j for j in d['junctions'] for rid in j['roads']}
    for i,seam in enumerate(abstract['seams']):
        a,b=seam['road_a'],seam['road_b']
        d['seams'].append(bridge(root,roads[a],roads[b],900000+i,owners[a]['incoming_lanes'],owners[b]['incoming_lanes'],3.5))
    d['junction_route']=[v['junction_id'] for v in itinerary(d)]
    root[:]=[h]+root.findall('road')+root.findall('controller')+root.findall('junction')
    from signal_placement import apply as place_signals
    place_signals(root,d)
    out.mkdir(parents=True,exist_ok=True);ET.indent(root);ET.ElementTree(root).write(out/'map.xodr',encoding='utf-8',xml_declaration=True)
    d['opendrive_sha256']=hashlib.sha256((out/'map.xodr').read_bytes()).hexdigest()
    for name,data in [('config.json',c),('manifest.json',d)]: (out/name).write_text(json.dumps(data,indent=2))
    write_default_rules(out,d)
    print('Generated heterogeneous:',out,'types',[v['type'] for v in cells],'transitions',len(d['seams']),'sampling',sampling)
    return d


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--config',required=True);p.add_argument('--out',required=True);a=p.parse_args()
    build(json.loads(Path(a.config).read_text(encoding='utf-8-sig')),a.out)
