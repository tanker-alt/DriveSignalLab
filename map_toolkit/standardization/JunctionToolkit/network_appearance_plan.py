"""Pure XML/metadata appearance plan, shared by static checks and renderers."""
import xml.etree.ElementTree as ET
from appearance_standard import settings


def plan(design,raw,mode='simple'):
    if mode not in ('simple','complete','off'):raise ValueError('Unknown network appearance mode')
    root=ET.fromstring(raw);roads={int(r.get('id')):r for r in root.findall('road')}
    ring_roads={rid for ring in design.get('roundabouts',[]) for rid in ring['ring_roads']+ring['ring_continuations']}
    # Also retain support for the standalone ring's simple preview.
    ring_roads.update(design.get('ring_roads',[])+design.get('ring_continuations',[]))
    grouped={}
    for sid,s in design['signals'].items():grouped.setdefault(s['road_id'],[]).append((sid,s))
    records=[]
    for rid in sorted(set(grouped)|ring_roads):
        r=roads[rid];sections=r.findall('./lanes/laneSection')
        if len(sections)!=1:raise ValueError('Signal/paint road must have one lane section: %s'%rid)
        lanes=sections[0].findall("./right/lane[@type='driving']")
        ids=sorted(int(l.get('id')) for l in lanes)
        if ids!=list(range(-len(lanes),0)) or not lanes:raise ValueError('Noncontiguous incoming lanes')
        widths=[]
        for lane in lanes:
            ws=lane.findall('width')
            if len(ws)!=1 or any(float(ws[0].get(k,0))!=0 for k in ('b','c','d')):raise ValueError('Variable width on signal road')
            widths.append(float(ws[0].get('a')))
        if len(set(widths))!=1:raise ValueError('Nonuniform lane widths within approach')
        entries=grouped.get(rid,[]);length=float(r.get('length'));stop=None;signal_s=None
        if entries:
            owned=[lane for _,s in entries for lane in s['incoming_lanes']]
            if sorted(owned)!=ids:raise ValueError('Signal ownership does not cover lanes exactly: %s'%rid)
            stations={float(s['reference_s']) for _,s in entries}
            heads={float(s.get('signal_s',s['reference_s']+3)) for _,s in entries}
            if len(stations)!=1 or len(heads)!=1:raise ValueError('Inconsistent signal stations')
            stop=stations.pop();signal_s=heads.pop()
            if not 0<stop<signal_s<length:raise ValueError('Signal/stop outside road')
            for sid,s in entries:
                xml=r.find("./signals/signal[@id='%s']"%sid)
                if xml is None or abs(float(xml.get('s'))-signal_s)>1e-6:raise ValueError('Signal XML differs from manifest')
        circulating=rid in ring_roads
        records.append(dict(road_id=rid,length=length,incoming_lanes=len(lanes),lane_width_m=widths[0],
                            stop_s=stop,signal_s=signal_s,signal_ids=[sid for sid,_ in entries],
                            circulating=circulating,crosswalk=not circulating,centerline=not circulating,
                            shoulder_width_m=design['config'].get('shoulder_width_m',0)))
    return dict(mode=mode,roads=records,style=settings(design['config']),grass=False,trees=False,
                roadside_props=False,ring_perimeter=mode=='complete' and any(r.get('sidewalk_connections') for r in design.get('roundabouts',[])),
                native_transition_markings=True,carla_validation_performed=False)
