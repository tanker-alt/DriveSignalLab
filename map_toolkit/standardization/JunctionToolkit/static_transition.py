import copy, math, xml.etree.ElementTree as ET
from pathlib import Path

TEMPLATES=Path(__file__).parent.parent/'references'/'transition'

def _start(road):
    g=road.find('./planView/geometry')
    return float(g.get('x')), float(g.get('y'))


def _replace_link(road, tag, element_id, contact_point):
    link=road.find('link')
    if link is None:
        link=ET.SubElement(road,'link')
    for n in list(link):
        if n.tag == tag and n.get('elementType') == 'road':
            link.remove(n)
    ET.SubElement(link,tag,{'elementType':'road','elementId':str(element_id),
                             'contactPoint':contact_point})


def _lane_link(section, lane_id, tag, target_id):
    lane=next((n for side in ('left','right') for n in section.findall('./'+side+'/lane')
               if n.get('id') == str(lane_id)), None)
    if lane is None:
        return
    link=lane.find('link')
    if link is None:
        link=ET.SubElement(lane,'link')
    for n in list(link):
        if n.tag == tag:
            link.remove(n)
    ET.SubElement(link,tag,{'id':str(target_id)})


def _clear_lane_links(road):
    for lane in road.findall('./lanes/laneSection/*/lane'):
        link=lane.find('link')
        if link is not None:
            for n in list(link): link.remove(n)


def insert(root, source, target, rid=20000):
    """Insert the RoadRunner lane-transition template into a real gap.

    ``source`` and ``target`` are the two inward-facing approach roads.  Their
    contact points are both at ``start``; the transition therefore runs from
    source.start to target.start.  The template is rotated to that vector and
    its lane count is preserved in the direction of travel.
    """
    a=root.find("./road[@id='%s']"%source); b=root.find("./road[@id='%s']"%target)
    if a is None or b is None:
        raise ValueError('Transition endpoints are missing: %s, %s' % (source,target))
    x,y=_start(a); tx,ty=_start(b)
    dx,dy=tx-x,ty-y; desired=math.hypot(dx,dy)
    if desired < 1.0:
        raise ValueError('Transition endpoints must leave a positive gap (%.3fm)' % desired)
    heading=math.atan2(dy,dx)
    # east_to_west is the 2->3 template in its native direction.  Rotate it
    # onto the seam vector so the 2-lane end is always at source.start.
    name='transition_east_to_west'
    ref=ET.parse(TEMPLATES/(name+'.xodr')).getroot().find('road')
    road=copy.deepcopy(ref); road.set('id',str(rid)); road.set('name','static_lane_transition')
    g0=ref.find('./planView/geometry'); ox,oy=float(g0.get('x')),float(g0.get('y'))
    raw_len=float(ref.get('length')); scale=desired/raw_len
    native_heading=float(g0.get('hdg'))
    rot=heading-native_heading
    cr,sr=math.cos(rot),math.sin(rot)
    for gg in road.findall('./planView/geometry'):
        lx=(float(gg.get('x'))-ox)*scale; ly=(float(gg.get('y'))-oy)*scale
        gg.set('x',str(x+cr*lx-sr*ly)); gg.set('y',str(y+sr*lx+cr*ly))
        gg.set('hdg',str(float(gg.get('hdg'))+rot)); gg.set('length',str(float(gg.get('length'))*scale)); gg.set('s',str(float(gg.get('s'))*scale))
        arc=gg.find('arc')
        if arc is not None and arc.get('curvature') is not None:
            arc.set('curvature',str(float(arc.get('curvature'))/scale))
    road.set('length',str(desired))
    for sec in road.findall('./lanes/laneSection'):
        sec.set('s',str(float(sec.get('s'))*scale))
    for node in road.findall('.//*[@s]'):
        if node.tag in ('laneSection','geometry','signal','object'):
            node.set('s',str(float(node.get('s'))*scale))
    link=road.find('link')
    if link is None: link=ET.SubElement(road,'link')
    for n in list(link): link.remove(n)
    ET.SubElement(link,'predecessor',{'elementType':'road','elementId':str(source),'contactPoint':'start'})
    ET.SubElement(link,'successor',{'elementType':'road','elementId':str(target),'contactPoint':'start'})
    # Remove the old direct seam relation and point both approaches to the
    # retained transition road.  Their junction successors remain untouched.
    _replace_link(a,'predecessor',rid,'start')
    _replace_link(b,'predecessor',rid,'start')
    # At source.start the two reference lines face opposite directions, so
    # the lane IDs are mirrored.  At target.start both references face the
    # same way, so the IDs are preserved.  These explicit lane links are what
    # lets CARLA expose a continuous waypoint graph across the seam.
    _clear_lane_links(a); _clear_lane_links(road); _clear_lane_links(b)
    asec=a.find('./lanes/laneSection'); first=road.find('./lanes/laneSection')
    last=road.findall('./lanes/laneSection')[-1]; bsec=b.find('./lanes/laneSection')
    for lane in asec.findall('./left/lane')+asec.findall('./right/lane'):
        try: lid=int(lane.get('id'))
        except (TypeError,ValueError): continue
        if lid: _lane_link(asec,lid,'predecessor',-lid)
    for lane in first.findall('./left/lane')+first.findall('./right/lane'):
        try: lid=int(lane.get('id'))
        except (TypeError,ValueError): continue
        if lid: _lane_link(first,lid,'predecessor',-lid)
    for lane in last.findall('./left/lane')+last.findall('./right/lane'):
        try: lid=int(lane.get('id'))
        except (TypeError,ValueError): continue
        if lid: _lane_link(last,lid,'successor',lid)
    for lane in bsec.findall('./left/lane')+bsec.findall('./right/lane'):
        try: lid=int(lane.get('id'))
        except (TypeError,ValueError): continue
        if lid: _lane_link(bsec,lid,'predecessor',lid)
    root.append(road)
    return road
