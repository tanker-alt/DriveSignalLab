"""Continuous raised curb/sidewalk outside every tapered driving lane section."""
import xml.etree.ElementTree as ET


def node(parent,tag,**attrs):
    return ET.SubElement(parent,tag,{k:str(v) for k,v in attrs.items()})


def link_lane(lane,tag,lid):
    link=lane.find('link')
    if link is None:link=ET.Element('link');lane.insert(0,link)
    for old in list(link.findall(tag)):link.remove(old)
    node(link,tag,id=lid)


def add_edges(road):
    """Widths are relative to outer driving edge, hence follow its polynomial."""
    sections=road.findall('./lanes/laneSection')
    edges=[]
    for section in sections:
        sides={}
        for side,sign in [('left',1),('right',-1)]:
            group=section.find(side)
            if group is None:continue
            existing=[l for l in group.findall('lane') if l.get('type')=='sidewalk']
            if existing:
                if len(existing)!=2:raise ValueError('Expected curb and sidewalk pair')
                sides[sign]=existing;continue
            count=max(abs(int(l.get('id'))) for l in group.findall('lane'))
            lanes=[]
            for offset,width in [(1,.2),(2,2.)]:
                lane=node(group,'lane',id=sign*(count+offset),type='sidewalk',level='false')
                node(lane,'width',sOffset=0,a=width,b=0,c=0,d=0)
                node(lane,'height',sOffset=0,inner=.15,outer=.15);lanes.append(lane)
            sides[sign]=lanes
        edges.append(sides)
    for a,b in zip(edges,edges[1:]):
        for sign in (-1,1):
            for left,right in zip(a[sign],b[sign]):
                link_lane(left,'successor',right.get('id'));link_lane(right,'predecessor',left.get('id'))
    return edges


def stitch_edges(bridge,a,b):
    edges=add_edges(bridge)
    for arm,endpoint,tag,mirror in [(a,edges[0],'predecessor',True),(b,edges[-1],'successor',False)]:
        section=arm.find('./lanes/laneSection')
        for side,sign in [('left',1),('right',-1)]:
            arm_lanes=section.findall("./%s/lane[@type='sidewalk']"%side)
            if len(arm_lanes)!=2:raise ValueError('Approach must have curb and sidewalk')
            bridge_lanes=endpoint[-sign if mirror else sign]
            for src,dst in zip(arm_lanes,bridge_lanes):
                link_lane(src,'predecessor',dst.get('id'));link_lane(dst,tag,src.get('id'))
