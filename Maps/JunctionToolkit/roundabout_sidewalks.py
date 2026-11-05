"""Outer perimeter follows entry, circulating arc and exit, leaving mouths open."""
from transition_sidewalks import add_edges, link_lane
from roundabout_links import verify
import xml.etree.ElementTree as ET


def add_perimeter(root,design):
    roads={int(r.get('id')):r for r in root.findall('road')}
    # Only the outer lane entry/exit connectors belong to the perimeter.
    # Curbs on the through-ring connectors would cut across the access roads.
    for rid in [1,2,3,4]+design['ring_roads']:
        add_edges(roads[rid])
    paths=[]
    for original in design['connections']:
        if abs(original['source_lane'])!=2 or abs(original['target_lane'])!=2:continue
        if original['source'] not in (1,2,3,4) and original['target'] not in (1,2,3,4):continue
        connector=roads[original['road']];add_edges(connector)
        junction=root.find("./junction[@id='%s']"%connector.get('junction'))
        connection=junction.find("./connection[@connectingRoad='%s']"%original['road'])
        for offset in (1,2):
            path=dict(road=original['road'],lane=-1-offset,source=original['source'],
                      source_lane=original['source_lane']-offset,target=original['target'],
                      target_lane=original['target_lane']+(offset if original['target_lane']>0 else -offset))
            lane=connector.find("./lanes/laneSection/right/lane[@id='%s']"%path['lane'])
            for tag,key,lkey in [('predecessor','source','source_lane'),('successor','target','target_lane')]:
                link_lane(lane,tag,path[lkey])
                parent=roads[path[key]]
                contact=connector.find('./link/'+tag).get('contactPoint')
                section=parent.findall('./lanes/laneSection')[0 if contact=='start' else -1]
                other=section.find("./*/lane[@id='%s']"%path[lkey])
                link_lane(other,'predecessor' if contact=='start' else 'successor',path['lane'])
            ET.SubElement(connection,'laneLink',{'from':str(path['source_lane']),'to':str(path['lane'])})
            paths.append(path)
    verify(root,paths)
    return paths
