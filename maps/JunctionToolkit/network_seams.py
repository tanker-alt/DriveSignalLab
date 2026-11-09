"""Shared start-to-start OpenDRIVE stitching and generator-time checks."""
import math
import xml.etree.ElementTree as ET


def stitch(a, b):
    ga=a.find('./planView/geometry'); gb=b.find('./planView/geometry')
    gap=math.hypot(float(ga.get('x'))-float(gb.get('x')),float(ga.get('y'))-float(gb.get('y')))
    angle=float(ga.get('hdg'))-float(gb.get('hdg'))
    if gap>.001 or abs(math.cos(angle)+1)>.000001:
        raise ValueError('Seam endpoints must coincide with opposing reference headings')
    for src,dst in ((a,b),(b,a)):
        lanes=src.findall('./lanes/laneSection/*/lane')
        target={int(l.get('id')):l for l in dst.findall('./lanes/laneSection/*/lane')}
        for lane in lanes:
            lid=int(lane.get('id'))
            if lid and (-lid not in target or target[-lid].get('type')!=lane.get('type')):
                raise ValueError('Seam lane mismatch: %s/%s to %s; explicit transition required' % (src.get('id'),lid,dst.get('id')))
        link=src.find('link')
        if link is None: link=ET.Element('link');src.insert(0,link)
        for old in list(link.findall('predecessor')):link.remove(old)
        link.insert(0,ET.Element('predecessor',elementType='road',elementId=dst.get('id'),contactPoint='start'))
        for lane in lanes:
            lid=int(lane.get('id'))
            if not lid:continue
            ll=lane.find('link')
            if ll is None:ll=ET.Element('link');lane.insert(0,ll)
            for old in list(ll.findall('predecessor')):ll.remove(old)
            ll.insert(0,ET.Element('predecessor',id=str(-lid)))


def stitch_manifest(root, manifest):
    roads={int(r.get('id')):r for r in root.findall('road')}
    for seam in manifest['seams']:
        stitch(roads[seam['road_a']],roads[seam['road_b']])
    manifest['seam_policy']='start-to-start, reciprocal road/lane predecessors, inverted lane IDs'
