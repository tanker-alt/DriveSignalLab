"""Concrete visual caps on native flat OpenDRIVE sidewalks (no collisions)."""
import math
import xml.etree.ElementTree as ET


def build(raw, mesh):
    count = 0
    root = ET.fromstring(raw)
    for road in root.findall('road'):
        geometries = road.findall('./planView/geometry')
        sections = road.findall('./lanes/laneSection')
        if len(geometries) != 1 or len(sections) != 1:
            continue
        g = geometries[0]
        arc = g.find('arc')
        if g.find('line') is None and arc is None:
            continue
        curvature = float(arc.get('curvature')) if arc is not None else 0.
        x0, y0, heading, length = [float(g.get(k)) for k in ('x', 'y', 'hdg', 'length')]
        for sign, side in ((1, 'left'), (-1, 'right')):
            offset = 0.
            bands = []
            for lane in sorted(sections[0].findall('./%s/lane' % side), key=lambda l: abs(int(l.get('id')))):
                width = float(lane.find('width').get('a'))
                if lane.get('type') == 'sidewalk':
                    bands.append((offset, width))
                offset += width
            if not bands:
                continue
            start, width = bands[0][0], sum(b[1] for b in bands)
            # Straight slabs span the whole sidewalk; small arc slabs follow
            # the curved edge with less than 3 cm pavement encroachment.
            tile = width if not curvature else .3
            rows = max(1, int(math.ceil(width/tile)))
            for row in range(rows):
                lateral = sign*(start+(row+.5)*width/rows)
                count_s = max(1, int(math.ceil(length*abs(1-curvature*lateral)/tile)))
                for k in range(count_s):
                    s = (k+.5)*length/count_s
                    angle = heading+curvature*s
                    if curvature:
                        x=x0+(math.sin(angle)-math.sin(heading))/curvature
                        y=y0-(math.cos(angle)-math.cos(heading))/curvature
                    else:
                        x=x0+s*math.cos(heading); y=y0+s*math.sin(heading)
                    x-=lateral*math.sin(angle); y+=lateral*math.cos(angle)
                    asset = 'SideWalk/SideWalkCube'
                    mesh(asset, x, -y, .16-tile/2, tile, -math.degrees(angle))
                    count += 1
    return dict(actor_count=count, surface_z_m=.16, native_collision_retained=True,
                material='CARLA SideWalkCube concrete', visual_collision=False)
