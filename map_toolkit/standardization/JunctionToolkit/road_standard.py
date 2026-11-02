"""Cross/grid urban road standard; portable between Python 3.7 and 3.12."""
import math
import json
from pathlib import Path
import xml.etree.ElementTree as ET
from appearance_standard import settings as appearance_settings

PROFILE = 'urban_cross3_v1'
TURNS = [['left'], ['straight'], ['right']]


def normalize(config):
    c = dict(config)
    if c.get('type') in ('t', 't_junction'):
        template = json.loads((Path(__file__).parent/'templates/t_cross2/profile.json').read_text(encoding='utf-8'))
        # T junctions share the standard surface, markings and scenery profile.
        # Their three active approaches use two incoming/outgoing lanes each.
        fixed = dict(standard_profile=PROFILE, incoming_lanes=2, lane_width_m=3.5,
                     sidewalk_width_m=2., curb_width_m=.2, curb_height_m=.15,
                     corner_radius_m=12., road_class='urban_uniform', roadside_props=False)
        for key, value in fixed.items():
            c[key] = template.get(key, value)
        c.setdefault('shoulder_enabled', False)
        shoulder = .5 if c['shoulder_enabled'] else 0.
        c['shoulder_width_m'] = shoulder
        c['junction_radius_m'] = 19. + shoulder
        c['corner_radius_m'] = c['junction_radius_m']-2*c['lane_width_m']-shoulder
        c.setdefault('spacing_m', 350.)
        c.setdefault('arm_length_m', 150.)
        return c
    if c.get('type') == 'grid':
        c['type'] = 'cross'
    fixed = dict(standard_profile=PROFILE, incoming_lanes=3, lane_width_m=3.5,
                 allowed_turns=TURNS, sidewalk_width_m=2., curb_width_m=.2,
                 curb_height_m=.15, corner_radius_m=12., road_class='urban_uniform',
                 roadside_props=False)
    for key, value in fixed.items():
        if key in c and c[key] != value:
            raise ValueError('%s is fixed to %r by %s' % (key, value, PROFILE))
        c[key] = value
    if 'arm_turns' in c or 'missing_arm' in c:
        raise ValueError('Standard cross/grid does not accept arm-specific turns or missing arms')
    c.setdefault('shoulder_enabled', False)
    if type(c['shoulder_enabled']) is not bool:
        raise ValueError('shoulder_enabled must be boolean')
    shoulder = .5 if c['shoulder_enabled'] else 0.
    radius = 10.5 + shoulder + 12.
    for key, value in [('shoulder_width_m', shoulder), ('junction_radius_m', radius)]:
        if key in c and c[key] != value:
            raise ValueError('%s is derived automatically as %s' % (key, value))
        c[key] = value
    c.setdefault('spacing_m', 350.)
    c.setdefault('arm_length_m', 150.)
    return c


def node(parent, tag, **attrs):
    return ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})


def stop_s(length):
    # Crosswalk spans [length-6, length-2]; 1m from downstream stop-paint edge.
    return length - 7.15


def mark(lane, station, kind, color='white'):
    m = node(lane, 'roadMark', sOffset=station, type=kind, weight='standard',
             color=color, width=.15, laneChange='none' if kind != 'broken' else 'both')
    if kind in ('solid solid', 'broken'):
        t = node(m, 'type', name=kind, width=.45 if kind == 'solid solid' else .15)
        for offset in (-.15, .15) if kind == 'solid solid' else (0.,):
            node(t, 'line', length=0 if kind == 'solid solid' else 3,
                 space=0 if kind == 'solid solid' else 6, tOffset=offset,
                 sOffset=0, rule='no passing' if kind == 'solid solid' else 'none', width=.15)


def raised_lane(side, lid, width, height):
    lane = node(side, 'lane', id=lid, type='sidewalk', level='false')
    node(lane, 'width', sOffset=0, a=width, b=0, c=0, d=0)
    node(lane, 'height', sOffset=0, inner=height, outer=height)
    return lane


def rectangle(objects, oid, name, kind, station, lateral, length, width):
    obj = node(objects, 'object', id=oid, name=name, type=kind, s=station, t=lateral,
               zOffset=.02, orientation='none', hdg=0, pitch=0, roll=0,
               length=length, width=width, height=0)
    outline = node(obj, 'outline', closed='true', outer='true')
    for i, (u, v) in enumerate([(-length/2, -width/2), (length/2, -width/2),
                                 (length/2, width/2), (-length/2, width/2)]):
        node(outline, 'cornerLocal', id=i, u=u, v=v, z=0, height=0)


def approach(road, c, length):
    section = road.find('./lanes/laneSection')
    for lane in section.findall('./*/lane'):
        lid = int(lane.get('id'))
        for old in lane.findall('roadMark'):
            lane.remove(old)
        if lid == 0:
            mark(lane, 0, 'solid solid', 'yellow')
        elif abs(lid) == c['incoming_lanes']:
            mark(lane, 0, 'solid')
        else:
            mark(lane, 0, 'broken')
            solid_length = appearance_settings()['markings']['approach_solid_length_m']
            if lid < 0 and solid_length > 0:
                mark(lane, stop_s(length)-solid_length, 'solid')
        # Stop the longitudinal paint before the crossing, both directions.
        mark(lane, stop_s(length)-.15 if lid < 0 else length-6, 'none')
    for sign, name in [(1, 'left'), (-1, 'right')]:
        side = section.find(name)
        idx = c['incoming_lanes']+1
        if c['shoulder_enabled']:
            lane = node(side, 'lane', id=sign*idx, type='sidewalk', level='false')
            node(lane, 'width', sOffset=0, a=c['shoulder_width_m'], b=0, c=0, d=0)
            node(lane, 'height', sOffset=0, inner=0, outer=0)
            idx += 1
        raised_lane(side, sign*idx, c['curb_width_m'], c['curb_height_m'])
        raised_lane(side, sign*(idx+1), c['sidewalk_width_m'], c['curb_height_m'])
    objects = road.find('objects')
    if objects is None:
        objects = node(road, 'objects')
    rid = int(road.get('id'))
    rectangle(objects, rid*100, 'pedestrian_crossing', 'crosswalk', length-4, 0, 4, 2*c['incoming_lanes']*c['lane_width_m'])
    rectangle(objects, rid*100+1, 'stop_line', 'none', stop_s(length), -c['incoming_lanes']*c['lane_width_m']/2, .3, c['incoming_lanes']*c['lane_width_m'])
    # Stripe footprints are kept explicitly, rather than relying on importer styling.
    for k in range(int(2*c['incoming_lanes']*c['lane_width_m'])):
        rectangle(objects, rid*100+2+k, 'crosswalk_stripe', 'none', length-4,
                  -c['incoming_lanes']*c['lane_width_m']+.25+k, 4, .5)


def auxiliary(root, rid, name, x, y, heading, length, elevation=0):
    road = node(root, 'road', id=rid, name=name, junction=-1, length=length)
    geometry = node(node(road, 'planView'), 'geometry', s=0, x=x, y=y,
                    hdg=heading, length=length)
    node(node(road, 'elevationProfile'), 'elevation', s=0, a=elevation, b=0, c=0, d=0)
    section = node(node(road, 'lanes'), 'laneSection', s=0)
    node(node(section, 'center'), 'lane', id=0, type='none', level='false')
    return road, geometry, section


def junction_surface(root, c, center_x):
    old = root.find("road[@id='900']")
    if old is not None:
        root.remove(old)
    extent, radius = c['junction_radius_m'], c['corner_radius_m']
    half = extent-radius
    surface, geom, section = auxiliary(root, 900, 'rounded_junction_pavement',
                                  center_x-extent, 0, 0, 2*extent, -.02)
    node(geom, 'line')
    # Piecewise linear sampling of true circular boundary, <= 6mm chord error.
    positive = [(extent-radius*math.cos(i*math.pi/2/48),
                 extent-radius*math.sin(i*math.pi/2/48)) for i in range(49)]
    samples = sorted([(-x, y) for x, y in positive] + positive)
    lanes = surface.find('lanes')
    lanes.remove(section)
    # CARLA optimizes straight reference lines to section endpoints, ignoring
    # intermediate width records when tessellating. Split at each chord endpoint.
    for (x, width), (end_x, end_width) in zip(samples, samples[1:]):
        section = node(lanes, 'laneSection', s=x+extent)
        left = node(section, 'left')
        node(node(section, 'center'), 'lane', id=0, type='none', level='false')
        right = node(section, 'right')
        for sign, side in [(1, left), (-1, right)]:
            lane = node(side, 'lane', id=sign, type='shoulder', level='false')
            node(lane, 'width', sOffset=0, a=width,
                 b=(end_width-width)/(end_x-x), c=0, d=0)
    # Each quarter circle starts at (half, extent), with curb on its left.
    for i in range(4):
        angle = i*math.pi/2
        x = half*math.cos(angle)-extent*math.sin(angle)
        y = half*math.sin(angle)+extent*math.cos(angle)
        _, geom, section = auxiliary(root, 910+i, 'curved_curb_and_sidewalk',
                                      center_x+x, y, -math.pi/2+angle, radius*math.pi/2)
        node(geom, 'arc', curvature=1/radius)
        side = node(section, 'left')
        raised_lane(side, 1, c['curb_width_m'], c['curb_height_m'])
        raised_lane(side, 2, c['sidewalk_width_m'], c['curb_height_m'])

    if c.get('type') in ('t', 't_junction'):
        # Close the absent arm with a continuous pavement edge and sidewalk.
        # Construct north-closed geometry, then rotate to the requested arm.
        for section in surface.findall('./lanes/laneSection'):
            width = section.find('./left/lane/width')
            width.set('a', str(half)); width.set('b', '0')
        for rid in (910, 911):
            root.remove(root.find("road[@id='%s']" % rid))
        _, geom, section = auxiliary(root, 914, 'closed_t_sidewalk',
                                      center_x-extent, half, 0, 2*extent)
        node(geom, 'line')
        side = node(section, 'left')
        raised_lane(side, 1, c['curb_width_m'], c['curb_height_m'])
        raised_lane(side, 2, c['sidewalk_width_m'], c['curb_height_m'])
        from layout import ARM_NAMES
        angle = (ARM_NAMES.index(c.get('missing_arm', 'north'))-3)*math.pi/2
        for road in root.findall('road'):
            if int(road.get('id')) < 900:
                continue
            for geom in road.findall('./planView/geometry'):
                x, y = float(geom.get('x'))-center_x, float(geom.get('y'))
                geom.set('x', str(center_x+x*math.cos(angle)-y*math.sin(angle)))
                geom.set('y', str(x*math.sin(angle)+y*math.cos(angle)))
                geom.set('hdg', str(float(geom.get('hdg'))+angle))


def is_standard(design):
    return design['config'].get('standard_profile') == PROFILE


