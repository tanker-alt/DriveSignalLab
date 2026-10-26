"""Generate connected OpenDRIVE cross intersections for CARLA 0.9.15.

Run with the generator Python (3.12), not the CARLA Python (3.7).
"""
import argparse
import copy
import hashlib
import json
import math
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from scenariogeneration import xodr
from layout import ARM_NAMES, TURN_OFFSETS, active_arms, arm_turns, is_t
from safety_rules import write_default_rules


def child(parent, tag, **attrs):
    return ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})


def check(c):
    allowed = {'name', 'rows', 'columns', 'spacing_m', 'arm_length_m',
               'incoming_lanes', 'lane_width_m', 'junction_radius_m',
               'allowed_turns', 'sidewalk_width_m', 'seed', 'roadside_props',
               'type', 'missing_arm', 'arm_turns'}
    if set(c) - allowed:
        raise ValueError('Unknown config keys: ' + str(sorted(set(c) - allowed)))
    c = dict(c)
    defaults = dict(name='cross', rows=1, columns=1, spacing_m=344.,
                    arm_length_m=150., incoming_lanes=3, lane_width_m=3.5,
                    junction_radius_m=22., sidewalk_width_m=2., seed=42,
                    roadside_props=True)
    for k, v in defaults.items(): c.setdefault(k, v)
    if c.get('type','cross') not in ('cross','t','t_junction'):
        raise ValueError('Supported geometry types: cross, t')
    if is_t(c):
        c['type']='t'
        c.setdefault('missing_arm','north')
        if (c['rows'],c['columns']) != (1,1):
            raise ValueError('T junction currently supports a single junction only')
        if 'allowed_turns' in c:
            raise ValueError('T junction uses arm_turns per approach, not shared allowed_turns')
    elif 'missing_arm' in c:
        raise ValueError('missing_arm only applies to a T junction')
    for k, low, high in [('rows', 1, 5), ('columns', 1, 5), ('incoming_lanes', 1, 4)]:
        if type(c[k]) is not int or not low <= c[k] <= high:
            raise ValueError('%s must be an integer in [%s,%s]' % (k, low, high))
    for k in ['spacing_m', 'arm_length_m', 'lane_width_m', 'junction_radius_m', 'sidewalk_width_m']:
        if not isinstance(c[k], (int, float)) or not math.isfinite(c[k]):
            raise ValueError(k + ' must be finite')
    if not 3 <= c['lane_width_m'] <= 4.5: raise ValueError('lane_width_m: 3 to 4.5')
    if not 0 <= c['sidewalk_width_m'] <= 5: raise ValueError('sidewalk_width_m: 0 to 5')
    if not 80 <= c['arm_length_m'] <= 500: raise ValueError('arm_length_m: 80 to 500')
    if c['junction_radius_m'] < c['incoming_lanes'] * c['lane_width_m'] + 8:
        raise ValueError('junction_radius_m must exceed half road width by at least 8m')
    if c['spacing_m'] < 2 * (c['junction_radius_m'] + 80):
        raise ValueError('spacing_m must leave at least 80m per inner approach')
    if not is_t(c):
        c.setdefault('allowed_turns', [['left'], ['straight'], ['right']] if c['incoming_lanes'] == 3
                     else [['left', 'straight', 'right'] for _ in range(c['incoming_lanes'])])
    resolved=arm_turns(c)
    if is_t(c):
        c['arm_turns']={ARM_NAMES[i]:turns for i,turns in resolved.items()}
    if type(c['seed']) is not int: raise ValueError('seed must be an integer')
    if type(c['roadside_props']) is not bool: raise ValueError('roadside_props must be boolean')
    return c


def make_tile(c, lengths):
    radius, width, n = c['junction_radius_m'], c['lane_width_m'], c['incoming_lanes']
    od = xodr.OpenDrive(c['name'], revMajor='1', revMinor='4')
    j = xodr.CommonJunctionCreator(10, 'Cross', startnum=100)
    movements = []
    active=active_arms(c);resolved=arm_turns(c)
    for i in active:
        length=lengths[i]
        road = xodr.create_road(xodr.Line(length), id=i+1, left_lanes=n, right_lanes=n, lane_width=width)
        j.add_incoming_road_circular_geometry(road, radius, i*math.pi/2, road_connection='successor')
        od.add_road(road)
    for i in active:
        for index, turns in enumerate(resolved[i], 1):
            for turn in turns:
                dest = (i + TURN_OFFSETS[turn]) % 4
                j.add_connection(i+1, dest+1, -index, index)
                movements.append(dict(road=i+1, lane=-index, target_road=dest+1,
                                      target_lane=index, movement=turn))
    od.add_junction_creator(j)
    od.adjust_roads_and_lanes()
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp)/'tile.xodr'; od.write_xml(str(path)); root = ET.parse(path).getroot()
    if active[0] != 0:
        # OpenDRIVE adjustment anchors the first present road at (0,0,0).
        # Restore the established compass and centre when west is missing.
        first=root.find("./road[@id='{}']/planView/geometry".format(active[0]+1))
        angle=active[0]*math.pi/2-float(first.get('hdg'))
        ca,sa=math.cos(angle),math.sin(angle)
        fx,fy=float(first.get('x')),float(first.get('y'))
        distance=lengths[active[0]]+radius
        tx=lengths[0]+radius-distance*math.cos(active[0]*math.pi/2)
        ty=-distance*math.sin(active[0]*math.pi/2)
        for geom in root.findall('./road/planView/geometry'):
            x,y=float(geom.get('x'))-fx,float(geom.get('y'))-fy
            geom.set('x',str(tx+ca*x-sa*y));geom.set('y',str(ty+sa*x+ca*y))
            geom.set('hdg',str(float(geom.get('hdg'))+angle))
    for geom in root.findall('./road/planView/geometry'):
        s = geom.find('spiral')
        if s is not None:
            a, b = float(s.get('curvStart')), float(s.get('curvEnd'))
            if abs(a-b) < 1e-8:
                geom.remove(s)
                if abs((a+b)/2) < 1e-8: child(geom, 'line')
                else: child(geom, 'arc', curvature=(a+b)/2)
    for road in root.findall('road'):
        rid = int(road.get('id'))
        if rid not in range(1, 5):
            # No lane stripes inside the intersection.
            for lane in road.findall('./lanes/laneSection/*/lane'):
                for mark in lane.findall('roadMark'): lane.remove(mark)
            continue
        length = lengths[rid-1]
        for lane in road.findall('./lanes/laneSection/*/lane'):
            for mark in lane.findall('roadMark'): lane.remove(mark)
            lid = int(lane.get('id'))
            child(lane, 'roadMark', sOffset=0, type='solid' if lid==0 or abs(lid)==n else 'broken',
                  weight='standard', color='yellow' if lid==0 else 'white', width=.15, laneChange='none')
        if c['sidewalk_width_m']:
            section = road.find('./lanes/laneSection')
            for sign, side in [(1, 'left'), (-1, 'right')]:
                # Raised sidewalks, stopping at the junction edge; no pedestrian nav claim.
                lane = child(section.find(side), 'lane', id=sign*(n+1), type='sidewalk', level='false')
                child(lane, 'width', sOffset=0, a=c['sidewalk_width_m'], b=0, c=0, d=0)
                child(lane, 'height', sOffset=0, inner=.15, outer=.15)
        signals = road.find('signals')
        if signals is None: signals = child(road, 'signals')
        for lane_index in range(1,n+1):
            sid=1000+rid*10+lane_index
            cid=2000+rid*10+lane_index
            s = child(signals, 'signal', s=length-3, t=-(lane_index-.5)*width-6.0, id=sid,
                      name='LaneSignal_{}_{}'.format(rid,lane_index), dynamic='yes', orientation='+', zOffset=5.5,
                      country='OpenDRIVE', type='1000001', subtype=-1, value=-1, height=3, width=.3, hOffset=0)
            child(s, 'validity', fromLane=-lane_index, toLane=-lane_index)
            ctrl = child(root, 'controller', id=cid, name='Controller_'+str(sid), sequence=lane_index)
            child(ctrl, 'control', signalId=sid, type=0)
            child(root.find('junction'), 'controller', id=cid, type=0, sequence=lane_index)
    surface = child(root, 'road', name='junction_pavement', length=2*radius, id=900, junction=-1)
    g = child(child(surface, 'planView'), 'geometry', s=0, x=lengths[0], y=radius, hdg=0, length=2*radius)
    child(g, 'line')
    child(child(surface, 'elevationProfile'), 'elevation', s=0, a=-.02, b=0, c=0, d=0)
    ls = child(child(surface, 'lanes'), 'laneSection', s=0)
    child(child(ls, 'center'), 'lane', id=0, type='none', level='false')
    lane = child(child(ls, 'right'), 'lane', id=-1, type='shoulder', level='false')
    child(lane, 'width', sOffset=0, a=2*radius, b=0, c=0, d=0)
    return root, movements


def generate(config, out):
    c = check(config)
    out = Path(out)
    if out.exists() and any(out.iterdir()): raise ValueError('Output must be empty (existing experiments are never overwritten)')
    root = ET.Element('OpenDRIVE')
    h = child(root, 'header', revMajor=1, revMinor=4, name=c['name'], version=1,
              date='2026-10-23', north=0, south=0, east=0, west=0)
    child(h, 'geoReference').text = '+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs'
    manifest = dict(schema_version=1, signal_policy='one_signal_per_incoming_lane', config=c, junctions=[], movements=[], seams=[], signals={},
                    status='generated_requires_validation', coordinate_system='OpenDRIVE XY; CARLA flips Y')
    road_nodes = {}
    for row in range(c['rows']):
        for col in range(c['columns']):
            idx = row*c['columns']+col; base = idx*10000
            neighbors = [(row,col-1),(row-1,col),(row,col+1),(row+1,col)]
            lengths = [(c['spacing_m']/2-c['junction_radius_m']) if 0<=r<c['rows'] and 0<=k<c['columns'] else c['arm_length_m'] for r,k in neighbors]
            tile, moves = make_tile(c, lengths)
            cx, cy = col*c['spacing_m'], row*c['spacing_m']
            dx = cx-(lengths[0]+c['junction_radius_m'])
            for road in tile.findall('road'):
                rid = int(road.get('id')); road.set('id', str(base+rid))
                if road.get('junction') != '-1': road.set('junction', str(base+int(road.get('junction'))))
                for g in road.findall('./planView/geometry'):
                    g.set('x',str(float(g.get('x'))+dx)); g.set('y',str(float(g.get('y'))+cy))
                for link in road.findall('./link/*'): link.set('elementId',str(base+int(link.get('elementId'))))
                for signal in road.findall('./signals/signal'):
                    signal.set('id',str(base+int(signal.get('id'))))
                    # Explicit pose bypasses CARLA's automatic roadside relocation.
                    # Keep s/validity as the independent control/stop association.
                    g=road.find('./planView/geometry');heading=float(g.get('hdg'))
                    station=float(signal.get('s'));lateral=float(signal.get('t'))
                    child(signal,'positionInertial',
                        x=float(g.get('x'))+station*math.cos(heading)-lateral*math.sin(heading),
                        y=float(g.get('y'))+station*math.sin(heading)+lateral*math.cos(heading),
                        z=0,hdg=heading,pitch=0,roll=0)
                root.append(road); road_nodes[base+rid] = road
            junction = tile.find('junction'); junction.set('id',str(base+10))
            for conn in junction.findall('connection'):
                for key in ['incomingRoad','connectingRoad']: conn.set(key,str(base+int(conn.get(key))))
            for ctrl in junction.findall('controller'): ctrl.set('id',str(base+int(ctrl.get('id'))))
            root.append(junction)
            for ctrl in tile.findall('controller'):
                ctrl.set('id',str(base+int(ctrl.get('id'))))
                for entry in ctrl.findall('control'): entry.set('signalId',str(base+int(entry.get('signalId'))))
                root.append(ctrl)
            active=active_arms(c)
            manifest['junctions'].append(dict(id=base+10, center=[cx,cy], roads=[base+i+1 for i in active],
                lengths=[lengths[i] for i in active],arms={ARM_NAMES[i]:base+i+1 for i in active}))
            for m in moves:
                m.update(road=m['road']+base, target_road=m['target_road']+base, junction_id=base+10)
                manifest['movements'].append(m)
            for slot,i in enumerate(active):
                for lane_index in range(1,c['incoming_lanes']+1):
                    manifest['signals'][str(base+1000+(i+1)*10+lane_index)] = dict(road_id=base+i+1, junction_id=base+10,
                        phase_slot=slot*c['incoming_lanes']+lane_index-1, approach_slot=slot,
                        reference_s=lengths[i]-6, incoming_lanes=[-lane_index])
    # Each pair of inward-oriented arms meets start-to-start. Lane signs invert.
    for row in range(c['rows']):
        for col in range(c['columns']):
            base=(row*c['columns']+col)*10000
            pairs=[]
            if col+1<c['columns']: pairs.append((base+3,base+10000+1))
            if row+1<c['rows']: pairs.append((base+4,base+c['columns']*10000+2))
            for a,b in pairs:
                for src,dst in [(a,b),(b,a)]:
                    road=road_nodes[src]
                    link=road.find('link'); pre=ET.Element('predecessor',dict(elementType='road',elementId=str(dst),contactPoint='start'));link.insert(0,pre)
                    for lane in road.findall('./lanes/laneSection/*/lane'):
                        lid=int(lane.get('id'))
                        if not lid: continue
                        ln=lane.find('link')
                        if ln is None: ln=ET.Element('link');lane.insert(0,ln)
                        child(ln,'predecessor',id=-lid)
                manifest['seams'].append(dict(road_a=a,road_b=b))
    # Keep the OpenDRIVE element order compatible with its schema.
    root[:] = [h]+root.findall('road')+root.findall('controller')+root.findall('junction')
    out.mkdir(parents=True,exist_ok=True)
    ET.indent(root)
    path=out/'map.xodr';ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
    manifest['opendrive_sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    (out/'config.json').write_text(json.dumps(c,indent=2),encoding='utf-8')
    write_default_rules(out, manifest)
    print('Generated:',path,'junctions:',len(manifest['junctions']),'movements:',len(manifest['movements']))
    return manifest


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',required=True);p.add_argument('--out',required=True)
    a=p.parse_args();generate(json.loads(Path(a.config).read_text(encoding='utf-8-sig')),a.out)
