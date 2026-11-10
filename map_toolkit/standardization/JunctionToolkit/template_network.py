"""Build a full cross grid, then prune planned arms with their dependent objects."""
import argparse
import hashlib
import json
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from generate import generate, make_tile
from runtime_config import map_output
from safety_rules import write_default_rules
from road_standard import junction_surface
from network_plan import plan
from layout import ARM_NAMES, arm_turns
from network_seams import stitch_manifest


def build(c, out):
    out = map_output(out)
    if out.exists() and any(out.iterdir()):
        raise ValueError('Output must be empty; preserve existing maps')
    c, attempts = plan(c)
    types = c['junction_types']
    missing = c['missing_arms']
    cfg = {k: v for k, v in c.items() if k not in ('junction_types', 'include_roundabout', 'missing_arms', 't_probability')}
    cfg.update(type='cross', incoming_lanes=3)
    with tempfile.TemporaryDirectory(dir=str(out.parent)) as td:
        folder = Path(td)/'full_cross_grid'
        m = generate(cfg, folder)
        root = ET.parse(folder/'map.xodr').getroot()
    removed = set()
    for j in m['junctions']:
        idx = (j['id']-10)//10000
        row, col = divmod(idx, cfg['columns'])
        if types.get('%d,%d' % (row, col)) == 't':
            arm = missing['%d,%d' % (row,col)]
            removed.add(idx*10000+ARM_NAMES.index(arm)+1)
            j['missing_arm'] = arm
    # A deleted arm invalidates every internal movement entering OR leaving it.
    for road in root.findall('road'):
        if road.get('junction') != '-1' and any(
                n.get('elementType') == 'road' and int(n.get('elementId')) in removed
                for n in road.findall('./link/*')):
            removed.add(int(road.get('id')))
    for road in list(root.findall('road')):
        if int(road.get('id')) in removed:
            root.remove(road)
            continue
        link = road.find('link')
        if link is not None:
            for n in list(link):
                if n.get('elementType') == 'road' and int(n.get('elementId')) in removed:
                    link.remove(n)
                    for ll in road.findall('./lanes/laneSection/*/lane/link'):
                        for ref in list(ll):
                            if ref.tag == n.tag:
                                ll.remove(ref)
    for j in root.findall('junction'):
        for con in list(j.findall('connection')):
            if any(int(con.get(k)) in removed for k in ('incomingRoad','connectingRoad')):
                j.remove(con)
    # Recompute T movements: simply deleting cross movements leaves one dead
    # incoming lane on each arm. Keep the full-grid approaches and seams intact.
    for j in m['junctions']:
        base=j['id']-10
        if not j.get('missing_arm'):
            continue
        lengths=[dict(zip(j['roads'],j['lengths']))[base+i] for i in range(1,5)]
        tc=dict(m['config'],type='t',missing_arm=j['missing_arm'])
        tc.pop('arm_turns',None)
        t_turns={ARM_NAMES[i]:v for i,v in arm_turns(tc).items()}
        tc['arm_turns']=t_turns
        tile,moves=make_tile(tc,lengths)
        for road in list(root.findall('road')):
            if road.get('junction')==str(j['id']):root.remove(road)
        target=root.find("junction[@id='%s']"%j['id'])
        for con in list(target.findall('connection')):target.remove(con)
        for road in tile.findall('road'):
            if road.get('junction')=='-1':continue
            road.set('id',str(base+int(road.get('id'))));road.set('junction',str(j['id']))
            for g in road.findall('./planView/geometry'):
                g.set('x',str(float(g.get('x'))+j['center'][0]-lengths[0]-m['config']['junction_radius_m']))
                g.set('y',str(float(g.get('y'))+j['center'][1]))
            for link in road.findall('./link/*'):link.set('elementId',str(base+int(link.get('elementId'))))
            root.append(road)
        for con in tile.find('junction').findall('connection'):
            for k in ('incomingRoad','connectingRoad'):con.set(k,str(base+int(con.get(k))))
            target.insert(len(target.findall('connection')),con)
        m['movements']=[v for v in m['movements'] if v['junction_id']!=j['id']]
        for move in moves:
            move.update(road=base+move['road'],target_road=base+move['target_road'],junction_id=j['id'])
            m['movements'].append(move)
        j['arm_turns']=t_turns
    # Rebuild only T-shaped pavement/curb fill at its actual centre, with 3 lanes.
    for j in m['junctions']:
        base = j['id']-10
        if not j.get('missing_arm'):
            continue
        for road in list(root.findall('road')):
            if base+900 <= int(road.get('id')) <= base+914:
                root.remove(road)
        local = ET.Element('OpenDRIVE')
        junction_surface(local, dict(m['config'],type='t',missing_arm=j['missing_arm']), 0)
        for road in local.findall('road'):
            road.set('id',str(base+int(road.get('id'))))
            for g in road.findall('./planView/geometry'):
                g.set('x',str(float(g.get('x'))+j['center'][0]))
                g.set('y',str(float(g.get('y'))+j['center'][1]))
            root.append(road)
    live_signals = {s.get('id') for s in root.findall('./road/signals/signal')}
    for ctrl in list(root.findall('controller')):
        for control in list(ctrl.findall('control')):
            if control.get('signalId') not in live_signals:
                ctrl.remove(control)
        if not ctrl.findall('control'):
            root.remove(ctrl)
    controllers = {x.get('id') for x in root.findall('controller')}
    for j in root.findall('junction'):
        for ref in list(j.findall('controller')):
            if ref.get('id') not in controllers:
                j.remove(ref)
    for j in m['junctions']:
        pairs = [(r,l) for r,l in zip(j['roads'],j['lengths']) if r not in removed]
        j['roads'] = [r for r,l in pairs]
        j['lengths'] = [l for r,l in pairs]
        j['arms'] = {k:v for k,v in j['arms'].items() if v not in removed}
        j['missing_arm'] = j.get('missing_arm')
        j['type'] = 't' if j['missing_arm'] else 'cross'
    m['signals'] = {sid:v for sid,v in m['signals'].items() if sid in live_signals}
    m['movements'] = [v for v in m['movements'] if v['road'] not in removed and v['target_road'] not in removed]
    m['seams'] = [s for s in m['seams'] if s['road_a'] not in removed and s['road_b'] not in removed]
    stitch_manifest(root,m)
    m['config'].update(type='random_grid',junction_types=types,missing_arms=missing,t_probability=c['t_probability'])
    m['generation'] = 'full_cross_grid_then_prune_planned_arms'
    m['removal_sampling'] = dict(attempts=attempts,proposal='uniform four directions',acceptance='connected junction graph',type_probability=c['t_probability'])
    m['removed_road_ids'] = sorted(removed)
    m['status'] = 'generated_requires_validation'
    root[:] = [root.find('header')]+root.findall('road')+root.findall('controller')+root.findall('junction')
    out.mkdir(parents=True, exist_ok=True)
    ET.indent(root)
    from signal_placement import apply as place_signals
    place_signals(root,m)
    ET.ElementTree(root).write(out/'map.xodr',encoding='utf-8',xml_declaration=True)
    m['opendrive_sha256'] = hashlib.sha256((out/'map.xodr').read_bytes()).hexdigest()
    (out/'manifest.json').write_text(json.dumps(m,indent=2),encoding='utf-8')
    (out/'config.json').write_text(json.dumps(m['config'],indent=2),encoding='utf-8')
    write_default_rules(out,m)
    return m

if __name__ == '__main__':
    p=argparse.ArgumentParser(); p.add_argument('--config',required=True);p.add_argument('--out',required=True)
    a=p.parse_args(); build(json.loads(Path(a.config).read_text(encoding='utf-8-sig')),a.out)
