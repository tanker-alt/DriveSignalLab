"""Export driving geometry without decoration for RoadRunner authoring."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
from runtime_config import map_output


def clean(source, out):
    source = Path(source)
    out = map_output(out)
    if out.exists() and any(out.iterdir()):
        raise ValueError('Output must be empty')
    raw = (source / 'map.xodr').read_bytes()
    root = ET.fromstring(raw)
    removed = {'roads': [], 'lanes': 0, 'roadMark': 0, 'objects': 0,
               'signals': 0, 'controller': 0}
    for road in list(root.findall('road')):
        if not road.findall("./lanes/laneSection/*/lane[@type='driving']"):
            removed['roads'].append(road.get('id'))
            root.remove(road)
            continue
        for section in road.findall('./lanes/laneSection'):
            for side in list(section):
                for lane in list(side.findall('lane')):
                    if lane.get('id') != '0' and lane.get('type') != 'driving':
                        side.remove(lane)
                        removed['lanes'] += 1
                if side.tag in ('left', 'right') and not list(side):
                    section.remove(side)
    for parent in root.iter():
        for node in list(parent):
            if node.tag in ('roadMark', 'objects', 'signals', 'controller'):
                removed[node.tag] += 1
                parent.remove(node)
            elif node.tag == 'userData':
                parent.remove(node)
    roads = {r.get('id') for r in root.findall('road')}
    for link in root.findall('./road/link/*'):
        if link.get('elementType') == 'road' and link.get('elementId') not in roads:
            raise ValueError('Removed auxiliary road is referenced by driving network')
    for c in root.findall('./junction/connection'):
        if any(c.get(k) not in roads for k in ('incomingRoad', 'connectingRoad')):
            raise ValueError('Dangling junction connection')
    out.mkdir(parents=True, exist_ok=True)
    root.find('header').set('name', out.name)
    ET.indent(root)
    target = out / 'map.xodr'
    ET.ElementTree(root).write(target, encoding='utf-8', xml_declaration=True)
    report = dict(profile='roadrunner_driving_base_v2', source_sha256=hashlib.sha256(raw).hexdigest(),
                  map_sha256=hashlib.sha256(target.read_bytes()).hexdigest(), removed=removed,
                  roads=len(roads), junctions=len(root.findall('junction')),
                  connections=len(root.findall('./junction/connection')),
                  visual_authoring='RoadRunner owns markings, intersection pavement, shoulders, sidewalks and signals')
    (out / 'manifest.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    config = source / 'config.json'
    if config.exists():
        (out / 'source_config.json').write_bytes(config.read_bytes())
    print(json.dumps(report, indent=2))
    return report


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', required=True)
    p.add_argument('--out', required=True)
    a = p.parse_args()
    clean(a.source, a.out)
