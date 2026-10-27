"""Map-owned stop-line geometry; no violation thresholds or scoring policy.

OpenDRIVE road/lane/s is authoritative. CARLA resolves the endpoints, so the
renderer and future evaluators can share resolve_stop_lines(). Python 3.7+.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def default_rules(design):
    lines = []
    for sid, signal in sorted(design['signals'].items()):
        for lane in signal['incoming_lanes']:
            lines.append(dict(id='stop_{}_{}'.format(signal['road_id'], lane),
                              junction_id=signal['junction_id'], signal_opendrive_id=sid,
                              road_id=signal['road_id'], lane_id=lane,
                              s_m=signal['reference_s']))
    return dict(schema_version='safety_rules.1',
                opendrive_sha256=design['opendrive_sha256'],
                coordinate_system='OpenDRIVE road_id/lane_id/s_m; resolved world coordinates use CARLA XY',
                geometry_status='designed_stop_line_not_tm_calibrated' if design['config'].get('standard_profile') else 'provisional_signal_reference',
                position_source='Map-designed stop line, 1m upstream of crosswalk paint' if design['config'].get('standard_profile') else 'Initial positions copied from manifest reference_s; not a calibrated traffic-manager stopping boundary',
                line_convention='Transverse lane-width segment centered at s_m; paint is centered on this segment',
                # CARLA debug material is emissive: low neutral RGB avoids bloom.
                visualization=dict(color_rgb=[2, 2, 2], thickness_m=0.30,
                                   height_above_road_m=0.12),
                stop_lines=lines,
                assessment=dict(status='pending_user_definition', red_light_violation=None,
                                speeding=None, out_of_bounds=None, wrong_way=None))


def validate_rules(rules, design, raw):
    digest = hashlib.sha256(raw).hexdigest()
    if digest != design['opendrive_sha256'] or digest != rules.get('opendrive_sha256'):
        raise ValueError('Safety rules/map fingerprint mismatch')
    if rules.get('schema_version') != 'safety_rules.1':
        raise ValueError('Unsupported safety rules schema')
    xml = ET.fromstring(raw)
    roads = {int(r.get('id')): r for r in xml.findall('road')}
    expected = {(sid, cfg['road_id'], lane) for sid, cfg in design['signals'].items()
                for lane in cfg['incoming_lanes']}
    seen, ids = set(), set()
    for line in rules['stop_lines']:
        key = (line['signal_opendrive_id'], line['road_id'], line['lane_id'])
        if key not in expected or key in seen or line['id'] in ids:
            raise ValueError('Duplicate or invalid stop-line signal/lane association')
        cfg = design['signals'][key[0]]
        if line['junction_id'] != cfg['junction_id']:
            raise ValueError('Stop-line junction mismatch')
        road = roads[key[1]]
        station = line['s_m']
        if isinstance(station, bool) or not isinstance(station, (float, int)) or not math.isfinite(station) or not 0 < station < float(road.get('length')):
            raise ValueError('Stop-line s_m must lie inside its road')
        sections = [s for s in road.findall('./lanes/laneSection') if float(s.get('s')) <= station]
        section = max(sections, key=lambda s: float(s.get('s')))
        lane = section.find("./*/lane[@id='{}']".format(key[2]))
        if lane is None or lane.get('type') != 'driving':
            raise ValueError('Stop line must belong to a driving lane')
        native = road.find("./signals/signal[@id='{}']".format(key[0]))
        if native is None or not any(int(v.get('fromLane')) <= key[2] <= int(v.get('toLane')) for v in native.findall('validity')):
            raise ValueError('Stop-line signal not valid for OpenDRIVE lane')
        seen.add(key)
        ids.add(line['id'])
    if seen != expected:
        raise ValueError('Stop lines must cover all controlled incoming lanes')
    visual = rules['visualization']
    for key in ('thickness_m', 'height_above_road_m'):
        value = visual[key]
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or not 0 < value <= 1:
            raise ValueError('Invalid visualization ' + key)
    color = visual['color_rgb']
    if len(color) != 3 or any(type(v) is not int or not 0 <= v <= 255 for v in color):
        raise ValueError('Invalid visualization color_rgb')
    return rules


def load_rules(folder, required=False):
    folder = Path(folder)
    path = folder / 'safety_rules.json'
    if not path.exists() and not required:
        return None
    rules = json.loads(path.read_text(encoding='utf-8'))
    design = json.loads((folder / 'manifest.json').read_text(encoding='utf-8'))
    return validate_rules(rules, design, (folder / 'map.xodr').read_bytes())


def write_default_rules(folder, design=None):
    folder = Path(folder)
    if design is None:
        design = json.loads((folder / 'manifest.json').read_text(encoding='utf-8'))
    rules = default_rules(design)
    validate_rules(rules, design, (folder / 'map.xodr').read_bytes())
    # Existing manually edited boundaries must never be silently replaced.
    with (folder / 'safety_rules.json').open('x', encoding='utf-8') as stream:
        json.dump(rules, stream, indent=2, ensure_ascii=False)
        stream.write('\n')
    return rules


def resolve_stop_lines(road_map, rules):
    if hashlib.sha256(road_map.to_opendrive().encode('utf-8')).hexdigest() != rules['opendrive_sha256']:
        raise ValueError('Loaded map does not match safety rules')
    resolved = []
    for line in rules['stop_lines']:
        wp = road_map.get_waypoint_xodr(line['road_id'], line['lane_id'], line['s_m'])
        if wp is None or (wp.road_id, wp.lane_id) != (line['road_id'], line['lane_id']) or abs(wp.s - line['s_m']) > .01 or wp.is_junction:
            raise ValueError('Cannot resolve stop line ' + line['id'])
        transform = wp.transform
        p, r, f = transform.location, transform.get_right_vector(), transform.get_forward_vector()
        half = wp.lane_width / 2
        if not math.isfinite(half) or half <= 0:
            raise ValueError('Invalid stop-line lane width')
        record = dict(line)
        record.update(center_world_m=[p.x, p.y, p.z],
                      start_world_m=[p.x-r.x*half, p.y-r.y*half, p.z-r.z*half],
                      end_world_m=[p.x+r.x*half, p.y+r.y*half, p.z+r.z*half],
                      forward_world=[f.x, f.y, f.z], lane_width_m=wp.lane_width)
        resolved.append(record)
    return resolved


def draw_stop_lines(world, rules):
    import carla
    lines = resolve_stop_lines(world.get_map(), rules)
    # Resolve/validate every line before changing the scene.
    visual = rules['visualization']
    for line in lines:
        points = [carla.Location(x=p[0], y=p[1], z=p[2]+visual['height_above_road_m'])
                  for p in (line['start_world_m'], line['end_world_m'])]
        world.debug.draw_line(points[0], points[1], thickness=visual['thickness_m'],
                              color=carla.Color(*visual['color_rgb']), life_time=0,
                              persistent_lines=True)
    return dict(schema_version='stop_line_render.1', opendrive_sha256=rules['opendrive_sha256'],
                geometry_status=rules['geometry_status'], stop_lines=lines,
                visualization=visual, rendering='Persistent debug overlay; restored by toolkit map loader; no collision geometry')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map_dir')
    parser.add_argument('--create', action='store_true', help='Create defaults only if safety_rules.json does not exist')
    parser.add_argument('--draw', action='store_true', help='Draw on matching loaded map without reloading or ticking')
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=2000)
    args = parser.parse_args()
    if args.create:
        write_default_rules(args.map_dir)
    rules = load_rules(args.map_dir, required=True)
    if args.draw:
        import carla
        client = carla.Client(args.host, args.port)
        client.set_timeout(15)
        report = draw_stop_lines(client.get_world(), rules)
        (Path(args.map_dir) / 'stop_lines_render_report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print('Validated {} stop lines: {}'.format(len(rules['stop_lines']), args.map_dir))


if __name__ == '__main__':
    main()
