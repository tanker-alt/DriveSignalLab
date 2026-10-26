"""Shared Safety configuration and map snapshot (CARLA is only used by callers)."""
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

DEFAULT_CONFIG = Path(__file__).parent / 'configs/safety.json'
ROOT = Path(__file__).resolve().parents[2] / 'InformationCollector'


def load_config(source=None):
    config = json.loads(json.dumps(source)) if isinstance(source, dict) else json.loads(Path(source or DEFAULT_CONFIG).read_text(encoding='utf-8-sig'))
    for key in ('red_light','out_of_bounds','wrong_way','lane_keeping'):
        config.pop(key, None)
    if config.get('schema_version') not in ('safety_evaluation.1','custom_safety.2'):
        raise ValueError('Unsupported custom Safety configuration')
    config['schema_version'] = 'custom_safety.2'
    config['reversing'].setdefault('trigger_speed_mps', .1)
    config['reversing'].setdefault('release_speed_mps', .05)
    schema = {
        'speeding': {'enabled': bool, 'limit_kmh': float, 'tolerance_kmh': float, 'minimum_duration_s': float},
        'reversing': {'enabled': bool, 'minimum_duration_s': float, 'end_at_zero': bool, 'trigger_speed_mps': float, 'release_speed_mps': float},
        'geometry': {'sample_step_m': float, 'boundary_uncertainty_m': float}}
    if set(config) != set(schema) | {'schema_version'}:
        raise ValueError('Unknown custom Safety configuration keys')
    for group, fields in schema.items():
        if not isinstance(config[group], dict) or set(config[group]) != set(fields):
            raise ValueError('Unknown/missing Safety keys: ' + group)
        for key, expected in fields.items():
            value = config[group][key]
            if expected is float:
                valid = type(value) in (int,float) and math.isfinite(value) and value >= 0
            else:
                valid = type(value) is expected
            if not valid:
                raise ValueError('Invalid Safety value: ' + group + '.' + key)
    if config['speeding']['limit_kmh'] <= 0 or config['geometry']['sample_step_m'] <= 0:
        raise ValueError('Speed limit and geometry step must be positive')
    if config['geometry']['sample_step_m'] > 0.25:
        raise ValueError('Geometry sampling may not exceed 0.25m')
    if config['reversing']['release_speed_mps'] >= config['reversing']['trigger_speed_mps']:
        raise ValueError('Reverse release deadband must be smaller than trigger')
    return config


def map_geometry(road_map, config, folder=None):
    """Snapshot only Driving lanes, including connectors; never shoulder fill.

    Edges are sampled from CARLA's XODR lane geometry. Polygon approximation
    and near-threshold uncertainty are recorded in the reproducible snapshot.
    """
    if not hasattr(road_map, 'to_opendrive'):
        return None
    raw = road_map.to_opendrive()
    digest = hashlib.sha256(raw.encode('utf-8')).hexdigest()
    if folder is None:
        matches = []
        for path in (ROOT.parent / 'Maps/MapDataset/maps').glob('*/safety_rules.json'):
            data = json.loads(path.read_text(encoding='utf-8-sig'))
            if data.get('opendrive_sha256') == digest:
                matches.append(path.parent)
        if len(matches) != 1:
            return None
        folder = matches[0]
    # Direct `python evaluation/run_collection.py` has no project-root import path.
    import importlib.util
    spec = importlib.util.spec_from_file_location('map_owned_safety_rules', ROOT.parent / 'Toolkits/JunctionToolkit/safety_rules.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    load_rules, resolve_stop_lines = module.load_rules, module.resolve_stop_lines
    rules = load_rules(folder, required=True)
    lines = resolve_stop_lines(road_map, rules)
    from shapely.geometry import Polygon, mapping
    from shapely.ops import unary_union
    polygons = []
    lane_regions = {}
    lane_count = 0
    step = config['geometry']['sample_step_m']
    xml = ET.fromstring(raw)
    for road in xml.findall('road'):
        rid, length = int(road.get('id')), float(road.get('length'))
        sections = road.findall('./lanes/laneSection')
        for index, section in enumerate(sections):
            start = float(section.get('s'))
            end = float(sections[index+1].get('s')) if index+1 < len(sections) else length
            for lane in section.findall('./*/lane'):
                if lane.get('type') != 'driving':
                    continue
                lid = int(lane.get('id'))
                count = max(1, int(math.ceil((end-start)/step)))
                left, right = [], []
                for k in range(count+1):
                    station = min(end-1e-5, max(start+1e-5, start+(end-start)*k/count))
                    wp = road_map.get_waypoint_xodr(rid, lid, station)
                    if wp is None or wp.road_id != rid or wp.lane_id != lid:
                        raise ValueError('Cannot resolve driving lane {} / {}'.format(rid, lid))
                    p, r = wp.transform.location, wp.transform.get_right_vector()
                    half = wp.lane_width/2
                    left.append((p.x-r.x*half, p.y-r.y*half))
                    right.append((p.x+r.x*half, p.y+r.y*half))
                poly = Polygon(left+right[::-1])
                if not poly.is_valid or poly.area <= 0:
                    raise ValueError('Invalid driving lane polygon {} / {}'.format(rid, lid))
                lane_regions['{}:{}:{}'.format(rid,index,lid)] = dict(
                    polygon=mapping(poly), centerline=[[(a[0]+b[0])/2,(a[1]+b[1])/2] for a,b in zip(left,right)],
                    is_junction=road.get('junction') != '-1')
                polygons.append(poly)
                lane_count += 1
    if not polygons:
        raise ValueError('No driving lanes')
    region = unary_union(polygons)
    return dict(schema_version='safety_geometry.1', opendrive_sha256=digest,
                map_folder=str(Path(folder).resolve()), stop_lines=lines, lane_regions=lane_regions,
                stop_line_rules=rules, driving_region=mapping(region), driving_lane_count=lane_count,
                geometry_config=config['geometry'], region_policy='driving_lanes_including_junction_connectors_only',
                coordinate_system='CARLA world XY metres',
                limit='Sampled lane-edge polygons; not rendered pavement or nearest-waypoint membership')


def legal_direction_fields(wp):
    """Do not mistake CARLA's nearest overlapping connector for a unique route."""
    if wp is None:
        return dict(legal_direction_status='unavailable')
    if getattr(wp, 'is_junction', False):
        return dict(legal_direction_status='junction_route_unresolved')
    transform = getattr(wp, 'transform', None)
    if transform is None:
        return dict(legal_direction_status='unavailable')
    f = transform.get_forward_vector()
    return dict(legal_direction_status='unique_lane', legal_forward_x=f.x,
                legal_forward_y=f.y, legal_forward_z=f.z)
