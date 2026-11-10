"""Portable offline route preprocessing and CARLA route consumption."""
import argparse
import os
import hashlib
import json
import math
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT_DATASET = Path(os.environ.get('ROUTE_ADAPTER_DATASET', str(HERE.parent / 'maps')))


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def read_map(folder):
    folder = Path(folder)
    raw = (folder / 'map.xodr').read_bytes()
    design = read_json(folder / 'manifest.json')
    digest = hashlib.sha256(raw).hexdigest()
    if digest != design.get('opendrive_sha256'):
        raise ValueError('Map/manifest fingerprint mismatch: ' + folder.name)
    return raw, design, digest


def planner_hash():
    digest = hashlib.sha256()
    for name in ('adapter.py', 'continuous_route_planner.py', 'network_itinerary.py',
                 'heterogeneous_route.py', 'roundabout_route.py'):
        digest.update(name.encode('ascii'))
        digest.update((HERE / name).read_bytes().replace(b'\r\n', b'\n'))
    return digest.hexdigest()


def validate_route(route, design, digest):
    if route.get('schema_version') != 'continuous_route.4' or route.get('opendrive_sha256') != digest:
        raise ValueError('Unsupported route or map/route fingerprint mismatch')
    if route.get('continuous') is not True:
        raise ValueError('Route has not passed continuity validation')
    points = route.get('points', [])
    if len(points) < 2 or any(len(p) != 3 or any(not math.isfinite(v) for v in p) for p in points):
        raise ValueError('Invalid route samples')
    gaps = [math.sqrt(sum((x-y)**2 for x,y in zip(a,b))) for a,b in zip(points,points[1:])]
    if max(gaps) > 1.25:
        raise ValueError('Discontinuous route samples')
    required = {jid for j in design['junctions'] for jid in j.get('opendrive_junction_ids', [j['id']])}
    if not required <= set(route['junction_order']):
        raise ValueError('Route does not cover every physical junction')
    return dict(samples=len(points), length_m=sum(gaps), max_sample_gap_m=max(gaps),
                required_junctions=len(required), covered_junctions=len(set(route['junction_order'])))


def build(folder):
    import carla
    try:
        from . import continuous_route_planner, heterogeneous_route, roundabout_route
    except ImportError:
        import continuous_route_planner, heterogeneous_route, roundabout_route
    raw, design, digest = read_map(folder)
    kind = design['config'].get('type')
    planner = {'heterogeneous': heterogeneous_route, 'roundabout': roundabout_route}.get(kind, continuous_route_planner)
    map_obj = carla.Map('offline_route', raw.decode('utf-8-sig'))
    points, first, report = planner.plan(map_obj, design)
    report.update(schema_version='continuous_route.4', opendrive_sha256=digest,
                  planner_sha256=planner_hash(), planner=planner.__name__.split('.')[-1],
                  carla_version='0.9.15', coordinates='CARLA world meters (not OpenDRIVE XY)',
                  points=[[p.x,p.y,p.z] for p in points], continuous=True,
                  physical_vehicle_validated=False,
                  scope='One all-junction continuous route, not all lanes or an arbitrary OD router')
    report.update(validate_route(report, design, digest))
    path = Path(folder) / 'generated_routes.json'
    path.write_text(json.dumps(report, indent=2), encoding='utf-8')
    return report


def load_route(folder, world=None):
    """Read samples without replanning; optionally require the loaded world to match."""
    _, design, digest = read_map(folder)
    report = read_json(Path(folder) / 'generated_routes.json')
    validate_route(report, design, digest)
    if world is not None:
        def node(element):
            return (element.tag, sorted(element.attrib.items()), (element.text or '').strip(),
                    [node(child) for child in element])
        actual = ET.fromstring(world.get_map().to_opendrive())
        expected = ET.fromstring((Path(folder) / 'map.xodr').read_bytes())
        if node(actual) != node(expected):
            raise ValueError('Loaded CARLA world differs from route map')
    return report


def carla_locations(folder, world=None):
    """For a trajectory follower; these are not Traffic Manager RoadOption commands."""
    import carla
    return [carla.Location(x=p[0], y=p[1], z=p[2]) for p in load_route(folder, world)['points']]


def spawn_transform(folder, world, height=0.3):
    """Locate the recorded start lane in the matched live world."""
    route = load_route(folder, world)
    start = route['start']
    waypoint = world.get_map().get_waypoint_xodr(start['road'], start['lane'], start['s'])
    if waypoint is None:
        raise ValueError('Start waypoint is missing')
    transform = waypoint.transform
    transform.location.z += height
    return transform


def load_world(folder, host, port):
    import carla
    raw, _, _ = read_map(folder)
    load_route(folder)
    client = carla.Client(host, port)
    client.set_timeout(120)
    world = client.get_world()
    if world.get_settings().synchronous_mode:
        raise RuntimeError('Finish the current synchronous experiment before loading')
    world.wait_for_tick(15)
    if any(world.get_actors().filter(pattern) for pattern in ('vehicle.*','sensor.*','walker.*')):
        raise RuntimeError('Finish the current actors/experiment before loading')
    # Run in a separate interpreter to isolate the toolkit's legacy module names.
    viewer = HERE.parent / 'JunctionToolkit' / 'view.py'
    if not viewer.is_file():
        viewer = HERE.parents[1] / 'Toolkits' / 'JunctionToolkit' / 'view.py'
    if not viewer.is_file():
        raise FileNotFoundError('Bundled JunctionToolkit/view.py is missing')
    subprocess.run([sys.executable, '-B', str(viewer),
                    str(Path(folder).resolve()), '--host', host, '--port', str(port),
                    '--beautify', 'auto', '--no-photos'], check=True)
    world = client.get_world()
    world.wait_for_tick(15)
    # Prevent uncontrolled native phase cycling until a controller takes ownership.
    for light in world.get_actors().filter('traffic.traffic_light'):
        light.set_green_time(1e9)
        light.set_yellow_time(1e9)
        light.set_red_time(1e9)
        light.set_state(carla.TrafficLightState.Red)
    client.apply_batch_sync([], False)
    load_route(folder, world)
    print('Loaded matched map with automatic beautification; native signals held red.')


def batch(dataset, rebuild):
    rows = []
    folders = sorted(p for p in Path(dataset).iterdir() if p.is_dir() and (p/'map.xodr').is_file())
    if not folders:
        raise ValueError('No map packages found')
    for folder in folders:
        try:
            report = build(folder) if rebuild else load_route(folder)
            rows.append(dict(map=folder.name, passed=True, opendrive_sha256=report['opendrive_sha256'],
                             route_sha256=hashlib.sha256((folder/'generated_routes.json').read_bytes()).hexdigest(),
                             planner=report['planner'], samples=report['samples'], length_m=report['length_m'],
                             max_sample_gap_m=report['max_sample_gap_m'], required_junctions=report['required_junctions']))
        except Exception as exc:
            rows.append(dict(map=folder.name, passed=False, error=str(exc)))
        print(json.dumps(rows[-1]), flush=True)
    result = dict(schema_version='map_route_release.1', passed=all(r['passed'] for r in rows), maps=rows,
                  planner_sha256=planner_hash(), physical_vehicle_validated=False,
                  scope='Offline CARLA topology/continuity only; stop-line alignment and traffic safety not certified')
    if rebuild:
        (Path(dataset)/'route_release.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    if not result['passed']:
        raise SystemExit(1)
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('command', choices=['build','batch','verify','load','od'])
    p.add_argument('--map-dir', type=Path)
    p.add_argument('--dataset', type=Path, default=DEFAULT_DATASET)
    p.add_argument('--output', type=Path, help='JSON output for the offline od command')
    p.add_argument('--host', default='localhost')
    p.add_argument('--port', type=int, default=2000)
    a=p.parse_args()
    if a.command in ('build','load','od') and a.map_dir is None:
        p.error('--map-dir is required')
    if a.command=='od':
        if __package__:
            from . import build_od_catalog
        else:
            sys.path.insert(0, str(HERE.parent))
            from route_adapter import build_od_catalog
        result=build_od_catalog(a.map_dir)
        if a.output:
            a.output.parent.mkdir(parents=True, exist_ok=True)
            a.output.write_text(json.dumps(result, indent=2, allow_nan=False), encoding='utf-8')
        print(json.dumps(dict(map=result['map_name'], routes=len(result['route_catalog']),
                             opendrive_sha256=result['opendrive_sha256'], server_connected=False)))
    elif a.command=='batch': batch(a.dataset, True)
    elif a.command=='verify':
        if a.map_dir: print(json.dumps({k:v for k,v in load_route(a.map_dir).items() if k!='points'}, indent=2))
        else: batch(a.dataset, False)
    elif a.command=='build':
        report=build(a.map_dir)
        print(json.dumps({k:v for k,v in report.items() if k not in ('points','movements')}, indent=2))
    else: load_world(a.map_dir,a.host,a.port)


if __name__=='__main__': main()
