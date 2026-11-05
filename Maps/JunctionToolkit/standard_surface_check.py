"""Read-only ray checks for native pavement infill and raised standard curbs."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import carla


def run(folder):
    folder = Path(folder)
    design = json.loads((folder/'manifest.json').read_text())
    if design['config'].get('standard_profile') != 'urban_cross3_v1':
        raise ValueError('Standard map required')
    client = carla.Client('localhost', 2000)
    client.set_timeout(10)
    world = client.get_world()
    if hashlib.sha256(world.get_map().to_opendrive().encode()).hexdigest() != design['opendrive_sha256']:
        raise ValueError('Loaded map differs from requested map')
    cfg = design['config']
    extent, radius = cfg['junction_radius_m'], cfg['corner_radius_m']
    records = []
    for junction in design['junctions']:
        cx, cy = junction['center']
        probes = [('pavement', 0, 0, -.02)]
        for i in range(4):
            angle = i*math.pi/2
            for kind, x, y, z in [('curb', extent-(radius-.1)/math.sqrt(2), extent-(radius-.1)/math.sqrt(2), .15),
                                   ('straight_curb', extent+10, 10.5+cfg['shoulder_width_m']+.1, .15),
                                   ('infill', 1.75, extent-2, -.02)]:
                probes.append((kind, x*math.cos(angle)-y*math.sin(angle), x*math.sin(angle)+y*math.cos(angle), z))
        for kind, x, y, expected in probes:
            hits = world.cast_ray(carla.Location(cx+x, -(cy+y), 2), carla.Location(cx+x, -(cy+y), -.5))
            actual = max([h.location.z for h in hits], default=None)
            # Connector overlaps are at z=0; CARLA sidewalk offset is 0.1524m.
            passed = actual is not None and abs(actual-expected) < .035
            records.append(dict(junction=junction['id'], kind=kind, expected_z=expected, actual_z=actual, passed=passed))
    report = dict(passed=all(r['passed'] for r in records), samples=records,
                  limits='Sampled collision surface heights, not exhaustive mesh or material certification')
    (folder/'surface_validation.json').write_text(json.dumps(report, indent=2))
    print('Surface samples:', len(records), 'passed:', report['passed'])
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map_dir')
    args = parser.parse_args()
    if not run(args.map_dir)['passed']:
        raise SystemExit(1)
