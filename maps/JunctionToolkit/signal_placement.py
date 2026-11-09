"""Independent physical lamp layout; logical signal/stop associations stay fixed."""
import math

VERSION = 'far_side_20261109'
OPPOSITE = {'east': 'west', 'west': 'east', 'north': 'south', 'south': 'north'}


def apply(root, design):
    """Set far-side poses after final geometry assembly; safe to apply again.

    Existing offsets are removed from the XML pose, not from cached manifest
    coordinates, so translated heterogeneous tiles retain correct world poses.
    """
    junctions = {j['id']: j for j in design['junctions']}
    for road in root.findall('road'):
        for signal in road.findall('./signals/signal'):
            cfg = design['signals'][signal.get('id')]
            pose = signal.find('positionInertial')
            if pose is None:
                raise ValueError('Explicit physical signal pose required: '+signal.get('id'))
            if cfg.get('role') in ('entry', 'circulating'):
                offset = 18.0
                policy = 'roundabout_local_downstream_tangent'
            else:
                junction = junctions[cfg['junction_id']]
                radius = junction.get('template_config', design['config']).get('junction_radius_m')
                if radius is None:
                    geometries = road.findall('./planView/geometry')
                    if len(geometries) != 1 or geometries[0].find('line') is None:
                        raise ValueError('Cannot infer junction boundary on curved approach')
                    g = geometries[0]; heading = float(g.get('hdg'))
                    x = float(g.get('x'))+float(road.get('length'))*math.cos(heading)
                    y = float(g.get('y'))+float(road.get('length'))*math.sin(heading)
                    radius = math.hypot(x-junction['center'][0], y-junction['center'][1])
                if radius <= 0:
                    raise ValueError('Junction radius must be positive')
                reach = 2*radius
                policy = 'far_side_junction_boundary'
                arms = junction.get('arms', {})
                origin = next((arm for arm, rid in arms.items() if rid == cfg['road_id']), None)
                if len(arms) == 3 and OPPOSITE.get(origin) not in arms:
                    lanes = road.findall('./lanes/laneSection/right/lane')
                    width = sum(float(l.find('width').get('a')) for l in lanes if l.get('type') == 'driving')
                    reach = radius+width
                    policy = 't_closed_side_pavement_boundary'
                offset = float(road.get('length'))+reach-float(signal.get('s'))
            heading = float(pose.get('hdg'))
            delta = offset-float(cfg.get('physical_forward_offset_m', 0))
            if abs(delta) > 1e-10:
                pose.set('x', str(float(pose.get('x'))+delta*math.cos(heading)))
                pose.set('y', str(float(pose.get('y'))+delta*math.sin(heading)))
            cfg['physical_forward_offset_m'] = offset
            cfg['physical_pose_opendrive'] = dict(pose.attrib)
            cfg['physical_placement_policy'] = policy
    design['physical_signal_layout_version'] = VERSION
