"""Read precomputed, map-bound CARLA routes without the original workspace."""
from .adapter import load_route, carla_locations, spawn_transform
from .errors import AdapterError


def adapt(map_dir):
    """Extract map topology, boundary OD routes, signals and stop lines offline."""
    from .topology_adapter import adapt as implementation
    return implementation(map_dir)


def build_od_catalog(map_dir):
    """Return audited boundary OD routes; never writes or connects to a server."""
    result = adapt(map_dir)
    if not result['passed']:
        reasons = [d['code'] for d in result['diagnostics'] if d['severity']=='error']
        if not all(r['passed'] for r in result['topology_checks']):
            reasons.append('route_topology_failed')
        raise AdapterError('invalid_od_catalog', ', '.join(reasons) or 'empty_boundary_catalog')
    return result


def audit_route(map_dir, route, expected_exit=None):
    """Audit road/lane/s plus CARLA-world XYZ samples against the frozen map."""
    import carla
    from .route_topology import LaneGraph
    from .adapter import read_map
    raw, _, _ = read_map(map_dir)
    return LaneGraph(carla.Map('offline_audit', raw.decode('utf-8-sig'))).audit(route, expected_exit)


def lane_s_direction(*args, **kwargs):
    from .topology_adapter import lane_s_direction as implementation
    return implementation(*args, **kwargs)


def bypass_route(*args, **kwargs):
    from .topology_adapter import bypass_route as implementation
    return implementation(*args, **kwargs)


def transition_route(*args, **kwargs):
    from .topology_adapter import transition_route as implementation
    return implementation(*args, **kwargs)
