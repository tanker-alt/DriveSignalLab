"""Validate generator references before passing OpenDRIVE to native CARLA."""
import math
import xml.etree.ElementTree as ET
from .errors import AdapterError


def validate_opendrive(raw):
    try:
        root = ET.fromstring(raw)
    except ET.ParseError as exc:
        raise AdapterError('invalid_opendrive_xml', str(exc))
    if root.tag != 'OpenDRIVE':
        raise AdapterError('invalid_opendrive_xml', 'Expected OpenDRIVE root')
    roads = {r.get('id'): r for r in root.findall('road')}
    junctions = {j.get('id'): j for j in root.findall('junction')}
    for road in roads.values():
        length = float(road.get('length', 'nan'))
        if not math.isfinite(length) or length <= .00002:
            raise AdapterError('invalid_road_length', road.get('id'))
        for link in road.findall('link/*'):
            table = roads if link.get('elementType') == 'road' else junctions
            if link.get('elementId') not in table:
                raise AdapterError('unresolved_road_link', '{} -> {}'.format(road.get('id'), link.attrib))
    for junction in junctions.values():
        for connection in junction.findall('connection'):
            incoming = roads.get(connection.get('incomingRoad'))
            connecting = roads.get(connection.get('connectingRoad'))
            if incoming is None or connecting is None:
                raise AdapterError('unresolved_connector', str(connection.attrib))
            incoming_ids = {lane.get('id') for lane in incoming.findall('lanes/laneSection/*/lane')}
            sections = connecting.findall('lanes/laneSection')
            if not sections or connection.get('contactPoint') not in ('start', 'end'):
                raise AdapterError('unresolved_connector', str(connection.attrib))
            target = sections[0] if connection.get('contactPoint') == 'start' else sections[-1]
            target_ids = {lane.get('id') for lane in target.findall('*/lane')}
            for link in connection.findall('laneLink'):
                if link.get('from') not in incoming_ids or link.get('to') not in target_ids:
                    raise AdapterError('unresolved_connector_lane', str(link.attrib))
    return root


def mapping_diagnostics(road_map, root, signals, stops, safety, digest):
    diagnostics = []
    def issue(code, detail, severity='error'):
        diagnostics.append(dict(code=code, detail=detail, severity=severity))
    if safety and safety.get('opendrive_sha256') != digest:
        issue('stop_line_map_hash_mismatch', 'safety_rules.json must be regenerated for this map')
    xml_signals = {s.get('id'): r.get('id') for r in root.findall('road') for s in r.findall('signals/signal')}
    signal_keys = set()
    for row in signals:
        sid, road, lane, s = (row[k] for k in ('signal_id', 'road_id', 'lane_id', 'reference_s'))
        signal_keys.add((sid, road, lane))
        if xml_signals.get(sid) != str(road):
            issue('signal_mapping_unresolved', 'Signal {} does not belong to road {}'.format(sid, road))
        if not _resolves(road_map, road, lane, s):
            issue('signal_lane_unresolved', str(row))
    for stop in stops:
        road, lane = stop.get('road_id'), stop.get('lane_id')
        if (str(stop.get('signal_opendrive_id')), road, lane) not in signal_keys:
            issue('stop_signal_mapping_unresolved', str(stop.get('id')))
        if not _resolves(road_map, road, lane, stop.get('s_m')):
            issue('stop_lane_unresolved', str(stop.get('id')))
    stop_keys = {(str(s.get('signal_opendrive_id')), s.get('road_id'), s.get('lane_id')) for s in stops}
    for missing in sorted(signal_keys - stop_keys):
        issue('signal_stop_missing', str(missing))
    if not signals:
        issue('signal_mapping_unavailable', 'No generator signal mapping; topology only', 'warning')
    issue('special_right_of_way_unsupported', 'Priority, authorized opposing travel and signal timing need scenario-level rules', 'warning')
    return diagnostics


def _resolves(road_map, road, lane, s):
    if not isinstance(road, int) or not isinstance(lane, int) or not isinstance(s, (int, float)) or not math.isfinite(s):
        return False
    waypoint = road_map.get_waypoint_xodr(road, lane, s)
    return waypoint is not None and str(waypoint.lane_type) == 'Driving'
