"""One physical vehicle traverses a cross/T/mixed network; spectator follows it.

Run with carla-env. All-green isolated connectivity acceptance, not a traffic
signal/controller evaluation. Uses CARLA next() for road/junction traversal,
and explicit smooth same-direction lane changes on broken approach markings.
"""
import argparse
import collections
import hashlib
import json
import math
import time
from pathlib import Path

import carla
try:
    from .network_itinerary import itinerary
except ImportError:
    from network_itinerary import itinerary


def waypoint_record(w):
    p = w.transform.location
    return dict(road=w.road_id, lane=w.lane_id, s=w.s, x=p.x, y=p.y, z=p.z)


def audit_seams(report, route, design):
    if design['config'].get('type')=='heterogeneous':
        from heterogeneous_route import audit
        return audit(report,route,design)
    if design['config'].get('type')=='roundabout':
        from roundabout_route import audit
        return audit(report,route,design)
    seams = {frozenset((s['road_a'], s['road_b'])) for s in design['seams']}
    transitions = report['road_transitions']
    observed = [[a['road'], b['road']] for a,b in zip(transitions, transitions[1:])
                if frozenset((a['road'], b['road'])) in seams]
    expected = [[a['target_road'], b['road']] for a,b in zip(route['movements'], route['movements'][1:])]
    report['observed_seams'] = observed
    report['expected_seams'] = expected
    report['seam_sequence_passed'] = observed == expected
    return observed == expected


def follow(start, road, lane, station):
    todo = collections.deque([(start, [])]); seen = set()
    while todo:
        w, history = todo.popleft()
        key = (w.road_id, w.lane_id, round(w.s, 1))
        if key in seen:
            continue
        seen.add(key); path = history + [w]
        if w.road_id == road and w.lane_id == lane and abs(w.s-station) < 1.1:
            return path
        if len(seen) > 20000:
            break
        todo.extend((n, path) for n in w.next(1.))
    raise ValueError('No waypoint continuation to %s/%s/%s' % (road, lane, station))


def plan(m, d, required=None):
    selected = itinerary(d, required)
    junctions = {j['id']:j for j in d['junctions']}
    points = []; changes = []; movements = []
    start = None
    for k, move in enumerate(selected):
        j = junctions[move['junction_id']]; src = move['road']; dst = move['target_road']
        movements.append(move)
        if start is None:
            start = m.get_waypoint_xodr(src, move['lane'], 25.)
        if start.lane_id != move['lane']:
            # Change one adjacent lane at a time, over 35m per lane.
            old = start.lane_id
            while old != move['lane']:
                new = old + (1 if move['lane'] > old else -1)
                station = start.s
                for n in range(36):
                    a = m.get_waypoint_xodr(src, old, station+n)
                    b = m.get_waypoint_xodr(src, new, station+n)
                    if not a or not b or a.is_junction or b.is_junction:
                        raise ValueError('Lane change leaves approach')
                    if not (a.lane_id*b.lane_id > 0):
                        raise ValueError('Lane change crosses opposing traffic')
                    # In CARLA, negative lane IDs: left is the larger lane ID.
                    allowed = carla.LaneChange.Left if new > old else carla.LaneChange.Right
                    if not (int(a.lane_change) & int(allowed)):
                        raise ValueError('Lane change prohibited at %s/%s/%s' % (src, old, a.s))
                    t = n/35.; blend = 10*t**3-15*t**4+6*t**5
                    pa, pb = a.transform.location, b.transform.location
                    points.append(carla.Location(x=pa.x+(pb.x-pa.x)*blend,
                                                 y=pa.y+(pb.y-pa.y)*blend, z=pa.z))
                changes.append(dict(road=src, from_lane=old, to_lane=new, start_s=station, end_s=station+35.))
                start = b; old = new
        if k+1 < len(selected):
            target_road = selected[k+1]['road']
            target_lane = -move['target_lane']; target_s = 25.
        else:
            target_road = dst; target_lane = move['target_lane']; target_s = 65.
        path = follow(start, target_road, target_lane, target_s)
        crossed = {w.junction_id for w in path if w.is_junction}
        if crossed != {j['id']}:
            raise ValueError('Unexpected junctions in route segment: %s' % crossed)
        points.extend(w.transform.location for w in path)
        start = path[-1]
    first = m.get_waypoint_xodr(movements[0]['road'], movements[0]['lane'], 25.)
    return points, first, dict(junction_order=[v['junction_id'] for v in selected],
                              start=waypoint_record(first), destination=waypoint_record(start),
                              movements=movements, lane_changes=changes,
                              length_m=sum(a.distance(b) for a,b in zip(points,points[1:])))
