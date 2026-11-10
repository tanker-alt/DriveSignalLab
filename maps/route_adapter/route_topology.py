"""Offline lane graph and route audit. Never creates a CARLA Client."""
import math
import xml.etree.ElementTree as ET
from collections import deque
import carla
from .errors import AdapterError


def key(w):
    return (w.road_id, w.section_id, w.lane_id)


def point(w):
    t = w.transform
    return dict(x=t.location.x, y=t.location.y, z=t.location.z,
                yaw=t.rotation.yaw, road_id=w.road_id, lane_id=w.lane_id,
                section_id=w.section_id, s=w.s)


class LaneGraph:
    def __init__(self, road_map):
        self.map = road_map
        self.segments = {}
        bounds={}
        lanes=[]
        for road in ET.fromstring(road_map.to_opendrive()).findall('road'):
            sections=road.findall('lanes/laneSection')
            for i,section in enumerate(sections):
                bounds[(int(road.attrib['id']),i)]=(float(section.attrib['s']),
                    float(sections[i+1].attrib['s']) if i+1<len(sections) else float(road.attrib['length']))
                lanes.extend((int(road.attrib['id']),i,int(lane.get('id')))
                             for lane in section.findall('*/lane') if lane.get('type')=='driving')
        # get_topology() can omit terminal segments behind very short roads.
        # Enumerate declared driving sections, then resolve both ends in CARLA.
        for lane_key in lanes:
            road,section,lane=lane_key
            low,high=bounds[(road,section)]
            if high-low <= .0002:
                raise AdapterError('lane_section_too_short', str(lane_key))
            probe=road_map.get_waypoint_xodr(road,lane,low+(high-low)*.4)
            following=road_map.get_waypoint_xodr(road,lane,low+(high-low)*.6)
            if not probe or not following:
                raise AdapterError('lane_direction_unresolved', str(lane_key))
            delta=following.transform.location-probe.transform.location
            forward=probe.transform.get_forward_vector()
            dot=delta.x*forward.x+delta.y*forward.y
            if abs(dot) < 1e-8:
                raise AdapterError('lane_direction_unresolved', str(lane_key))
            positive=dot>0
            start_s=low+.00001 if positive else high-.00001
            end_s=high-.00001 if positive else low+.00001
            start=road_map.get_waypoint_xodr(road,lane,start_s)
            end=road_map.get_waypoint_xodr(road,lane,end_s)
            if start is None or end is None or key(start)!=lane_key or key(end)!=lane_key:
                raise AdapterError('lane_end_unresolved', str(lane_key))
            self.segments[lane_key]=(start,end)
        # A 10 cm probe can skip an entire short connector. Cross only the
        # endpoint epsilon, then validate every edge before traversing anything.
        self.edges = {k: {key(w) for w in b.next(.0001)} for k, (a, b) in self.segments.items()}
        for source, targets in self.edges.items():
            for target in targets:
                if target not in self.segments:
                    raise AdapterError('unresolved_successor', '{} -> {}'.format(source, target))
        self.entries = sorted(k for k, (a, b) in self.segments.items() if not a.previous(.0001))
        self.exits = sorted(k for k in self.segments if not self.edges[k])

    def catalog(self):
        if not self.entries or not self.exits:
            raise AdapterError('no_boundary_od', 'Closed or disconnected network needs explicit endpoints')
        result = []
        for origin in self.entries:
            pending = deque([origin]); parents = {origin: None}
            reached_exit = False
            while pending:
                current = pending.popleft()
                if current in self.exits:
                    reached_exit = True
                    path = []; cursor = current
                    while cursor is not None:
                        path.append(cursor); cursor = parents[cursor]
                    path.reverse()
                    route = []
                    for k in path:
                        a, b = self.segments[k]
                        sign = 1 if b.s > a.s else -1
                        start = a.s + (15 * sign if k == origin else .00001*sign)
                        end = b.s - (15 * sign if k == current else .00001*sign)
                        if sign * (end-start) <= 0:
                            raise AdapterError('boundary_road_too_short', '15 m clearance at '+str(k))
                        count = int(abs(end-start))
                        samples = [start + sign*i for i in range(count+1)]
                        if abs(samples[-1]-end) > .001:
                            samples.append(end)
                        for s in samples:
                            waypoint = self.map.get_waypoint_xodr(k[0], k[2], s)
                            if waypoint is None or key(waypoint) != k:
                                raise AdapterError('lane_sample_unresolved', '{} at s={}'.format(k, s))
                            route.append(point(waypoint))
                    result.append(dict(route_id='od_{}_{}_{}_{}'.format(origin[0], origin[2], current[0], current[2]),
                                       origin=origin[0], lane=origin[2], destination=current[0],
                                       destination_lane=current[2], route=route))
                    continue
                for nxt in sorted(self.edges[current]):
                    if nxt not in self.segments:
                        raise ValueError('Unresolved successor '+str(nxt))
                    if nxt not in parents:
                        parents[nxt] = current
                        pending.append(nxt)
            if not reached_exit:
                raise AdapterError('unreachable_boundary_exit', str(origin))
        return result

    def audit(self, route, expected_exit=None):
        errors = []; warnings = []; resolved = []
        def fail(index, reason):
            errors.append(dict(index=index, reason=reason))
        if len(route) < 2:
            return dict(passed=False, errors=[dict(index=0, reason='empty_or_short_route')])
        for i, p in enumerate(route):
            if not isinstance(p, dict) or any(not isinstance(p.get(k), int) for k in ('road_id','lane_id')) or ('section_id' in p and not isinstance(p['section_id'], int)):
                fail(i, 'invalid_lane_reference'); resolved.append(None); continue
            if any(not isinstance(p.get(k), (int,float)) or not math.isfinite(p[k]) for k in ('x','y','z','s')):
                fail(i, 'invalid_coordinates'); resolved.append(None); continue
            declared = self.map.get_waypoint_xodr(p['road_id'], p['lane_id'], p['s'])
            if declared is None:
                # CARLA next() can emit s==road.length while get_waypoint_xodr
                # excludes that endpoint. Resolve only a 10 micrometre boundary.
                for offset in (-.00001,.00001):
                    declared=self.map.get_waypoint_xodr(p['road_id'],p['lane_id'],p['s']+offset)
                    if declared is not None: break
            w = self.map.get_waypoint(carla.Location(p['x'],p['y'],p['z']), project_to_road=True,
                                      lane_type=carla.LaneType.Driving)
            if not declared or not w:
                fail(i, 'missing_lane'); resolved.append(None); continue
            center = declared.transform.location
            if math.hypot(center.x-p['x'],center.y-p['y']) <= declared.lane_width/2 + .01:
                w = declared
            # Lane metadata can intentionally lag during interpolated lane changes.
            # Validate the physical center path, and report label differences separately.
            loc = w.transform.location
            if math.hypot(loc.x-p['x'],loc.y-p['y']) > w.lane_width/2 + .15 or abs(loc.z-p['z']) > .5:
                fail(i, 'outside_driving_surface')
            if w.road_id != p['road_id'] or w.lane_id != p['lane_id']:
                warnings.append(dict(index=i, reason='interpolated_geometry_differs_from_lane_label'))
            resolved.append(w)
        for i, (a,b) in enumerate(zip(resolved,resolved[1:])):
            if not a or not b:
                continue
            p,q = route[i],route[i+1]; dx=q['x']-p['x'];dy=q['y']-p['y']
            distance=math.hypot(dx,dy)
            if distance > 3.0:
                fail(i, 'sample_gap_over_3m')
            if distance < .00001 and key(a)==key(b):
                continue
            f=a.transform.get_forward_vector()
            if dx*f.x+dy*f.y < -.05:
                fail(i, 'backwards_motion')
            if key(a)==key(b):
                if key(a) not in self.segments:
                    fail(i, 'unresolved_lane_segment'); continue
                first,last=self.segments[key(a)]
                if (b.s-a.s)*(1 if last.s>first.s else -1) < -.05:
                    fail(i, 'reversed_s')
            elif key(b) in self.edges.get(key(a),set()):
                # An edge alone does not authorize jumping over the whole road.
                end=self.segments[key(a)][1];start=self.segments[key(b)][0]
                if abs(end.s-a.s)+abs(b.s-start.s)>distance+1.0:
                    fail(i,'connector_jump')
            elif a.road_id==b.road_id and a.section_id==b.section_id:
                left=a.get_left_lane();right=a.get_right_lane()
                side='Left' if left and key(left)==key(b) else ('Right' if right and key(right)==key(b) else None)
                g=b.transform.get_forward_vector()
                if not side or f.x*g.x+f.y*g.y < .9:
                    fail(i,'nonadjacent_or_opposing_lane_change')
                elif a.is_junction or b.is_junction or str(a.lane_change) not in (side,'Both'):
                    fail(i,'prohibited_lane_change')
            else:
                fail(i,'disconnected_lane_transition')
        for i in (0,len(route)-1):
            w=resolved[i];p=route[i]
            if w and (w.road_id,w.lane_id)!=(p['road_id'],p['lane_id']):
                fail(i,'endpoint_lane_mismatch')
        if expected_exit and resolved[-1]:
            w=resolved[-1]
            if (w.road_id,w.lane_id)!=tuple(expected_exit):
                fail(len(route)-1,'wrong_destination')
        return dict(passed=not errors, point_count=len(route), errors=errors,
                    geometry_label_warning_count=len(warnings), warnings=warnings[:5],
                    start=point(resolved[0]) if resolved[0] else None,
                    end=point(resolved[-1]) if resolved[-1] else None)
