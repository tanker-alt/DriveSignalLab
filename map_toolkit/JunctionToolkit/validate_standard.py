"""Independent geometric and marking acceptance for the standard road profile."""
import math


def audit(tree, design, road_map):
    errors = []
    c = design['config']
    roads = {int(r.get('id')): r for r in tree.findall('road')}
    radius = c['corner_radius_m']
    extent = c['junction_radius_m']
    half = 10.5+c['shoulder_width_m']
    centers = {j['id']: j['center'] for j in design['junctions']}
    objects = tree.findall('./road/objects/object')
    if len({o.get('id') for o in objects}) != len(objects):
        errors.append('Object IDs must be globally unique')
    if c['incoming_lanes'] != 3 or c['allowed_turns'] != [['left'], ['straight'], ['right']]:
        errors.append('Standard requires three single-turn lanes, without U-turns')
    for junction in design['junctions']:
        for rid, length in zip(junction['roads'], junction['lengths']):
            road = roads[rid]
            section = road.find('./lanes/laneSection')
            marks = section.findall('./center/lane/roadMark')
            if not marks or marks[0].get('type') != 'solid solid' or marks[0].get('color') != 'yellow':
                errors.append('Missing double yellow line: %s' % rid)
            crosswalk = road.find("./objects/object[@type='crosswalk']")
            stop = road.find("./objects/object[@name='stop_line']")
            if crosswalk is None or stop is None:
                errors.append('Missing crossing/stop line: %s' % rid)
                continue
            crossing_near = float(crosswalk.get('s'))-float(crosswalk.get('length'))/2
            stop_far = float(stop.get('s'))+float(stop.get('length'))/2
            if abs(crossing_near-stop_far-1) > 1e-6:
                errors.append('Stop line must precede crossing by 1m: %s' % rid)
            for lid in (-1, -2):
                solid = section.find("./right/lane[@id='%s']/roadMark[@type='solid']" % lid)
                if solid is None or abs(float(stop.get('s'))-float(solid.get('sOffset'))-25) > 1e-6:
                    errors.append('Missing 25m solid approach divider: %s/%s' % (rid, lid))
            if len(road.findall("./objects/object[@name='crosswalk_stripe']")) != 21:
                errors.append('Crossing stripe coverage mismatch: %s' % rid)
            for side in ('left', 'right'):
                lanes = section.findall('./%s/lane' % side)
                widths = [(l.get('type'), float(l.find('width').get('a'))) for l in lanes]
                expected = [('driving', 3.5)]*3
                if c['shoulder_enabled']:
                    expected.append(('shoulder', .5))
                expected += [('sidewalk', .2), ('sidewalk', 2.)]
                if widths != expected:
                    errors.append('Approach curb/shoulder dimensions differ: %s/%s' % (rid, side))
        for i in range(4):
            corner = roads[junction['id']-10+910+i]
            g = corner.find('./planView/geometry')
            heading = float(g.get('hdg'))
            x, y = float(g.get('x')), float(g.get('y'))
            curvature = float(g.find('arc').get('curvature'))
            sweep = float(g.get('length'))*curvature
            ex = x+(math.sin(heading+sweep)-math.sin(heading))/curvature
            ey = y-(math.cos(heading+sweep)-math.cos(heading))/curvature
            angle = i*math.pi/2
            cx, cy = junction['center']
            expected_start = (cx+half*math.cos(angle)-extent*math.sin(angle), cy+half*math.sin(angle)+extent*math.cos(angle))
            expected_end = (cx+extent*math.cos(angle)-half*math.sin(angle), cy+extent*math.sin(angle)+half*math.cos(angle))
            if max(math.hypot(x-expected_start[0], y-expected_start[1]), math.hypot(ex-expected_end[0], ey-expected_end[1])) > 1e-5:
                errors.append('Curved curb does not meet straight boundaries: %s' % corner.get('id'))
    checked = 0
    outside = []
    for wp in road_map.generate_waypoints(.5):
        if not wp.is_junction or wp.junction_id not in centers:
            continue
        # CARLA's waypoint list contains driving lanes here. Check both edges.
        cx, cy = centers[wp.junction_id]
        p, right = wp.transform.location, wp.transform.get_right_vector()
        for sign in (-1, 1):
            x = abs(p.x+sign*right.x*wp.lane_width/2-cx)
            y = abs(-(p.y+sign*right.y*wp.lane_width/2)-cy)
            if x > half and y > half and math.hypot(extent-x, extent-y) < radius-.03:
                outside.append((wp.road_id, wp.s))
            checked += 1
    if outside:
        errors.append('Driving lane edges cross rounded curb: %s samples' % len(outside))
    return dict(passed=not errors, errors=errors, lane_edge_samples=checked,
                limits='Offline geometry only; no vehicle swept-body, renderer, collision or pedestrian navigation certification')
