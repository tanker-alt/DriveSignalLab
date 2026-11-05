"""A full circulating lap, including every access junction, then a real exit."""


def plan(m, design, lane=-1):
    from network_route_check import follow, waypoint_record
    start=m.get_waypoint_xodr(1,lane,design['config']['arm_length_m']-35.)
    current=start; waypoints=[]
    # Return to the first ring segment before exiting at the following arm.
    # This proves a complete cycle rather than merely reaching a nearby exit.
    checkpoints=[(road,lane,5.) for road in (20,21,22,23,20)]
    checkpoints.append((2,-lane,design['config']['arm_length_m']-35.))
    for road,target_lane,station in checkpoints:
        segment=follow(current,road,target_lane,station)
        if any(w.road_id in range(1,5) and w.lane_id>0 for w in segment[:-1]) and road not in range(1,5):
            raise ValueError('Route left the roundabout before completing the lap')
        waypoints.extend(segment[1:] if waypoints else segment)
        current=segment[-1]
    order=[]
    for w in waypoints:
        if w.is_junction and (not order or order[-1]!=w.junction_id):order.append(w.junction_id)
    if set(order)!={j['id'] for j in design['junctions']}:
        raise ValueError('Roundabout route omits an access junction')
    points=[w.transform.location for w in waypoints]
    return points,start,dict(junction_order=order,start=waypoint_record(start),destination=waypoint_record(current),
                            movements=[],lane_changes=[],ring_sequence=[20,21,22,23,20],lane=lane,
                            length_m=sum(a.distance(b) for a,b in zip(points,points[1:])))


def audit(report, route, design):
    observed=[]
    for row in report['road_transitions']:
        if row['road'] in design['ring_roads'] and (not observed or observed[-1]!=row['road']):
            observed.append(row['road'])
    report['observed_ring_sequence']=observed
    report['expected_ring_sequence']=route['ring_sequence']
    report['seam_sequence_passed']=observed==route['ring_sequence']
    return report['seam_sequence_passed']
