"""Controlled parked CARLA vehicles; independent Shapely/dense-road oracle.

Requires an idle asynchronous world without vehicles. Owns ticks temporarily,
never changes map or lights, removes only owned actors and restores settings.
This validates static physical geometry, not dynamic following behaviour.
"""
import argparse
import json
import math
from datetime import datetime, timezone
from pathlib import Path
import carla
import numpy as np
from shapely.geometry import MultiPoint, LineString, Point
from shapely.ops import unary_union
from evaluation.map_context import read_inventory, junctions
from evaluation.approach_geometry import build, DEFAULT_APPROACH_CONFIG
from evaluation.approach_metrics import ApproachMetrics


def dense_path(road_map, lane, step=.025):
    first = lane['samples'][0]
    wp = road_map.get_waypoint(carla.Location(x=first['x'], y=first['y'], z=first['z']))
    points, widths = [(first['x'],first['y'])], [wp.lane_width]
    distance=0.
    target=lane['samples'][-1]['s']
    while distance < target:
        candidates=wp.previous(step)
        if len(candidates)!=1 or candidates[0].is_junction:
            break
        wp=candidates[0]
        p=wp.transform.location
        delta=np.asarray([p.x,p.y])-points[-1]
        length=float(np.linalg.norm(delta))
        if length<1e-7:
            break
        fraction=min(1.,(target-distance)/length)
        point=np.asarray(points[-1])+fraction*delta
        points.append(tuple(point)); widths.append(wp.lane_width)
        distance+=fraction*length
    return np.asarray(points), np.asarray(widths)


def oracle(rows, lane, road_map):
    points, widths = dense_path(road_map, lane)
    path = LineString(points)
    polygons = [MultiPoint([(p[0], p[1]) for p in r['bbox_vertices_world_json']]).convex_hull for r in rows]
    union = unary_union(polygons)
    occupied = 0.
    for a, b, width in zip(points[:-1], points[1:], widths):
        delta = b-a
        length = np.linalg.norm(delta)
        normal = np.array([-delta[1], delta[0]]) / length
        mid = (a+b)/2
        cross = LineString([mid-normal*width/2, mid+normal*width/2])
        if cross.intersection(union).length > 1e-6:
            occupied += length
    members=[]
    for row, poly in zip(rows, polygons):
        keys={(p['road_id'],p['section_id'],p['lane_id']) for p in lane['samples']}
        if (row['road_id'],row['section_id'],row['lane_id']) not in keys:
            continue
        origin=Point(row['x'],row['y'])
        station=path.project(origin)
        # Detect endpoint clipping rather than interpreting an external actor
        # as a member. Physical bbox projection extends the endpoint tangents.
        if origin.distance(path) > widths[0]/2 or station >= path.length-1e-4:
            continue
        distances=[]
        for p in poly.exterior.coords:
            s=path.project(Point(p))
            if s <= 1e-8:
                direction=(points[1]-points[0])/np.linalg.norm(points[1]-points[0])
                s=min(0.,float(np.dot(np.asarray(p)-points[0],direction)))
            elif s >= path.length-1e-8:
                direction=(points[-1]-points[-2])/np.linalg.norm(points[-1]-points[-2])
                s=path.length+max(0.,float(np.dot(np.asarray(p)-points[-1],direction)))
            distances.append(s)
        members.append(dict(id=row['vehicle_id'],front=min(distances),rear=max(distances),
                            speed=math.hypot(row['vx'],row['vy'])))
    members.sort(key=lambda m:m['front'])
    queue=[]
    for member in members:
        if member['speed']>1 or (not queue and member['front']>10) or (queue and member['front']-queue[-1]['rear']>5):
            break
        queue.append(member)
    return dict(primary_vehicle_count=len(members),queue_vehicle_count=len(queue),
                queue_tail_id=queue[-1]['id'] if queue else None,
                queue_length_m=queue[-1]['rear']-queue[0]['front'] if queue else 0.,
                spatial_occupancy=occupied/path.length, members=members)


def rows_from(world, actors):
    snapshot=world.get_snapshot()
    rows=[]
    for actor in actors:
        state=snapshot.find(actor.id)
        transform=state.get_transform()
        p=transform.location
        v,a=state.get_velocity(),state.get_acceleration()
        wp=world.get_map().get_waypoint(p,project_to_road=False)
        if wp is None:
            raise RuntimeError('Test origin is outside a road lane')
        vertices=actor.bounding_box.get_world_vertices(transform)
        rows.append(dict(vehicle_id=actor.id,type_id=actor.type_id,frame=snapshot.frame,
            x=p.x,y=p.y,z=p.z,yaw=transform.rotation.yaw,vx=v.x,vy=v.y,ax=a.x,ay=a.y,
            road_id=wp.road_id,section_id=wp.section_id,lane_id=wp.lane_id,
            bbox_vertices_world_json=[[p.x,p.y,p.z] for p in vertices]))
    return rows


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--junction-id',type=int,default=1427)
    args=parser.parse_args()
    client=carla.Client('localhost',2000); client.set_timeout(10)
    world=client.get_world(); original=world.get_settings()
    if original.synchronous_mode or world.get_actors().filter('vehicle.*'):
        raise RuntimeError('Requires idle asynchronous world with no vehicles')
    output=Path('evaluation/outputs')/('queue_acceptance_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
    output.mkdir()
    inventory=read_inventory(world,args.junction_id)
    config=json.loads(DEFAULT_APPROACH_CONFIG.read_text(encoding='utf-8'))
    geometry=build(world,inventory,config)
    def bend(lane):
        points=np.asarray([[p['x'],p['y']] for p in lane['samples']])
        return max(Point(p).distance(LineString([points[0],points[-1]])) for p in points)
    available=[lane for lane in geometry['lanes'] if lane['samples'][-1]['s']>30]
    straight=min(available,key=bend); curved=max(available,key=bend)
    curve_inventory=inventory
    # Town05/1427 is almost straight: find a genuinely curved signal approach
    # on the same loaded map, and record its separate junction explicitly.
    for jid in junctions(world):
        candidate_inventory=read_inventory(world,jid)
        if not candidate_inventory['traffic_lights']:
            continue
        candidate_geometry=build(world,candidate_inventory,config)
        for candidate in candidate_geometry['lanes']:
            if candidate['samples'][-1]['s']>30 and bend(candidate)>bend(curved):
                curved=candidate; curve_inventory=candidate_inventory
    cases=[('two_cars',straight,['vehicle.tesla.model3']*2,3,False,False),
           ('three_lengths',straight,['vehicle.tesla.model3','vehicle.volkswagen.t2','vehicle.carlamotors.carlacola'],3,False,False),
           ('disconnected_tail',straight,['vehicle.tesla.model3']*3,6,False,False),
           ('cross_lane',straight,['vehicle.carlamotors.carlacola'],3,True,False),
           ('tail_boundary',straight,['vehicle.tesla.model3']*3,3,False,True),
           ('curved_three',curved,['vehicle.tesla.model3']*3,3,False,False)]
    actors=[]; reports=[]
    settings=world.get_settings(); settings.synchronous_mode=True; settings.fixed_delta_seconds=.05
    try:
        world.apply_settings(settings)
        for name,lane,models,gap,cross,boundary in cases:
            case_inventory=curve_inventory if name=='curved_three' else inventory
            target=next(s for l in case_inventory['traffic_lights'] for s in l['stop_waypoints']
                        if '{}_{}_{}'.format(s['road_id'],s['section_id'],s['lane_id'])==lane['id'])
            stop=world.get_map().get_waypoint_xodr(target['road_id'],target['lane_id'],target['s'])
            rear=2.; centers=[]
            for index,model in enumerate(models):
                bp=world.get_blueprint_library().find(model)
                bp.set_attribute('role_name','queue_acceptance')
                staging=stop.previous(8+index*14)[0].transform
                staging.location.z+=.5
                actor=world.spawn_actor(bp,staging); actors.append(actor)
                actor.set_simulate_physics(False)
                extent=actor.bounding_box.extent.x
                center=rear+extent+actor.bounding_box.location.x
                wp=stop.previous(center)[0]
                transform=wp.transform
                transform.location.z+=.3
                if cross:
                    # Shift towards an actual adjacent driving lane.
                    choices=[(wp.get_left_lane(),-1),(wp.get_right_lane(),1)]
                    monitored={(p['road_id'],p['lane_id']) for l in geometry['lanes'] for p in l['samples']}
                    neighbor,sign=next((p,s) for p,s in choices if p and p.lane_type==carla.LaneType.Driving
                                       and (p.road_id,p.lane_id) in monitored)
                    vector=transform.get_right_vector()
                    shift=sign*(wp.lane_width/2-.25)
                    transform.location.x+=shift*vector.x; transform.location.y+=shift*vector.y
                actor.set_transform(transform)
                actor.apply_control(carla.VehicleControl(brake=1,hand_brake=True))
                centers.append(center)
                rear=center-actor.bounding_box.location.x+extent+(gap if index==len(models)-2 else 3)
            for actor in actors:
                actor.set_simulate_physics(True)
            for _ in range(50):
                world.tick()
            rows=rows_from(world,actors)
            case_config=dict(config)
            if boundary:
                case_config['detection_length_m']=centers[-1]+.5
            case_geometry=build(world,case_inventory,case_config)
            engine=ApproachMetrics(case_geometry)
            actual=engine.sample(rows)
            checks=[]
            for lane_geometry,result in zip(case_geometry['lanes'],actual):
                if result['primary_vehicle_count']==0 and not result['spatial_occupancy']:
                    continue
                expected=oracle(rows,lane_geometry,world.get_map())
                errors={key:abs(result[key]-expected[key]) for key in ('queue_length_m','spatial_occupancy')}
                passed=(all(result[k]==expected[k] for k in ('primary_vehicle_count','queue_vehicle_count','queue_tail_id'))
                        and errors['queue_length_m']<.15 and errors['spatial_occupancy']<.01)
                checks.append(dict(approach_id=lane_geometry['id'],actual=result,expected=expected,errors=errors,passed=bool(passed)))
            report=dict(case=name,selected_approach=lane['id'],centerline_bend_m=bend(lane),rows=rows,
                checks=checks,geometry=case_geometry,passed=bool(checks) and all(c['passed'] for c in checks),
                max_speed_mps=max(math.hypot(r['vx'],r['vy']) for r in rows))
            if report['max_speed_mps']>.1:
                report['passed']=False
            if cross and sum(c['actual']['spatial_occupancy']>0 for c in checks)<2:
                report['passed']=False
            if name=='curved_three' and bend(lane)<1:
                report['passed']=False
            reports.append(report)
            (output/(name+'.json')).write_text(json.dumps(report,indent=2),encoding='utf-8')
            print(name,report['passed'],[(c['approach_id'],c['errors']) for c in checks],flush=True)
            for actor in actors:
                actor.destroy()
            world.tick()
            if {a.id for a in actors}.intersection(a.id for a in world.get_actors().filter('vehicle.*')):
                raise RuntimeError('Owned actor cleanup failed')
            actors=[]
    finally:
        for actor in actors:
            if actor.is_alive:
                actor.destroy()
        world.apply_settings(original)
        summary=dict(output=str(output),cases=[dict(case=r['case'],passed=r['passed']) for r in reports],
            completed_all_cases=len(reports)==len(cases),
            restored_synchronous_mode=world.get_settings().synchronous_mode,
            remaining_vehicles=len(world.get_actors().filter('vehicle.*')),
            oracle='Shapely geometry + independent 0.025m CARLA waypoint sampling',
            tolerances=dict(queue_length_m=.15,spatial_occupancy=.01),
            scope='parked physical actors; reference waypoints, not surveyed painted stop lines')
        (output/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
        print(json.dumps(summary,indent=2))


if __name__=='__main__':
    main()
