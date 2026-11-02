"""RoadRunner Round-Cross2 style using packaged CARLA equivalents."""
import json, math, random
from pathlib import Path
import carla

def build(world, design, client=None, host='localhost', port=2000):
    style=json.loads((Path(__file__).parent/'templates/round_cross2/appearance.json').read_text())
    ids=[]; failures=[]; pending=[]; library=world.get_blueprint_library()
    def mesh(asset,x,y,z,yaw=0,scale=1):
        bp=library.find('static.prop.mesh');bp.set_attribute('mesh_path',asset);bp.set_attribute('scale',str(scale));bp.set_attribute('role_name','roundabout_template')
        transform=carla.Transform(carla.Location(x,y,z),carla.Rotation(yaw=yaw))
        if client is not None:
            pending.append(carla.command.SpawnActor(bp,transform).then(carla.command.SetSimulatePhysics(carla.command.FutureActor,False)).then(carla.command.SetEnableGravity(carla.command.FutureActor,False)))
            ids.append(None)
            return
        a=world.try_spawn_actor(bp,transform)
        if a is None: failures.append([asset,x,y]);return
        a.set_simulate_physics(False);a.set_collisions(False);ids.append(a.id)
    road_map=world.get_map()
    paint_start=len(ids)
    paint_asset='/Game/Carla/Static/RoadLine/SM_BassicTrafficLineCube_trafficlineCube.SM_BassicTrafficLineCube_trafficlineCube'
    for rid in list(range(1,5))+design['ring_roads']+design['ring_continuations']:
        length=design['road_geometry'][str(rid)]['length']
        for lane in ((-1,1) if rid<10 else (-1,)):
            station=1.0
            while station<length-1:
                if station%9<3 and not (rid<10 and station>length-4.75):
                    wp=road_map.get_waypoint_xodr(rid,lane,station)
                    if wp:
                        r=wp.transform.get_right_vector();p=wp.transform.location+r*(wp.lane_width/2)
                        scale=.15/.124086; yaw=wp.transform.rotation.yaw; rad=math.radians(yaw)
                        mesh(paint_asset,p.x-.061836*scale*math.cos(rad),p.y-.061836*scale*math.sin(rad),.006-.003803*scale,yaw,scale)
                station+=.15
    yellow_start=len(ids)
    yellow='/Game/Carla/Static/RoadLine/St_Road_TileRoad_LaneMarkingBroken.St_Road_TileRoad_LaneMarkingBroken'
    for rid in range(1,5):
        length=design['road_geometry'][str(rid)]['length']
        station=.65
        while station<length-4.75:
            wp=road_map.get_waypoint_xodr(rid,-1,station)
            right=wp.transform.get_right_vector()
            for offset in (-.15,.15):
                p=wp.transform.location+right*(-wp.lane_width/2+offset)
                mesh(yellow,p.x,p.y,.006-.029153*.35,wp.transform.rotation.yaw,.35)
            station+=1.
    yellow_count=len(ids)-yellow_start
    # White paint slabs share the measured material/top offset used above.
    crossing_reports=[]; stop_reports=[]
    def white_at(rid,station,lateral):
        wp=road_map.get_waypoint_xodr(rid,-1,station)
        if wp is None:raise RuntimeError('Missing crossing/stop waypoint')
        p=wp.transform.location+wp.transform.get_right_vector()*(lateral-wp.lane_width/2)
        scale=.5/.124086; yaw=wp.transform.rotation.yaw; rad=math.radians(yaw)
        mesh(paint_asset,p.x-.061836*scale*math.cos(rad),p.y-.061836*scale*math.sin(rad),
             wp.transform.location.z+.006-.003803*scale,yaw,scale)
    lane_width=design['config']['lane_width_m']
    half=design['config']['incoming_lanes']*lane_width
    for rid in range(1,5):
        length=design['road_geometry'][str(rid)]['length']
        # 4 m crossing, 0.5 m bars separated by 0.5 m, across both directions.
        for stripe in range(int(2*half)):
            lateral=-half+.25+stripe
            for k in range(8):white_at(rid,length-4.25+k*.5,lateral)
        crossing_reports.append(dict(road_id=rid,s_start=length-4.5,s_end=length-.5,width_m=4.))
    # Honour the same reference_s used by the signal/stop logic; never draw
    # the entry stop bar across the outgoing carriageway.
    seen=set()
    for signal in design['signals'].values():
        rid=signal['road_id'];station=signal['reference_s']
        if (rid,station) in seen:continue
        seen.add((rid,station))
        for k in range(int(half/.5)):white_at(rid,station,.25+k*.5)
        stop_reports.append(dict(road_id=rid,s=station,lateral_range=[0,half],role=signal['role']))
    paint_count=len(ids)-paint_start
    extent=210
    for x in range(-extent,extent+1,24):
        for y in range(-extent,extent+1,40):
            mesh(style['grass'],x-76.219147,y+67.5,-.352402)
    ground_count=len(ids)-paint_count
    sidewalk_start=len(ids)
    sidewalk='/Game/Carla/Static/SideWalk/SideWalkCube.SideWalkCube'
    # Match the cross-junction concrete cap: 2 m wide, top at 0.16 m.
    for rid in range(1,5):
        length=design['road_geometry'][str(rid)]['length']
        for lane in (-2,2):
            station=1.
            while station<length:
                wp=road_map.get_waypoint_xodr(rid,lane,station)
                p=wp.transform.location+wp.transform.get_right_vector()*(wp.lane_width/2+1.)
                mesh(sidewalk,p.x,p.y,-.84,wp.transform.rotation.yaw,2.)
                station+=2.
    # Inner ring reference line is the inner asphalt boundary. Keep all
    # concrete on the island side; small tangent tiles follow the circle.
    inner_edge=design['config']['ring_centerline_radius_m']-design['config']['lane_width_m']/2
    tile=.5
    inner_start=len(ids)
    sidewalk_width=style.get('sidewalk_width_m',2.)
    rows=int(math.ceil(sidewalk_width/tile))
    for row in range(rows):
        radius=inner_edge-.03-(row+.5)*tile
        count=int(math.ceil(2*math.pi*radius/(tile*.98)))
        for k in range(count):
            theta=2*math.pi*k/count
            mesh(sidewalk,radius*math.cos(theta),radius*math.sin(theta),.16-tile/2,
                 math.degrees(theta)+90,tile)
    inner_sidewalk_count=len(ids)-inner_start
    # Follow only the exposed outer boundary: outer entry/exit connectors
    # plus inter-access ring arcs. Ring continuations cross the mouths and
    # must not carry a sidewalk across traffic.
    outer_start=len(ids)
    outer_edges=[(rid,-2) for rid in design['ring_roads']]
    outer_edges += [(100+i*10+offset,-1) for i in range(4) for offset in (2,4)]
    for rid,lane in outer_edges:
        length=design['road_geometry'][str(rid)]['length']
        count=int(math.ceil(length/(tile*.85)))
        for k in range(count+1):
            station=max(.001,min(length-.001,k*length/count))
            wp=road_map.get_waypoint_xodr(rid,lane,station)
            if wp is None:raise RuntimeError('Missing outer sidewalk waypoint: '+str(rid))
            right=wp.transform.get_right_vector()
            for row in range(rows):
                lateral=wp.lane_width/2+.03+(row+.5)*tile
                pos=wp.transform.location+right*lateral
                mesh(sidewalk,pos.x,pos.y,.16-tile/2,wp.transform.rotation.yaw,tile)
    outer_sidewalk_count=len(ids)-outer_start
    sidewalk_count=len(ids)-sidewalk_start
    rng=random.Random(design['config'].get('seed',42))
    tree_positions=[]
    # Four parallel rows per side, aligned with each approach, not a forest grid.
    for rid in range(1,5):
        length=design['road_geometry'][str(rid)]['length']
        station=8.
        while station<length-24.:
            for lane in (-2,2):
                wp=road_map.get_waypoint_xodr(rid,lane,station)
                for row in range(style.get('tree_rows_per_side',4)):
                    lateral=wp.lane_width/2+sidewalk_width+5.+row*7.
                    pos=wp.transform.location+wp.transform.get_right_vector()*lateral
                    nearest=road_map.get_waypoint(pos,project_to_road=True,lane_type=carla.LaneType.Driving)
                    if nearest and pos.distance(nearest.transform.location)<9.:continue
                    mesh(rng.choice(style['trees']),pos.x,pos.y,-.2,rng.uniform(0,360))
                    tree_positions.append([pos.x,pos.y])
            station+=style.get('tree_spacing_m',10.)
    # Fill the four spaces between approaches with curved roadside groves.
    grove_start=len(ids)
    for quadrant in range(4):
        for radius in (52.,62.,72.):
            for degrees in range(23,69,11):
                theta=math.radians(quadrant*90+degrees)
                pos=carla.Location(radius*math.cos(theta),radius*math.sin(theta),0)
                nearest=road_map.get_waypoint(pos,project_to_road=True,lane_type=carla.LaneType.Driving)
                if nearest and pos.distance(nearest.transform.location)<12.:continue
                if any(math.hypot(pos.x-x,pos.y-y)<7. for x,y in tree_positions):continue
                mesh(rng.choice(style['trees']),pos.x,pos.y,-.2,rng.uniform(0,360))
                tree_positions.append([pos.x,pos.y])
    grove_count=len(ids)-grove_start
    if client is not None:
        from concurrent.futures import ProcessPoolExecutor
        from network_preview import disable_collision_batch
        print('Roundabout surface commands:',len(pending),flush=True)
        ids=[]
        for start in range(0,len(pending),300):
            for result in client.apply_batch_sync(pending[start:start+300],False):
                if result.error:raise RuntimeError(result.error)
                ids.append(result.actor_id)
        batches=[(ids[i:i+500],host,port) for i in range(0,len(ids),500)]
        with ProcessPoolExecutor(max_workers=16) as pool:
            assert sum(pool.map(disable_collision_batch,batches))==len(ids)
        print('Roundabout decoration collisions disabled:',len(ids),flush=True)
    return dict(actor_ids=ids,crosswalks=crossing_reports,stop_lines=stop_reports,ground_count=ground_count,paint_count=paint_count,sidewalk_count=sidewalk_count,inner_sidewalk_count=inner_sidewalk_count,outer_sidewalk_count=outer_sidewalk_count,tree_positions=tree_positions,grove_count=grove_count,yellow_count=yellow_count,tree_count=len(ids)-ground_count-paint_count-sidewalk_count,failures=failures,template=style,markings='runtime white dashes and continuous double-yellow approach centre lines; no debug overlays')
