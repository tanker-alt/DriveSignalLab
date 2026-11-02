"""Keep native controllable heads; replace intrusive stock supports with gantries."""
import math
import carla


def build(world, design):
    if design.get('signal_policy') != 'one_signal_per_incoming_lane':
        return None
    road_map=world.get_map();n=design['config']['incoming_lanes'];width=design['config']['lane_width_m']
    lights={l.get_opendrive_id():l for l in world.get_actors().filter('traffic.traffic_light')}
    poses=[]
    for sid,cfg in design['signals'].items():
        boxes=lights[sid].get_light_boxes()
        if len(boxes)!=1:raise RuntimeError('Expected one native head: '+sid)
        head=boxes[0];wp=road_map.get_waypoint_xodr(cfg['road_id'],cfg['incoming_lanes'][0],cfg.get('signal_s',cfg['reference_s']+3))
        right=wp.transform.get_right_vector();delta=head.location-wp.transform.location
        error=abs(delta.x*right.x+delta.y*right.y)
        if error>.1 or head.location.z-head.extent.z<4.5:
            raise RuntimeError('Native head not safely aligned over its lane: '+sid)
        poses.append(dict(signal_id=sid,road_id=cfg['road_id'],lane_id=cfg['incoming_lanes'][0],lateral_error_m=error,
                          head=[head.location.x,head.location.y,head.location.z]))
    poles=[o.id for o in world.get_environment_objects(carla.CityObjectLabel.Poles) if 'BP_TLOpenDrive' in o.name]
    if len(poles)!=3*len(lights):raise RuntimeError('Unknown stock support layout; refusing partial hiding')
    world.enable_environment_objects(set(poles),False)
    owned=[]
    frames=[]
    def mesh(path,location,rotation=carla.Rotation(),scale=1.):
        bp=world.get_blueprint_library().find('static.prop.mesh')
        bp.set_attribute('mesh_path',path);bp.set_attribute('scale',str(scale));bp.set_attribute('role_name','lane_signal_gantry')
        actor=world.spawn_actor(bp,carla.Transform(location,rotation));owned.append(actor.id)
        actor.set_simulate_physics(False);actor.set_collisions(False)
        return actor
    # Measured stock mesh bounding interval along its local Z axis (metres).
    bottom=.464282;length=5.843178
    asset='/Game/Carla/Static/Pole/TrafficLight/SM_TrafficLight__VPole_Main.SM_TrafficLight__VPole_Main'
    def beam(start,end):
        dx,dy,dz=end.x-start.x,end.y-start.y,end.z-start.z
        dist=math.sqrt(dx*dx+dy*dy+dz*dz);scale=dist/length
        unit=carla.Vector3D(dx/dist,dy/dist,dz/dist)
        rotation=carla.Rotation(pitch=-math.degrees(math.acos(unit.z)),yaw=math.degrees(math.atan2(unit.y,unit.x)))
        actual=rotation.get_up_vector()
        if abs(actual.x-unit.x)+abs(actual.y-unit.y)+abs(actual.z-unit.z)>.001:
            raise RuntimeError('Gantry beam rotation mismatch')
        origin=carla.Location(start.x-unit.x*bottom*scale,start.y-unit.y*bottom*scale,start.z-unit.z*bottom*scale)
        mesh(asset,origin,rotation,scale)
    for j in design['junctions']:
        for rid,length_m in zip(j['roads'],j['lengths']):
            wp=road_map.get_waypoint_xodr(rid,-1,length_m-3)
            right=wp.transform.get_right_vector();forward=wp.transform.get_forward_vector()
            # Official complete mast/curved arm/street-lamp mesh; local +X
            # points from the roadside pole into the road.
            def point(lateral,z):
                p=wp.transform.location
                return carla.Location(p.x+right.x*(lateral-width/2)-forward.x*.42,
                                      p.y+right.y*(lateral-width/2)-forward.y*.42,z)
            curb=n*width+design['config'].get('shoulder_width_m',0)+.8
            model='SM_TrafficLight01' if n>=3 else 'SM_TrafficLight02'
            span=11.879336 if n>=3 else 7.31334
            scale=max(1.,(curb-.35)/span)
            base_z=.65 if n>=3 else .8
            yaw=math.degrees(math.atan2(-right.y,-right.x))
            actor=mesh('/Game/Carla/Static/Pole/'+model+'.'+model,
                       point(curb,base_z),carla.Rotation(yaw=yaw),scale)
            mesh('/Game/Carla/Static/Pole/TrafficLight/SM_TrafficLight__Ground_base.SM_TrafficLight__Ground_base',
                 point(curb,.15),carla.Rotation(yaw=yaw),1.5)
            # Centreline samples of the official lower arm (asset-local m).
            profile=[(0,5.28),(4.,6.35),(4.5,6.42),(11.879336,6.51)] if n>=3 else [(0,5.2),(2,5.78),(4,6.15),(7.31334,6.4)]
            hangers=[]
            for lane in range(n):
                lateral=(lane+.5)*width;distance=(curb-lateral)/scale
                for (x0,z0),(x1,z1) in zip(profile,profile[1:]):
                    if x0<=distance<=x1:
                        top=base_z+scale*(z0+(z1-z0)*(distance-x0)/(x1-x0));break
                else:raise RuntimeError('Lane outside official arm span')
                if top<=6.13:raise RuntimeError('Official arm below lamp mount')
                beam(point(lateral,6.13),point(lateral,top))
                hangers.append(dict(lane_id=-(lane+1),arm_attachment_z=top))
            frames.append(dict(road_id=rid,actor_id=actor.id,asset=model,scale=scale,hangers=hangers))
    return dict(native_heads=poses,hidden_stock_supports=poles,gantry_actor_ids=owned,
                model='native CARLA controllable heads + official complete mast meshes',official_frames=frames,
                collision_policy='support meshes visual only; original lane triggers retained')
