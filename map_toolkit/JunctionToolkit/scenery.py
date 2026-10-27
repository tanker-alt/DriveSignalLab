"""Cooked CARLA meshes plus a double-yellow centerline viewer overlay.

Underlying OpenDRIVE remains the authoritative driving/collision surface.
Asset dimensions are verified against the stock 0.9.15 installation.
"""
import math
import carla


def build(world, design, safety_rules=None):
    if design['config'].get('standard_profile') == 'urban_cross3_v1':
        from standard_preview import build as build_standard
        return build_standard(world, design)
    owned=[];failures=[];centerlines=[];m=world.get_map()
    n=design['config']['incoming_lanes'];width=design['config']['lane_width_m']
    radius=design['config']['junction_radius_m']
    stops={}
    if safety_rules:
        for line in safety_rules['stop_lines']:
            stops[line['road_id']]=min(stops.get(line['road_id'],float('inf')),line['s_m'])
    def mesh(asset,x,y,z,yaw=0,scale=1):
        bp=world.get_blueprint_library().find('static.prop.mesh')
        bp.set_attribute('mesh_path','/Game/Carla/Static/'+asset+'.'+asset.split('/')[-1])
        bp.set_attribute('scale',str(scale));bp.set_attribute('role_name','junction_toolkit_scenery')
        a=world.try_spawn_actor(bp,carla.Transform(carla.Location(x=x,y=y,z=z),carla.Rotation(yaw=yaw)))
        if a:
            a.set_simulate_physics(False);a.set_collisions(False);owned.append(a.id)
        else:failures.append(asset)
        return a
    # Road tiles use their baked asphalt material. Small visual-only overlap
    # with the original road is above it; never changes physics or topology.
    scale=width/4.073974
    tile_length=8.215106*scale
    # A simple tiled ground surface removes the floating-road appearance.
    span=max(design['config']['rows'],design['config']['columns'])*design['config']['spacing_m']+2*design['config']['arm_length_m']+100
    mesh('Ground/GroundCube',(design['config']['columns']-1)*design['config']['spacing_m']/2,
         -(design['config']['rows']-1)*design['config']['spacing_m']/2,-span/2-.2,0,span)
    for j in design['junctions']:
        for rid,length in zip(j['roads'],j['lengths']):
            for lane in range(-n,n+1):
                if not lane:continue
                count=max(2,int(math.ceil(length/tile_length)))
                for k in range(count):
                    s=tile_length/2+k*(length-tile_length)/(count-1)
                    wp=m.get_waypoint_xodr(rid,lane,s);t=wp.transform;r=t.get_right_vector();f=t.get_forward_vector()
                    # Baked mesh pivot is offset from its bounding-box center.
                    mesh('Road/St_Road_TileRoad_RoadL',t.location.x-r.x*2.035781*scale+f.x*.021558*scale,
                         t.location.y-r.y*2.035781*scale+f.y*.021558*scale,.012+k*.0005,t.rotation.yaw,scale)
            stop_s=stops.get(rid,length-6.)
            def marking(start_s,end_s,lateral,thickness,color):
                endpoints=[]
                for station in (start_s,end_s):
                    t=m.get_waypoint_xodr(rid,-1,station).transform;r=t.get_right_vector()
                    endpoints.append(carla.Location(x=t.location.x+r.x*(lateral-width/2),
                        y=t.location.y+r.y*(lateral-width/2),z=.09))
                world.debug.draw_line(endpoints[0],endpoints[1],thickness=thickness,
                    color=carla.Color(*color),life_time=0,persistent_lines=True)
                return endpoints
            # White same-direction separators; last 25m of approach is solid.
            # All approach markings terminate before the transverse stop paint.
            for boundary in range(-n+1,n):
                if boundary == 0:continue
                end=stop_s-.30 if boundary>0 else length-.1
                solid_start=max(.1,stop_s-25.) if boundary>0 else end
                station=2.
                while station<solid_start:
                    marking(station,min(station+3.,solid_start),boundary*width,.12,(2,2,2))
                    station+=9.
                if solid_start<end:
                    marking(solid_start,end,boundary*width,.12,(2,2,2))
            # Narrow double yellow ends upstream of stop paint, not in junction.
            for offset in [-.15,.15]:
                endpoints=marking(.1,stop_s-.30,offset,.12,(3,2,0))
                centerlines.append(dict(road_id=rid,offset_m=offset,render_thickness=.12,
                    end_s=stop_s-.30,stop_s=stop_s,
                    start=[endpoints[0].x,endpoints[0].y,endpoints[0].z],
                    end=[endpoints[1].x,endpoints[1].y,endpoints[1].z]))
        # Paved intersection tiles; shoulder fill below supplies collision.
        cx,cy=j['center'];sc=2*radius/600
        mesh('Road/RoadsTown03/SM_BaseRoad',cx,-cy,.012-.25*sc,0,sc)
    return dict(actor_ids=owned,house_ids=[],centerlines=centerlines,failed_assets=failures,
                road_material='stock St_Road_TileRoad_RoadL asphalt',
                markings='white dashed separators with 25m solid approach sections; narrow double yellow terminates before stop lines',
                collision_policy='appearance meshes have collisions disabled; OpenDRIVE road and sidewalk retain physics')
