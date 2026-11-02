"""Flat standard cross/grid paint and cooked Grass1 ground, no debug drawing."""
import math
from concurrent.futures import ThreadPoolExecutor
import carla
from road_standard import stop_s
from appearance_standard import settings

ROLE = 'standard_surface_finish'

def build(world, design, client=None):
    for a in world.get_actors():
        if a.attributes.get('role_name') in (ROLE, 'appearance_probe', 'standard_road_ground'):
            a.destroy()
    ids=[];stops=[];pending=[];m=world.get_map();cfg=design['config'];style=settings(cfg);half=cfg['incoming_lanes']*cfg['lane_width_m'];width=cfg['lane_width_m']
    network=cfg['rows']*cfg['columns']>1
    library=world.get_blueprint_library()
    def mesh(asset,x,y,z,scale=1,yaw=0):
        bp=library.find('static.prop.mesh')
        bp.set_attribute('mesh_path','/Game/Carla/Static/'+asset+'.'+asset.split('/')[-1])
        bp.set_attribute('scale',str(scale));bp.set_attribute('role_name',ROLE)
        transform=carla.Transform(carla.Location(x,y,z),carla.Rotation(yaw=yaw))
        if client is not None:
            pending.append(carla.command.SpawnActor(bp,transform).then(carla.command.SetSimulatePhysics(carla.command.FutureActor,False)).then(carla.command.SetEnableGravity(carla.command.FutureActor,False)))
            return
        a=world.spawn_actor(bp,transform)
        a.set_simulate_physics(False);a.set_collisions(False);ids.append(a.id)
    for j in design['junctions']:
        for rid,length in zip(j['roads'],j['lengths']):
            # Negative lanes travel toward the junction. Positive lateral is
            # the driver's right, so [0,10.5] is precisely the incoming half.
            def point(s,t):
                tr=m.get_waypoint_xodr(rid,-1,max(.01,min(length-.01,s))).transform
                r=tr.get_right_vector()
                return tr.location.x+r.x*(t-1.75),tr.location.y+r.y*(t-1.75),tr.rotation.yaw
            def white(s0,t0,s1,t1,width):
                distance=math.hypot(s1-s0,t1-t0)
                count=max(1,int(math.ceil(distance/width)))
                scale=width/.124086
                for i in range(count):
                    along=width/2+i*max(0,distance-width)/max(1,count-1)
                    fraction=along/distance
                    x,y,yaw=point(s0+(s1-s0)*fraction,t0+(t1-t0)*fraction)
                    yaw+=math.degrees(math.atan2(t1-t0,s1-s0))
                    rad=math.radians(yaw)
                    mesh('RoadLine/SM_BassicTrafficLineCube_trafficlineCube',x-.061836*scale*math.cos(rad),y-.061836*scale*math.sin(rad),.006-.003803*scale,scale,yaw)
            stop=stop_s(length)
            white(stop,0,stop,half,.5)
            stops.append(dict(road_id=rid,s=stop,lanes=list(range(-1,-cfg['incoming_lanes']-1,-1)),lateral_range=[0,half]))
            for k in range(int(2*half)):white(length-6,-half+.25+k,length-2,-half+.25+k,.5)
            for t in [sign*k*width for sign in (-1,1) for k in range(1,cfg['incoming_lanes'])]:
                end=stop-.3 if t>0 else length-6.3
                along=.1
                while along<end:
                    white(along,t,min(along+style['markings']['dash_length_m'],end),t,.15)
                    along+=style['markings']['dash_length_m']+style['markings']['dash_gap_m']
            if not network:
                for t in (-half,half):white(.1,t,stop-.3 if t>0 else length-6.3,t,.15)
            for t in (-.15,.15):
                along=.65
                while along<length-7.7:
                    x,y,yaw=point(along,t)
                    mesh('RoadLine/St_Road_TileRoad_LaneMarkingBroken',x,y,.006-.029153*.35,.35,yaw)
                    along+=1.
    sidewalk_report=dict(actor_count=0,enabled=False,native_collision_retained=True)
    if not network:
        from runtime_sidewalks import build as sidewalks
        sidewalk_report=sidewalks(m.to_opendrive(),mesh)
        extent=cfg['arm_length_m']+cfg['junction_radius_m']+35
        xmax=(cfg['columns']-1)*cfg['spacing_m']+extent
        ymax=(cfg['rows']-1)*cfg['spacing_m']+extent
        # Flat Town10 grass patch; overlap tiles below native pavement/sidewalk.
        x=-extent
        while x<xmax:
            y=-extent
            while y<ymax:
                mesh(style['grass']['mesh'],x-76.219147,y+67.5,style['grass']['surface_z_m']-.152402)
                y+=40
            x+=24
    if client is not None:
        print('Surface commands prepared:',len(pending),flush=True)
        for start in range(0,len(pending),300):
            if start%3000==0:print('Surface spawn:',start,'/',len(pending),flush=True)
            results=client.apply_batch_sync(pending[start:start+300],False)
            for result in results:
                if result.error:raise RuntimeError(result.error)
                ids.append(result.actor_id)
        # RPC calls are independent. Bound concurrency to avoid a serial round
        # trip for every small paint slab on large networks.
        if not network:
            with ThreadPoolExecutor(max_workers=8) as pool:
                list(pool.map(lambda a:a.set_collisions(False),world.get_actors(ids)))
    return dict(actor_ids=ids,stop_lines=stops,sidewalks=sidewalk_report,paint_top_m=.006,appearance_standard=style,
                network_standard=network,collision_finalized=not network,grass='disabled' if network else 'CARLA cooked Grass1_Ground',
                grass_surface_z_m=-.2,debug_lines=False)
