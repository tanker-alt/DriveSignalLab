"""Concrete caps make the native ring sidewalks visible in CARLA's flat mesh."""
import math
import carla


def build(client,world,design):
    road_map=world.get_map();pending=[];ids=[];samples=[]
    blueprint=world.get_blueprint_library().find('static.prop.mesh')
    blueprint.set_attribute('mesh_path','/Game/Carla/Static/SideWalk/SideWalkCube.SideWalkCube')
    blueprint.set_attribute('role_name','ring_perimeter_sidewalk')
    import xml.etree.ElementTree as ET
    root=ET.fromstring(road_map.to_opendrive())
    roads={int(r.get('id')):r for r in root.findall('road')}
    for ring in design.get('roundabouts',[]):
        paths=ring.get('sidewalk_connections',[])
        if not paths:continue
        selected=set(ring['ring_roads'])|{p['road'] for p in paths}
        arms={p[k] for p in paths for k in ('source','target') if p[k] not in ring['ring_roads']}
        for rid in sorted(selected|arms):
            road=roads[rid];length=float(road.get('length'))
            start=max(0,length-12) if rid in arms else 0
            for lane in road.findall("./lanes/laneSection/*/lane[@type='sidewalk']"):
                lid=int(lane.get('id'));width=float(lane.find('width').get('a'))
                rows=max(1,int(math.ceil(width/.5)));tile=width/rows
                count=int(math.ceil((length-start)/(.85*tile)))
                for i in range(count):
                    station=start+(i+.5)*(length-start)/count
                    wp=road_map.get_waypoint_xodr(rid,lid,station)
                    if wp is None:raise ValueError('Missing native ring sidewalk')
                    right=wp.transform.get_right_vector()
                    for row in range(rows):
                        lateral=-width/2+(row+.5)*tile
                        p=wp.transform.location+right*lateral
                        blueprint.set_attribute('scale',str(tile))
                        transform=carla.Transform(carla.Location(p.x,p.y,.16-tile/2),carla.Rotation(yaw=wp.transform.rotation.yaw))
                        pending.append(carla.command.SpawnActor(blueprint,transform))
                samples.append(dict(road=rid,lane=lid,s_start=start,s_end=length,width_m=width))
    # No physics/collision overlays: native OpenDRIVE retains road collision.
    try:
        for offset in range(0,len(pending),300):
            results=client.apply_batch_sync(pending[offset:offset+300],False)
            for result in results:
                if result.error:raise RuntimeError(result.error)
                ids.append(result.actor_id)
            for actor in world.get_actors([r.actor_id for r in results]):
                actor.set_simulate_physics(False);actor.set_collisions(False)
    except Exception:
        client.apply_batch([carla.command.DestroyActor(i) for i in ids]);raise
    return dict(actor_ids=ids,actor_count=len(ids),bands=samples,collision=False,surface_z_m=.16)
