"""Deterministic roadside trees based on generated arm geometry."""
import math, random
import carla
from appearance_standard import settings
TREE_ASSETS=[('/Game/Carla/Static/Vegetation/Trees/SM_Acer_02.SM_Acer_02',5.0),('/Game/Carla/Static/Vegetation/Trees/SM_Beech.SM_Beech',7.0),('/Game/Carla/Static/Vegetation/Trees/SM_Cypress.SM_Cypress',3.0)]
def build(world,design,client=None):
 cfg=design['config']; style=settings(cfg).get('vegetation',{}); rng=random.Random(cfg.get('seed',42)+9173)
 owned=[]; pending=[]; rejected=dict(road=0,sidewalk=0,junction_visibility=0,ray=0,spawn=0); grid=max(int(cfg.get('rows',1)),int(cfg.get('columns',1)))
 density_scale=math.sqrt(max(1,grid))
 step=style.get('tree_spacing_m',10.)*density_scale
 rows=max(1,int(round(4.0/density_scale)))
 bp=world.get_blueprint_library().find('static.prop.mesh'); bp.set_attribute('role_name','standard_tree_scenery')
 half=cfg['incoming_lanes']*cfg['lane_width_m']+cfg.get('shoulder_width_m',0)+cfg['curb_width_m']+cfg['sidewalk_width_m']
 for j in design['junctions']:
  cx,cy=j['center']; arms=j.get('arms',{})
  for arm,rid in arms.items():
   length=dict(zip(j['roads'],j['lengths']))[rid]
   # CARLA stores OpenDRIVE Y with a sign flip; keep trees outside both sidewalks.
   ux,uy={'east':(1,0),'west':(-1,0),'north':(0,1),'south':(0,-1)}[arm]
   vx,vy=-uy,ux
   start=cfg['junction_radius_m']+24.; end=min(length-20.,length+cfg['junction_radius_m'])
   if end<=start: continue
   for along in [start+i*step for i in range(max(1,int((end-start)/step)+1))]:
    for side in (-1,1):
     for row in range(rows):
      lateral=half+8.0+row*8.0
      x=cx+ux*along+vx*side*lateral; y=cy+uy*along+vy*side*lateral
      asset,_=rng.choice(TREE_ASSETS); bp.set_attribute('mesh_path',asset)
      transform=carla.Transform(carla.Location(x=x,y=-y,z=-.2),carla.Rotation(yaw=rng.uniform(0,360)))
      if client is not None:
       pending.append(carla.command.SpawnActor(bp,transform).then(carla.command.SetSimulatePhysics(carla.command.FutureActor,False)).then(carla.command.SetEnableGravity(carla.command.FutureActor,False)))
      else:
       actor=world.try_spawn_actor(bp,transform)
       if actor is None: rejected['spawn']+=1; continue
       actor.set_simulate_physics(False); actor.set_collisions(False); owned.append(actor.id)
 if client is not None:
  for start in range(0,len(pending),300):
   results=client.apply_batch_sync(pending[start:start+300],False)
   for result in results:
    if result.error: rejected['spawn']+=1
    else: owned.append(result.actor_id)
 return dict(actor_ids=owned,count=len(owned),rejected=rejected,assets=[a[0] for a in TREE_ASSETS],grid_spacing_m=step,lane_edge_buffer_m=8.0,tree_rows_per_side=rows,density_scale=density_scale,ground_z_m=-.2,collision_policy='CARLA-native meshes; collisions disabled',boundary_policy='Generated-arm corridors; both sidewalk sides, junction visibility buffer')


