"""Load candidate roundabout explicitly; all entry signals held red, no phase plan."""
import argparse,hashlib,json,queue
from pathlib import Path
import carla
from lane_signals import build


def load(folder,host='localhost',port=2000,photos=True,overview=False):
    folder=Path(folder);d=json.loads((folder/'manifest.json').read_text());raw=(folder/'map.xodr').read_bytes()
    if not json.loads((folder/'topology_validation.json').read_text())['passed']:raise ValueError('Topology failed')
    if hashlib.sha256(raw).hexdigest()!=d['opendrive_sha256']:raise ValueError('Map hash mismatch')
    client=carla.Client(host,port);client.set_timeout(120);w=client.get_world()
    if w.get_settings().synchronous_mode:raise RuntimeError('Finish current experiment first')
    w.wait_for_tick(15)
    if any(w.get_actors().filter(p) for p in ('vehicle.*','sensor.*','walker.*')):raise RuntimeError('Active actors; finish experiment first')
    old_camera=w.get_spectator().get_transform()
    saved=dict(location=dict(x=old_camera.location.x,y=old_camera.location.y,z=old_camera.location.z),rotation=dict(pitch=old_camera.rotation.pitch,yaw=old_camera.rotation.yaw,roll=old_camera.rotation.roll))
    (folder/'observer_before_load.json').write_text(json.dumps(saved,indent=2))
    w=client.generate_opendrive_world(raw.decode(),carla.OpendriveGenerationParameters(vertex_distance=1.,max_road_length=30.,wall_height=0.,additional_width=0.,smooth_junctions=False,enable_mesh_visibility=True,enable_pedestrian_navigation=False))
    for _ in range(10):w.wait_for_tick(15)
    if w.get_map().to_opendrive()!=raw.decode():raise RuntimeError('Server map differs')
    lights={a.get_opendrive_id():a for a in w.get_actors().filter('traffic.traffic_light')}
    if set(lights)!=set(d['signals']):raise RuntimeError('Signal IDs differ')
    for sid,cfg in d['signals'].items():
        light=lights[sid]
        if {(p.road_id,p.lane_id) for p in light.get_affected_lane_waypoints()}!={(cfg['road_id'],cfg['incoming_lanes'][0])}:raise RuntimeError('Lane ownership differs: '+sid)
        light.set_state(carla.TrafficLightState.Red);light.freeze(True)
    support=build(w,d)
    decoration=[]
    def mesh(asset,scale,z):
        bp=w.get_blueprint_library().find('static.prop.mesh');bp.set_attribute('mesh_path',asset);bp.set_attribute('scale',str(scale));bp.set_attribute('role_name','roundabout_scenery')
        a=w.spawn_actor(bp,carla.Transform(carla.Location(0,0,z)));a.set_simulate_physics(False);a.set_collisions(False);decoration.append(a.id)
    mesh('/Game/Carla/Static/Ground/GroundCube.GroundCube',500,-250.2)
    diameter=2*(d['config']['ring_centerline_radius_m']-d['config']['lane_width_m']/2-1)
    mesh('/Game/Carla/Static/Pole/SM_PoleCylinder.SM_PoleCylinder',diameter,-diameter/2+.25)
    w.set_weather(carla.WeatherParameters(cloudiness=25,sun_altitude_angle=55,sun_azimuth_angle=110))
    for cfg in d['signals'].values():
        wp=w.get_map().get_waypoint_xodr(cfg['road_id'],cfg['incoming_lanes'][0],cfg['reference_s']);right=wp.transform.get_right_vector();p=wp.transform.location;p.z=.08
        w.debug.draw_line(p-right*(wp.lane_width*.46),p+right*(wp.lane_width*.46),thickness=.12,color=carla.Color(230,230,230),life_time=0,persistent_lines=True)
    from roundabout_marks import build as marks
    group_labels=marks(w,d)
    # Preserve user's observer pose. Preview sensors never move the spectator.
    w.get_spectator().set_transform(carla.Transform(carla.Location(-58,68,95),carla.Rotation(pitch=-48,yaw=-49,roll=0)) if overview else old_camera)
    report=dict(signal_count=len(lights),signal_validity_passed=True,phase_plan=None,all_signal_state='all_red_frozen',lane_signals=support,
                signal_groups=d.get('signal_groups'),group_label_positions=group_labels,decoration_ids=decoration,
                limits='Native OpenDRIVE meshes with visual-only ground/island and stop-reference overlays; driving remains experimental.')
    (folder/'load_report.json').write_text(json.dumps(report,indent=2))
    if photos:
        bp=w.get_blueprint_library().find('sensor.camera.rgb');bp.set_attribute('image_size_x','1400');bp.set_attribute('image_size_y','1400')
        q=queue.Queue();cam=w.spawn_actor(bp,carla.Transform(carla.Location(0,0,125),carla.Rotation(pitch=-90,yaw=-90)))
        try:
            cam.listen(q.put)
            for _ in range(10):frame=q.get(timeout=20)
            frame.save_to_disk(str(folder/'carla_overview.png'))
        finally:cam.stop();cam.destroy()
    print('Loaded roundabout:',folder,len(lights),'independent entry/circulating signals held RED. No phase schedule installed; camera:',('overview with roll=0' if overview else 'previous pose preserved'))
    return w,d


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('map_dir');p.add_argument('--no-photos',action='store_true');p.add_argument('--overview',action='store_true');a=p.parse_args();load(a.map_dir,photos=not a.no_photos,overview=a.overview)
