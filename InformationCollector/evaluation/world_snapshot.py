"""Current-frame world truth for evaluators; no scores, future data or tick calls."""
import math

def collect_world(world, snapshot):
    actors, lights, polygons, errors = [], {}, {}, []
    try:
        world_actors = list(world.get_actors())
    except (TypeError, AttributeError) as exc:
        return dict(frame=snapshot.frame, t=snapshot.timestamp.elapsed_seconds, actors=[],
                    traffic_lights={}, traffic_light_polygons={}, complete=False,
                    errors=[dict(reason=str(exc))])
    for actor in world_actors:
        kind = actor.type_id
        try:
            if kind.startswith(('vehicle.', 'walker.pedestrian.')):
                state = snapshot.find(actor.id)
                if state is None:
                    continue
                tf, vel, acc = state.get_transform(), state.get_velocity(), state.get_acceleration()
                ang = state.get_angular_velocity()
                ext = actor.bounding_box.extent
                actors.append(dict(id=actor.id, type_id=kind, x=tf.location.x, y=tf.location.y,
                    yaw=math.radians(tf.rotation.yaw), vel_x=vel.x, vel_y=vel.y,
                    accel_x=acc.x, accel_y=acc.y, ang_vel=ang.z,
                    extent_x=ext.x, extent_y=ext.y))
            elif 'traffic_light' in kind:
                lights[actor.id] = int(actor.get_state())
                trigger = actor.trigger_volume
                tf = actor.get_transform()
                import carla
                corners = [carla.Location(x=trigger.location.x+sx*trigger.extent.x,
                                          y=trigger.location.y+sy*trigger.extent.y,
                                          z=trigger.location.z)
                           for sx, sy in [(-1,-1),(-1,1),(1,1),(1,-1)]]
                for corner in corners:
                    tf.transform(corner)
                polygons[actor.id] = [(p.x,p.y) for p in corners]
        except (AttributeError, RuntimeError, TypeError, ValueError) as exc:
            errors.append(dict(actor_id=actor.id, reason=str(exc)))
    same_frame = world.get_snapshot().frame == snapshot.frame
    return dict(frame=snapshot.frame, t=snapshot.timestamp.elapsed_seconds, actors=actors,
                traffic_lights=lights, traffic_light_polygons=polygons,
                complete=not errors and same_frame, errors=errors)
