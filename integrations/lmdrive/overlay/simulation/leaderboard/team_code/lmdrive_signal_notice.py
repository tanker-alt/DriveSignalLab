"""Camera-time roadside notice; no future snapshots or control overrides."""
def signal_notice(message, ego, camera_frame):
    if not message or message.get('ego_index') != ego:
        return '', None
    snap = message.get('camera_signal')
    if not snap or snap.get('carla_frame') != camera_frame or snap.get('alignment_method') != 'exact_snapshot':
        return '', None
    if message.get('camera_frame') != camera_frame or message.get('route_stage') != 'approach':
        return '', None
    state = snap['actual_state']
    movement = message['movement'].lower()
    text = ('Traffic signal for your %s movement: %s. '
            'Current signal phase ends in %.1f seconds. '
            'This is not a countdown to green.') % (movement, state.lower(), max(0., snap['phase_remaining_s']))
    return text, dict(ego_index=ego, traffic_light_id=message['traffic_light_id'],
                      movement=movement, camera_frame=camera_frame,
                      simulation_time=snap['simulation_time'], actual_state=state,
                      phase=snap['phase'], phase_remaining_s=snap['phase_remaining_s'])
