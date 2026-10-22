"""Fixed-time policy and CARLA bridge; the policy can be replaced independently."""
import json
import math
from pathlib import Path


def project_arc(location, points, cumulative):
    best = (float('inf'), 0.0)
    for i, (a, b) in enumerate(zip(points, points[1:])):
        dx, dy = b.x-a.x, b.y-a.y
        norm = dx*dx + dy*dy
        if norm < 1e-8:
            continue
        q = max(0, min(1, ((location.x-a.x)*dx + (location.y-a.y)*dy)/norm))
        dist = (location.x-a.x-q*dx)**2 + (location.y-a.y-q*dy)**2
        if dist < best[0]:
            best = (dist, cumulative[i] + q*math.sqrt(norm))
    return best[1]


def camera_signal_at_frame(history, frame, light_id):
    # A completed ego may outlive its removed camera during actor cleanup.
    # No camera observation means no aligned signal, not a synthetic frame.
    if frame is None:
        return None
    record = history.get(frame)
    method = 'exact_snapshot'
    if record is None:
        before = [f for f in history if f < frame]
        after = [f for f in history if f > frame]
        if not before or not after:
            return None
        left, right = history[max(before)], history[min(after)]
        # CARLA actor teardown can advance hidden ticks. Only bridge gaps
        # proven to have one unchanged frozen phase at both endpoints.
        if left['actual_states'] != right['actual_states'] or left['phase'] != right['phase']:
            return None
        q = (frame-left['carla_frame'])/(right['carla_frame']-left['carla_frame'])
        record = dict(left, carla_frame=frame,
                      simulation_time=left['simulation_time']+q*(right['simulation_time']-left['simulation_time']),
                      elapsed_s=left['elapsed_s']+q*(right['elapsed_s']-left['elapsed_s']),
                      phase_remaining_s=left['phase_remaining_s']+q*(right['phase_remaining_s']-left['phase_remaining_s']))
        method = 'stable_interval'
    return {'carla_frame':record['carla_frame'],'simulation_time':record['simulation_time'],
            'elapsed_s':record['elapsed_s'],'phase':record['phase'],'phase_remaining_s':record['phase_remaining_s'],
            'actual_state':record['actual_states'][light_id], 'alignment_method':method}


class TrafficLightAgent:
    def __init__(self, green_seconds=10.0, yellow_seconds=3.0, clearance_seconds=2.0, initial_red_seconds=6.0):
        self.initial_red_seconds = float(initial_red_seconds)
        self.stages = [('ego_0_green', green_seconds, 0, 'Green'),
                       ('ego_0_yellow', yellow_seconds, 0, 'Yellow'),
                       ('all_red_after_0', clearance_seconds, None, 'Red'),
                       ('ego_1_green', green_seconds, 1, 'Green'),
                       ('ego_1_yellow', yellow_seconds, 1, 'Yellow'),
                       ('all_red_after_1', clearance_seconds, None, 'Red')]
        if self.initial_red_seconds < 0 or any(float(s[1]) <= 0 for s in self.stages):
            raise ValueError('Invalid phase durations')

    def decide(self, elapsed):
        if elapsed < self.initial_red_seconds:
            return {'phase': 'initial_all_red', 'active_ego': None, 'state': 'Red',
                    'remaining_s': self.initial_red_seconds - elapsed}
        cycle = sum(float(s[1]) for s in self.stages)
        offset = (elapsed - self.initial_red_seconds) % cycle
        for name, duration, ego, state in self.stages:
            if offset < duration:
                return {'phase': name, 'active_ego': ego, 'state': state, 'remaining_s': float(duration) - offset}
            offset -= duration
        raise RuntimeError('Phase calculation failed')


class CarlaTrafficLightRuntime:
    def __init__(self, world, routes, output_dir, policy):
        import carla
        self.carla = carla
        self.world = world
        self.map = world.get_map()
        self.policy = policy
        self.output = Path(output_dir)
        self.output.mkdir(parents=True, exist_ok=True)
        self.log = (self.output / 'traffic_signals.jsonl').open('w', buffering=1)
        self.origin = world.get_snapshot().timestamp.elapsed_seconds
        self.last_phase = None
        self.last_elapsed = None
        self.prepared_decision = None
        self.prepared_states = None
        self.signal_apply_lag_frames = 0
        self.progress = [{ 'waiting_s': 0.0, 'entered_s': None, 'exited_s': None, 'start_s': None} for _ in routes]
        self.messages = {}
        self.signal_history = {}
        self.latest_lane_state = None
        self.background_runtime = None
        self.last_lane_log_frame = None
        self.mappings = []
        lights = list(world.get_actors().filter('*traffic_light*'))
        for ego, route in enumerate(routes):
            points = [item[0].location for item in route]
            wps = [self.map.get_waypoint(p) for p in points]
            first_junction = next((i for i,w in enumerate(wps) if w.is_junction), None)
            if first_junction is None:
                raise ValueError('ego %d does not traverse a junction' % ego)
            junction_id = wps[first_junction].get_junction().id
            cumulative = [0.0]
            for a,b in zip(points, points[1:]):
                cumulative.append(cumulative[-1] + a.distance(b))
            candidates = []
            # Use the stop line at the end of the affected approach lane, as
            # used by MDrive autopilot, rather than the trigger-box centre.
            for light in lights:
                for trigger_wp in light.get_stop_waypoints():
                    wp = trigger_wp
                    for _ in range(500):
                        nxt = wp.next(0.5)
                        if not nxt or nxt[0].is_junction:
                            break
                        wp = nxt[0]
                    nxt = wp.next(0.5)
                    if not nxt or not nxt[0].is_junction or nxt[0].get_junction().id != junction_id:
                        continue
                    matches = [i for i,w in enumerate(wps[:first_junction])
                               if w.road_id == wp.road_id and w.lane_id == wp.lane_id]
                    if not matches:
                        continue
                    idx = min(matches, key=lambda i: points[i].distance(wp.transform.location))
                    if points[idx].distance(wp.transform.location) > 4.0:
                        continue
                    candidates.append((cumulative[idx], light, wp, idx))
            if not candidates:
                raise ValueError('No route-matched signal for ego %d' % ego)
            _, light, stop, idx = min(candidates, key=lambda item: item[0])
            exit_wp = next((w for w in wps[first_junction:] if not w.is_junction), None)
            if exit_wp is None:
                raise ValueError('Route must extend beyond the controlled junction')
            delta = (exit_wp.transform.rotation.yaw-stop.transform.rotation.yaw+180)%360-180
            movement = 'RIGHT' if delta>35 else ('LEFT' if delta < -35 else 'STRAIGHT')
            group = {a.id:a for a in light.get_group_traffic_lights()}
            self.mappings.append({'ego':ego,'light':light,'stop':stop,'junction_id':junction_id,
                                  'movement':movement,'points':points,'wps':wps,'cumulative':cumulative,
                                  'stop_index':idx,'stop_arc':project_arc(stop.transform.location,points,cumulative),
                                  'exit_wp':exit_wp,
                                  'group':group})
        if len(self.mappings) not in (1, 2) or len({m['light'].id for m in self.mappings}) != len(self.mappings):
            raise ValueError('This minimal policy requires one or two distinct incoming signals')
        if any(m['junction_id'] != self.mappings[0]['junction_id'] or set(m['group']) != set(self.mappings[0]['group']) for m in self.mappings):
            raise ValueError('Both routes must be controlled by one verified signal group')
        self.group = self.mappings[0]['group']
        self.previous = {i:(a.get_state(),a.is_frozen()) for i,a in self.group.items()}
        for light in self.group.values():
            light.freeze(True)
        manifest = []
        for m in self.mappings:
            p=m['stop'].transform.location
            manifest.append({'ego_index':m['ego'],'traffic_light_id':m['light'].id,'junction_id':m['junction_id'],
                             'road_id':m['stop'].road_id,'lane_id':m['stop'].lane_id,'movement':m['movement'],
                             'stop_line':{'x':p.x,'y':p.y,'z':p.z},'group_ids':sorted(self.group)})
        (self.output/'traffic_light_mapping.json').write_text(json.dumps(manifest,indent=2))
        self.prepare(self.origin)

    def write(self, record):
        self.log.write(json.dumps(record)+'\n')

    def prepare(self, simulation_time):
        elapsed = max(0.0, simulation_time-self.origin)
        observation = self.collect_lane_state()
        self.latest_lane_state = observation
        if hasattr(self.policy, 'update'):
            self.policy.update(elapsed, observation)
        decision = self.policy.decide(elapsed)
        desired = {i:'Red' for i in self.group}
        if decision['active_ego'] is not None and decision['active_ego'] < len(self.mappings):
            desired[self.mappings[decision['active_ego']]['light'].id] = decision['state']
        for i,light in self.group.items():
            light.set_state(getattr(self.carla.TrafficLightState,desired[i]))
        # ``simulation_time`` is the timestamp of the world frame these
        # commands will govern.  Keep that exact decision for post-tick
        # validation.  Recomputing it from floating-point elapsed time can
        # select the following phase on a boundary while CARLA still exposes
        # the state commanded for this frame.
        self.prepared_decision = dict(decision)
        self.prepared_states = dict(desired)
        # get_state() reads the last world snapshot. Validate actual states
        # in observe() after the evaluator's next tick, not immediately here.
        if decision['phase'] != self.last_phase:
            self.write({'type':'phase_change','effective_simulation_time':simulation_time,'elapsed_s':elapsed,
                        'phase':decision['phase'],'commanded_states':desired,
                        'controller':decision.get('controller','fixed'),
                        'target_action':decision.get('target_action'),
                        'forced_by_max_green':decision.get('forced_by_max_green', False),
                        'lane_state':observation,'q_values':decision.get('q_values')})
            self.last_phase = decision['phase']

    def collect_lane_state(self, detection_distance=40.0):
        """Count CARLA ground-truth vehicles on mapped approach/exit lanes."""
        incoming = [0 for _ in self.mappings]
        outgoing = [0 for _ in self.mappings]
        queued = [0 for _ in self.mappings]
        vehicle_ids = [{'incoming': [], 'outgoing': []} for _ in self.mappings]
        for actor in self.world.get_actors().filter('*vehicle*'):
            if not actor.is_alive:
                continue
            location = actor.get_location()
            wp = self.map.get_waypoint(location)
            velocity = actor.get_velocity()
            speed = math.sqrt(velocity.x**2+velocity.y**2+velocity.z**2)
            for index, mapping in enumerate(self.mappings):
                distance = mapping['stop_arc']-project_arc(location,mapping['points'],mapping['cumulative'])
                if (wp.road_id,wp.lane_id) == (mapping['stop'].road_id,mapping['stop'].lane_id) and 0 <= distance <= detection_distance:
                    incoming[index] += 1
                    queued[index] += int(speed < 0.1)
                    vehicle_ids[index]['incoming'].append(actor.id)
                    break
                exit_wp = mapping['exit_wp']
                if (wp.road_id,wp.lane_id) == (exit_wp.road_id,exit_wp.lane_id) and -detection_distance <= distance < 0:
                    outgoing[index] += 1
                    vehicle_ids[index]['outgoing'].append(actor.id)
                    break
        # A turning Traffic Manager actor can leave the junction on a lane
        # adjacent to the ego route's first exit waypoint.  Its lifecycle is
        # still known exactly by BackgroundTrafficRuntime.  During the short
        # post-junction sink window, include that ground-truth movement in the
        # outgoing count unless the geometric counter already saw it.
        runtime = self.background_runtime
        if runtime is not None:
            for demand_id, state in runtime.states.items():
                actor = runtime.actors.get(demand_id)
                if (not state.get('exited') or state.get('removed') or
                        actor is None or not actor.is_alive):
                    continue
                movement = runtime.demand[demand_id]['movement']
                if movement >= len(outgoing):
                    continue
                # Route geometry can associate a turning actor with the
                # other movement's exit lane.  The demand manifest is the
                # authoritative movement label, so move (rather than merely
                # duplicate) an already counted actor to that bucket.
                for index, lane in enumerate(vehicle_ids):
                    if actor.id in lane['outgoing'] and index != movement:
                        lane['outgoing'].remove(actor.id)
                        outgoing[index] -= 1
                if actor.id in vehicle_ids[movement]['outgoing']:
                    continue
                outgoing[movement] += 1
                vehicle_ids[movement]['outgoing'].append(actor.id)
        background_vehicles = []
        if runtime is not None:
            for demand_id, actor in runtime.actors.items():
                state = runtime.states.get(demand_id, {})
                if actor is None or not actor.is_alive or state.get('removed'):
                    continue
                location = actor.get_location()
                velocity = actor.get_velocity()
                speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
                background_vehicles.append({
                    'demand_id': demand_id, 'actor_id': actor.id,
                    'movement': runtime.demand[demand_id]['movement'],
                    'x': location.x, 'y': location.y, 'speed_mps': speed,
                    'stage': ('exited' if state.get('exited') else
                              ('junction' if state.get('entered') else 'approach'))})
        return {'schema':'carla_lane_counts_v1','detection_distance_m':detection_distance,
                'incoming':incoming,'outgoing':outgoing,'queued':queued,
                'pressure':[a-b for a,b in zip(incoming,outgoing)],
                'vehicle_ids':vehicle_ids,
                'background_vehicles':background_vehicles}

    def record_snapshot(self):
        snapshot = self.world.get_snapshot()
        now = snapshot.timestamp.elapsed_seconds
        elapsed = now-self.origin
        decision = (self.prepared_decision or
                    self.policy.decide(max(0, elapsed)))
        states = {i:str(light.get_state()).split('.')[-1] for i,light in self.group.items()}
        desired = self.prepared_states
        if desired is None:
            desired = {i:'Red' for i in self.group}
            if decision['active_ego'] is not None and decision['active_ego'] < len(self.mappings):
                desired[self.mappings[decision['active_ego']]['light'].id] = decision['state']
        if sum(s == 'Green' for s in states.values()) > 1:
            raise RuntimeError('Signal group snapshot contains conflicting green lights')
        if states != desired:
            # CARLA 0.9.12 may expose the previous frozen state for one frame
            # at a phase transition. Re-issue the command and tolerate that
            # bounded apply lag, while still failing a persistent overwrite.
            self.signal_apply_lag_frames += 1
            self.write({'type': 'signal_apply_lag', 'carla_frame': snapshot.frame,
                        'simulation_time': now, 'elapsed_s': elapsed,
                        'commanded_states': desired, 'actual_states': states,
                        'consecutive_frames': self.signal_apply_lag_frames})
            for light_id, light in self.group.items():
                light.set_state(getattr(self.carla.TrafficLightState,
                                        desired[light_id]))
            if self.signal_apply_lag_frames > 2:
                raise RuntimeError(
                    'Signal group state failed to apply for more than two frames')
        else:
            self.signal_apply_lag_frames = 0
        record={'type':'signal_snapshot','carla_frame':snapshot.frame,'simulation_time':now,
                'elapsed_s':elapsed,'phase':decision['phase'],'phase_remaining_s':decision['remaining_s'],'actual_states':states}
        record['commanded_states'] = dict(desired)
        record['signal_apply_lag_frames'] = self.signal_apply_lag_frames
        if snapshot.frame not in self.signal_history:
            self.signal_history[snapshot.frame]=record
            self.write(record)
        if snapshot.frame != self.last_lane_log_frame and self.latest_lane_state is not None:
            self.write({'type':'lane_state','carla_frame':snapshot.frame,'simulation_time':now,
                        'elapsed_s':elapsed,'phase':decision['phase'],
                        'controller':decision.get('controller','fixed'),
                        **self.latest_lane_state})
            self.last_lane_log_frame = snapshot.frame
        return record

    def observe(self, ego_actors, camera_frames=None):
        record=self.record_snapshot()
        now=record['simulation_time']
        elapsed=record['elapsed_s']
        decision=self.policy.decide(max(0,elapsed))
        dt=0 if self.last_elapsed is None else elapsed-self.last_elapsed
        states=record['actual_states']
        snapshot=self.world.get_snapshot()
        messages = {}
        for m,actor in zip(self.mappings,ego_actors):
            if actor is None or not actor.is_alive:
                continue
            state = states[m['light'].id]
            # record_snapshot() already enforces mutual exclusion and rejects
            # a command that remains unapplied for more than two frames.  Do
            # not perform a second zero-latency check here: CARLA 0.9.12 can
            # expose the previous state for one bounded transition frame.
            location = actor.get_location()
            # Project onto the dense route rather than using Euclidean distance.
            distance=m['stop_arc']-project_arc(location,m['points'],m['cumulative'])
            wp=self.map.get_waypoint(location)
            inside=wp.is_junction and wp.get_junction().id==m['junction_id']
            progress=self.progress[m['ego']]
            if progress['start_s'] is None:progress['start_s']=elapsed
            velocity=actor.get_velocity()
            speed=math.sqrt(velocity.x**2+velocity.y**2+velocity.z**2)
            if progress['entered_s'] is None and inside:
                progress['entered_s']=elapsed
                self.write({'type':'junction_enter','ego_index':m['ego'],'vehicle_id':actor.id,'elapsed_s':elapsed})
            if progress['entered_s'] is not None and progress['exited_s'] is None and not inside:
                progress['exited_s']=elapsed
                self.write({'type':'junction_exit','ego_index':m['ego'],'vehicle_id':actor.id,'elapsed_s':elapsed})
            if progress['entered_s'] is None and 0 <= distance <= 40 and speed<0.1 and state in ('Red','Yellow'):
                progress['waiting_s']+=dt
            stage='approach' if progress['entered_s'] is None else ('inside' if inside else 'departed')
            message={'type':'vehicle_signal_message','ego_index':m['ego'],'vehicle_id':actor.id,
                     'carla_frame':snapshot.frame,'simulation_time':now,'elapsed_s':elapsed,
                     'camera_frame':(camera_frames or {}).get(m['ego']),
                     'junction_id':m['junction_id'],'traffic_light_id':m['light'].id,'road_id':m['stop'].road_id,
                     'lane_id':m['stop'].lane_id,'movement':m['movement'],'route_stage':stage,
                     'actual_state':state,'phase':decision['phase'],'phase_remaining_s':decision['remaining_s'],
                     'movement_permitted':state=='Green' if stage=='approach' else None,
                     'distance_to_stop_line_m':distance,'speed_mps':speed,'x':location.x,'y':location.y}
            message['camera_signal']=camera_signal_at_frame(self.signal_history,message['camera_frame'],m['light'].id)
            messages[m['ego']]=message
            self.write(message)
        self.last_elapsed=elapsed
        self.messages=messages
        return messages

    def close(self):
        summary={'throughput_vehicles':sum(p['exited_s'] is not None for p in self.progress),'egos':[]}
        for i,p in enumerate(self.progress):
            item=dict(p,ego_index=i)
            item['observation_to_exit_s']=p['exited_s']-p['start_s'] if p['exited_s'] is not None else None
            item['junction_traversal_s']=p['exited_s']-p['entered_s'] if p['exited_s'] is not None else None
            summary['egos'].append(item)
        (self.output/'traffic_metrics.json').write_text(json.dumps(summary,indent=2))
        self.log.close()
        for i,light in self.group.items():
            if light.is_alive:
                state,frozen=self.previous[i]
                light.set_state(state)
                light.freeze(frozen)
