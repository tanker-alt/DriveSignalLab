"""Deterministic, route-bound CARLA background traffic for the test junction."""
import json
import math
import random
from pathlib import Path


PROFILE_COUNTS = {'none': 0, 'low': 2, 'medium': 5, 'high': 10}


def build_demand(profile, seed, movement_count=2, spawn_distance_m=12.0,
                 headway_s=6.0, count_override=None):
    if profile not in PROFILE_COUNTS:
        raise ValueError('background profile must be one of %s' % sorted(PROFILE_COUNTS))
    count = PROFILE_COUNTS[profile] if count_override is None else int(count_override)
    if count < 0:
        raise ValueError('background vehicles per movement must be nonnegative')
    rng = random.Random(int(seed))
    # One compact blueprint avoids spawn-box bias: a longer random model can
    # be rejected at the same valid injection point and change actual demand.
    model = 'vehicle.nissan.micra'
    demand = []
    # Interleave movements in the manifest; physical spawn positions remain
    # fixed behind the corresponding ego and do not depend on controller state.
    for rank in range(count):
        order = list(range(movement_count))
        rng.shuffle(order)
        for movement in order:
            demand.append({'demand_id': len(demand), 'movement': movement,
                           'rank': rank + 1, 'scheduled_s': rank * float(headway_s),
                           'distance_behind_ego_m': float(spawn_distance_m),
                           'model': model})
    return demand


class BackgroundTrafficRuntime:
    """Spawn a reproducible queue on each mapped approach and audit its life cycle."""
    def __init__(self, world, mappings, output_dir, profile='none', seed=2000,
                 count_override=None, spawn_distance_m=12.0, headway_s=6.0,
                 speed_difference=0.0, strict_routes=False):
        self.strict_routes = bool(strict_routes)
        self.route_agents = {}
        self.assigned_route_points = {}
        self.world = world
        self.map = world.get_map()
        self.output = Path(output_dir)
        self.profile = profile
        self.seed = int(seed)
        self.actors = {}
        self.states = {}
        self.log = (self.output / 'background_traffic.jsonl').open('w', buffering=1)
        self.demand = build_demand(profile, seed, len(mappings), spawn_distance_m,
                                   headway_s, count_override)
        self.mappings = mappings
        self.speed_difference = float(speed_difference)
        self.pending = {item['demand_id']: item for item in self.demand}
        self.last_attempt = {}
        self.tm_port = self._traffic_manager_port()
        self.tm = self._traffic_manager()
        plan = {'schema': 'carla_route_background_v1', 'profile': profile, 'seed': self.seed,
                'vehicles_per_movement': len(self.demand) // max(1, len(mappings)),
                'total_vehicles': len(self.demand),
                'routing_mode': 'BasicAgent_exact_waypoints' if self.strict_routes else 'TrafficManager',
                'spawn_distance_m': float(spawn_distance_m), 'headway_s': float(headway_s),
                'traffic_manager_port': self.tm_port, 'demand': self.demand}
        (self.output / 'background_traffic_plan.json').write_text(json.dumps(plan, indent=2))
        self._write({'type': 'background_plan', **plan})

    def _traffic_manager_port(self):
        from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
        return int(CarlaDataProvider.get_traffic_manager_port())

    def _traffic_manager(self):
        if self.strict_routes:
            return None
        from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
        return CarlaDataProvider._client.get_trafficmanager(self.tm_port)

    def _write(self, record):
        record.setdefault('carla_frame', self.world.get_snapshot().frame)
        record.setdefault('simulation_time', self.world.get_snapshot().timestamp.elapsed_seconds)
        self.log.write(json.dumps(record) + '\n')

    @staticmethod
    def _previous(wp, distance):
        # Never cross an upstream junction: TM may turn there before reaching
        # the controlled intersection. Short approaches inject closer to the ego
        # start and retry when occupied, rather than changing the demand route.
        start = wp
        travelled = 0.0
        while travelled + 0.5 <= float(distance) + 1e-6:
            candidates = [item for item in wp.previous(0.5)
                          if not item.is_junction and
                          (item.road_id, item.lane_id) == (start.road_id, start.lane_id)]
            if len(candidates) != 1:
                break
            wp = candidates[0]
            travelled += 0.5
        return wp

    def prepare(self, elapsed_s):
        """Try each due demand at most once per simulated second until admitted."""
        elapsed_s = float(elapsed_s)
        for demand_id, item in list(self.pending.items()):
            if elapsed_s + 1e-6 < item['scheduled_s']:
                continue
            if elapsed_s - self.last_attempt.get(demand_id, -1e9) < 1.0:
                continue
            self.last_attempt[demand_id] = elapsed_s
            if self._spawn(item, elapsed_s):
                del self.pending[demand_id]

        for demand_id, agent in self.route_agents.items():
            actor = self.actors.get(demand_id)
            if actor is not None and actor.is_alive and not self.states[demand_id].get('removed'):
                actor.apply_control(agent.run_step())

    def _spawn(self, item, elapsed_s):
        blueprints = self.world.get_blueprint_library()
        mapping = self.mappings[item['movement']]
        start_wp = self.map.get_waypoint(mapping['points'][0])
        spawn_wp = self._previous(start_wp, item['distance_behind_ego_m'])
        transform = spawn_wp.transform
        transform.location.z += 0.25
        choices = list(blueprints.filter(item['model']))
        if not choices:
            raise RuntimeError('Missing CARLA blueprint %s' % item['model'])
        blueprint = choices[0]
        if blueprint.has_attribute('role_name'):
            blueprint.set_attribute('role_name', 'background_%03d' % item['demand_id'])
        actor = self.world.try_spawn_actor(blueprint, transform)
        if actor is None:
            self._write({'type': 'background_spawn_deferred', **item, 'elapsed_s': elapsed_s,
                         'x': transform.location.x, 'y': transform.location.y})
            return False
        if self.strict_routes:
            from agents.navigation.basic_agent import BasicAgent
            from agents.navigation.local_planner import RoadOption
            # New actors are manual by default; do not create a default-port TM.
            agent = BasicAgent(actor, target_speed=18)
            # Give background demand its own outlet destination, before the
            # next intersection's signals. The VLA route may extend farther.
            route_points = list(mapping['points'])
            exit_location = mapping['exit_wp'].transform.location
            exit_index = min(range(len(route_points)), key=lambda i:
                math.hypot(route_points[i].x-exit_location.x, route_points[i].y-exit_location.y))
            end_index, outlet_distance = exit_index, 0.0
            for i in range(exit_index+1, len(route_points)):
                point, previous = route_points[i], route_points[i-1]
                distance = math.hypot(point.x-previous.x, point.y-previous.y)
                if outlet_distance+distance > 20.0 or self.map.get_waypoint(point).is_junction:
                    break
                outlet_distance += distance
                end_index = i
            if outlet_distance < 5.0:
                raise RuntimeError('Background outlet is too short for a destination')
            route_points = route_points[:end_index+1]
            waypoints = [self.map.get_waypoint(point) for point in route_points]
            agent.set_global_plan([(wp, RoadOption.STRAIGHT if wp.is_junction else RoadOption.LANEFOLLOW)
                                   for wp in waypoints])
            self.route_agents[item['demand_id']] = agent
            self.assigned_route_points[item['demand_id']] = [transform.location] + route_points
            self._write(dict(type='vehicle_route_association', actor_id=actor.id,
                demand_id=item['demand_id'], movement=item['movement'], maneuver='STRAIGHT',
                traffic_light_id=mapping['light'].id, junction_id=mapping['junction_id'],
                incoming_road_id=mapping['stop'].road_id, incoming_lane_id=mapping['stop'].lane_id,
                exit_road_id=mapping['exit_wp'].road_id, exit_lane_id=mapping['exit_wp'].lane_id,
                stop_line_arc_m=mapping['stop_arc'], outlet_length_m=outlet_distance,
                route_points=[dict(x=p.x,y=p.y,z=p.z) for p in self.assigned_route_points[item['demand_id']]]))
        else:
            actor.set_autopilot(True, self.tm_port)
            self.tm.auto_lane_change(actor, False)
            self.tm.ignore_lights_percentage(actor, 0.0)
            self.tm.ignore_vehicles_percentage(actor, 0.0)
            self.tm.vehicle_percentage_speed_difference(actor, self.speed_difference)
            if hasattr(self.tm, 'set_path'):
                self.tm.set_path(actor, [point.location for point in mapping['points'][1:]])
        self.actors[item['demand_id']] = actor
        self.states[item['demand_id']] = {'seen_approach': False, 'entered': False,
                                           'exited': False, 'waiting_s': 0.0,
                                           'last_time': None, 'actual_spawn_s': elapsed_s}
        self._write({'type': 'background_spawned', 'actor_id': actor.id, **item,
                     'elapsed_s': elapsed_s, 'x': transform.location.x, 'y': transform.location.y,
                     'road_id': spawn_wp.road_id, 'lane_id': spawn_wp.lane_id})
        return True

    def observe(self, lane_state, elapsed_s):
        incoming_ids = [set(item['incoming']) for item in lane_state['vehicle_ids']]
        for demand_id, actor in self.actors.items():
            state = self.states[demand_id]
            now = float(elapsed_s)
            if state['exited']:
                if self.strict_routes and not state.get('removed'):
                    if not actor.is_alive:
                        raise RuntimeError('Route-bound background disappeared before its destination')
                    location = actor.get_location()
                    end = self.assigned_route_points[demand_id][-1]
                    distance = math.hypot(location.x-end.x, location.y-end.y)
                    if self.route_agents[demand_id].done() and distance <= 4.0:
                        state['route_finished'] = True
                        state['removed'] = True
                        self._write(dict(type='background_route_completed', demand_id=demand_id,
                            actor_id=actor.id, movement=self.demand[demand_id]['movement'],
                            distance_to_goal_m=distance, goal_tolerance_m=4.0,
                            x=location.x,y=location.y, elapsed_s=now))
                        actor.destroy()
                    continue
                if (not state.get('removed') and
                        now >= state.get('sink_at_s', float('inf'))):
                    if actor.is_alive:
                        actor.destroy()
                    state['removed'] = True
                    self._write({'type': 'background_removed_at_sink',
                                 'demand_id': demand_id, 'actor_id': actor.id,
                                 'movement': self.demand[demand_id]['movement'],
                                 'elapsed_s': now})
                continue
            if not actor.is_alive:
                continue
            item = self.demand[demand_id]
            movement = item['movement']
            dt = 0.0 if state['last_time'] is None else max(0.0, now-state['last_time'])
            state['last_time'] = now
            loc = actor.get_location()
            wp = self.map.get_waypoint(loc)
            velocity = actor.get_velocity()
            speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            inside = wp.is_junction and wp.get_junction().id == self.mappings[movement]['junction_id']
            if actor.id in incoming_ids[movement] and not state['seen_approach']:
                state['seen_approach'] = True
                self._write({'type': 'background_approach_enter', 'demand_id': demand_id,
                             'actor_id': actor.id, 'movement': movement, 'elapsed_s': now})
            if state['seen_approach'] and not state['entered'] and speed < 0.1:
                state['waiting_s'] += dt
            if inside and not state['entered']:
                state['entered'] = True
                self._write({'type': 'background_junction_enter', 'demand_id': demand_id,
                             'actor_id': actor.id, 'movement': movement, 'elapsed_s': now})
            if self.strict_routes:
                from common.grouped_fixed_signal import point_segment_distance
                mapping = self.mappings[movement]
                lateral = min(point_segment_distance((loc.x,loc.y),(a.x,a.y),(b.x,b.y))
                              for a,b in zip(self.assigned_route_points[demand_id],self.assigned_route_points[demand_id][1:]))
                native = actor.get_traffic_light() if actor.is_at_traffic_light() else None
                association_ok = (native is None or native.id == mapping['light'].id or inside)
                self._write(dict(type='route_validation', actor_id=actor.id, demand_id=demand_id,
                    movement=movement, lateral_error_m=lateral, inside=inside,
                    expected_light_id=mapping['light'].id,
                    native_light_id=native.id if native else None, association_ok=association_ok,
                    actual_light_state=str(mapping['light'].get_state()),
                    road_id=wp.road_id,lane_id=wp.lane_id))
                if lateral > 1.5 or not association_ok:
                    raise RuntimeError('Background route or controlling-signal mismatch for %s' % actor.id)
            if state['entered'] and not inside:
                if self.strict_routes and (wp.road_id,wp.lane_id) != (self.mappings[movement]['exit_wp'].road_id,self.mappings[movement]['exit_wp'].lane_id):
                    raise RuntimeError('Background vehicle used the wrong junction exit')
                state['exited'] = True
                state['sink_at_s'] = now + 1.0
                state['removed'] = False
                self._write({'type': 'background_junction_exit', 'demand_id': demand_id,
                             'actor_id': actor.id, 'movement': movement, 'elapsed_s': now,
                             'waiting_s': state['waiting_s']})

    def status(self):
        planned = len(self.demand)
        spawned = len(self.actors)
        exited = sum(x['exited'] for x in self.states.values())
        return {'enabled': planned > 0, 'profile': self.profile, 'planned': planned,
                'spawned': spawned, 'pending': len(self.pending), 'exited': exited,
                'route_finished': sum(x.get('route_finished', False) for x in self.states.values()),
                'drained': planned == spawned == exited and not self.pending and
                (not getattr(self, 'strict_routes', False) or
                 all(x.get('route_finished', False) for x in self.states.values()))}

    def close(self):
        summary = {'schema': 'carla_route_background_metrics_v1', 'profile': self.profile,
                   'seed': self.seed, 'planned': len(self.demand), 'spawned': len(self.actors),
                   'pending_at_end': len(self.pending),
                   'approach_seen': sum(x['seen_approach'] for x in self.states.values()),
                   'junction_entered': sum(x['entered'] for x in self.states.values()),
                   'junction_exited': sum(x['exited'] for x in self.states.values()),
                   'route_finished': sum(x.get('route_finished', False) for x in self.states.values()),
                   'strict_routes': self.strict_routes,
                   'vehicles': []}
        for demand_id, state in sorted(self.states.items()):
            summary['vehicles'].append({'demand_id': demand_id,
                                        'actor_id': self.actors[demand_id].id,
                                        **self.demand[demand_id], **state})
        (self.output / 'background_traffic_metrics.json').write_text(json.dumps(summary, indent=2))
        self._write({'type': 'background_summary', **summary})
        # Leaderboard tears down world actors before calling agent.destroy().
        # Touching CARLA actor proxies here can abort in native code because
        # their server-side actors are already gone. World cleanup owns them.
        self.log.close()
        return summary
