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
                 speed_difference=0.0):
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
                'spawn_distance_m': float(spawn_distance_m), 'headway_s': float(headway_s),
                'traffic_manager_port': self.tm_port, 'demand': self.demand}
        (self.output / 'background_traffic_plan.json').write_text(json.dumps(plan, indent=2))
        self._write({'type': 'background_plan', **plan})

    def _traffic_manager_port(self):
        from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
        return int(CarlaDataProvider.get_traffic_manager_port())

    def _traffic_manager(self):
        from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
        return CarlaDataProvider._client.get_trafficmanager(self.tm_port)

    def _write(self, record):
        record.setdefault('carla_frame', self.world.get_snapshot().frame)
        record.setdefault('simulation_time', self.world.get_snapshot().timestamp.elapsed_seconds)
        self.log.write(json.dumps(record) + '\n')

    @staticmethod
    def _previous(wp, distance):
        candidates = wp.previous(float(distance))
        if not candidates:
            raise RuntimeError('No waypoint %.1fm behind ego start' % distance)
        # Stay on the same road/lane when CARLA returns multiple predecessors.
        same = [item for item in candidates
                if (item.road_id, item.lane_id) == (wp.road_id, wp.lane_id)]
        return (same or candidates)[0]

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
            if state['entered'] and not inside:
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
                'drained': planned == spawned == exited and not self.pending}

    def close(self):
        summary = {'schema': 'carla_route_background_metrics_v1', 'profile': self.profile,
                   'seed': self.seed, 'planned': len(self.demand), 'spawned': len(self.actors),
                   'pending_at_end': len(self.pending),
                   'approach_seen': sum(x['seen_approach'] for x in self.states.values()),
                   'junction_entered': sum(x['entered'] for x in self.states.values()),
                   'junction_exited': sum(x['exited'] for x in self.states.values()),
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
