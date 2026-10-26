"""Explicit composition for existing demos; collection and evaluation remain separate."""
from .collector import FrameCollector, DEFAULT_INVENTORY, DEFAULT_OUTPUT
from .evaluator import MetricEvaluator


class EvaluatedCollector:
    def __init__(self, world, *args, vehicle_ids=(), routes=None, collision_accident_config=None, strict_collision_coverage=False, **kwargs):
        self.strict_collision_coverage = strict_collision_coverage
        self.collector = FrameCollector(world, *args, **kwargs)
        try:
            self.metrics = MetricEvaluator(self.collector.output, self.collector.junction_id,
                self.collector.detection_config, self.collector.approach_geometry,
                safety_config=kwargs.get('safety_config'), vehicle_ids=vehicle_ids,
                road_map=world.get_map(), routes=routes,
                collision_accident_config=collision_accident_config,
                collision_merge_gap_seconds=(kwargs.get('collision_config') or {}).get('merge_gap_seconds',1.0))
        except BaseException:
            self.collector.close('failed')
            raise

    def __getattr__(self, name):
        return getattr(self.collector, name)

    @property
    def evaluation(self):
        return self.metrics.recorder

    def prepare_collision_tick(self, spawned_vehicles=()):
        """Call after spawning and before world.tick(), with a single synchronous owner.

        Registers all vehicles, including upstream ones, before their first physical
        step. Does not tick or control vehicles; spawning after this call is forbidden.
        """
        world = self.collector.world
        recorder = self.collector.collision_recorder
        if recorder is None:
            raise RuntimeError('Collision recording must be enabled')
        if not world.get_settings().synchronous_mode:
            raise RuntimeError('Pre-tick collision registration requires synchronous mode')
        frame = world.get_snapshot().frame
        ids = []
        vehicles = {a.id: a for a in world.get_actors().filter('vehicle.*')}
        vehicles.update({a.id: a for a in spawned_vehicles})
        for actor in vehicles.values():
            if actor.attributes.get('role_name') == 'junction_geometry_probe':
                continue
            recorder.attach(actor, frame)
            sensor = recorder.sensors[actor.id]
            listening = sensor.is_listening
            if not (listening() if callable(listening) else listening):
                raise RuntimeError('Collision sensor is not ready: '+str(actor.id))
            ids.append(actor.id)
        if world.get_snapshot().frame != frame:
            raise RuntimeError('World advanced during collision registration')
        return ids

    def _check_collision_coverage(self, snapshot):
        recorder = self.collector.collision_recorder
        if recorder is None:
            raise RuntimeError('Strict collision coverage requires collision recording')
        for actor in self.collector.world.get_actors().filter('vehicle.*'):
            if actor.attributes.get('role_name') == 'junction_geometry_probe' or snapshot.find(actor.id) is None:
                continue
            loc = snapshot.find(actor.id).get_transform().location
            if (loc.x-self.collector.center[0])**2 + (loc.y-self.collector.center[1])**2 > self.collector.radius**2:
                continue
            attached = recorder.attachment_frames.get(actor.id)
            sensor = recorder.sensors.get(actor.id)
            listening = getattr(sensor, 'is_listening', False)
            if (attached is None or attached >= snapshot.frame or actor.id in recorder.ended or
                    not getattr(sensor, 'is_alive', False) or not (listening() if callable(listening) else listening)):
                raise RuntimeError('Uncovered collision vehicle/frame: {}/{}; call prepare_collision_tick before tick'.format(actor.id, snapshot.frame))

    def sample(self, snapshot, phase=None, planner_targets=None):
        if self.strict_collision_coverage:
            self._check_collision_coverage(snapshot)
        packet = self.collector.sample(snapshot, phase, planner_targets)
        self.metrics.sample(packet)
        return packet

    def sample_observation(self, snapshot, planner_targets=None):
        self.sample(snapshot, planner_targets=planner_targets)
        return self.collector.last_observation

    def close(self, status='completed'):
        try:
            self.collector.close(status)
        finally:
            self.metrics.close(status)
