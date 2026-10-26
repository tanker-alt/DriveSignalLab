"""记录 CARLA 碰撞传感器原始回调和覆盖范围，不把回调条数当作事故次数。"""

import json
import math
import threading
try:
    from .collision_events import validate_gap
except ImportError:
    from collision_events import validate_gap


class CollisionRecorder:
    def __init__(self, world, output, merge_gap_seconds=1.0):
        validate_gap(merge_gap_seconds)
        self.merge_gap_seconds=merge_gap_seconds
        self.world = world
        self.sensors = {}
        self.attachment_frames = {}
        self.lock = threading.Lock()
        self.closed = False
        self.output = output
        self.last_frame = None
        self.last_time = None
        self.observed = set()
        self.ended = set()
        self.file = (output / "collisions.jsonl").open("w", encoding="utf-8")

    def write(self, record):
        # 传感器线程与仿真主线程可能同时写入，必须锁住完整的一行。
        with self.lock:
            if not self.closed:
                self.file.write(json.dumps(record, ensure_ascii=False) + "\n")
                self.file.flush()

    def attach(self, vehicle, frame):
        """首次观测才安装，之前的碰撞未知；记录覆盖起点而非声称全程无碰撞。"""
        if vehicle.id in self.sensors:
            return
        import carla
        try:
            sensor = self.world.spawn_actor(self.world.get_blueprint_library().find("sensor.other.collision"),
                                            carla.Transform(), attach_to=vehicle)
        except RuntimeError as exc:
            self.write(dict(event='sensor_attach_failed',vehicle_id=vehicle.id,frame=frame,error=str(exc)))
            raise
        self.sensors[vehicle.id] = sensor
        self.attachment_frames[vehicle.id] = int(frame)
        vehicle_id = vehicle.id

        def on_collision(event):
            impulse = event.normal_impulse
            location=event.transform.location
            self.write(dict(event="collision_callback", frame=event.frame, sim_time=event.timestamp,
                vehicle_id=vehicle_id, other_actor_id=event.other_actor.id,
                vehicle_type=vehicle.type_id,sensor_id=sensor.id,
                other_actor_type=event.other_actor.type_id,
                sensor_location=dict(x=location.x,y=location.y,z=location.z),
                impulse_x=impulse.x, impulse_y=impulse.y, impulse_z=impulse.z,
                impulse_magnitude=math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)))
        try:
            sensor.listen(on_collision)
        except RuntimeError as exc:
            self.write(dict(event='sensor_listen_failed',vehicle_id=vehicle_id,frame=frame,error=str(exc)))
            raise
        self.write(dict(event="sensor_attached", vehicle_id=vehicle_id, sensor_id=sensor.id,frame=frame,
                        earliest_eligible_frame=int(frame)+1,
                        note="Coverage starts after attachment; earlier events unknown"))

    def observe(self, snapshot, vehicle_ids):
        """Coverage audit only: callbacks have no per-frame completion barrier."""
        self.last_frame=snapshot.frame
        self.last_time=snapshot.timestamp.elapsed_seconds
        current=set(vehicle_ids)
        for vid in current-self.observed:
            self.write(dict(event='scope_enter',vehicle_id=vid,frame=self.last_frame,sim_time=self.last_time))
        for vid in self.observed-current:
            self.write(dict(event='scope_exit',vehicle_id=vid,frame=self.last_frame,sim_time=self.last_time))
        self.observed=current
        alive=[]
        for vid,sensor in self.sensors.items():
            if vid in self.ended:
                continue
            # Newly spawned actors are absent from the current synchronous snapshot.
            # Do not treat that expected pre-eligibility state as permanent failure.
            if snapshot.frame <= self.attachment_frames[vid]:
                continue
            reason=None
            if snapshot.find(vid) is None:
                reason='vehicle_absent_from_world_snapshot'
            else:
                listening = sensor.is_listening
                listening = listening() if callable(listening) else listening
                if not sensor.is_alive or not listening:
                    reason='sensor_not_listening_or_destroyed'
            if reason:
                self.ended.add(vid)
                self.write(dict(event='coverage_ended',vehicle_id=vid,frame=self.last_frame,
                                sim_time=self.last_time,reason=reason))
            else:
                alive.append(vid)
        self.write(dict(event='coverage_frame',frame=self.last_frame,sim_time=self.last_time,
                        observed_vehicle_ids=sorted(current),listening_vehicle_ids=sorted(alive),
                        note='Listener health is not proof that all callbacks were delivered'))

    def close(self):
        if self.closed:
            return
        # 先停止监听，再关文件。CARLA 没有空碰撞帧的回调屏障，关闭时晚到消息
        # 不能保证完整接收，明确记录该限制，而不是用任意 sleep 声称同步完成。
        for vehicle_id, sensor in self.sensors.items():
            if vehicle_id not in self.ended:
                self.write(dict(event='coverage_ended',vehicle_id=vehicle_id,frame=self.last_frame,
                                sim_time=self.last_time,reason='recorder_closed'))
            for action in (sensor.stop, sensor.destroy):
                try:
                    action()
                except RuntimeError as exc:
                    self.write(dict(event="sensor_cleanup_error", vehicle_id=vehicle_id, error=str(exc)))
        self.write(dict(event="recording_closed",frame=self.last_frame,sim_time=self.last_time,
                        note="Late in-flight callbacks may be absent"))
        with self.lock:
            self.closed = True
            self.file.close()
