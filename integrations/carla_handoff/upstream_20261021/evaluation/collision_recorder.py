"""记录 CARLA 碰撞传感器原始回调和覆盖范围，不把回调条数当作事故次数。"""

import json
import math
import threading


class CollisionRecorder:
    def __init__(self, world, output):
        self.world = world
        self.sensors = {}
        self.lock = threading.Lock()
        self.closed = False
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
        sensor = self.world.spawn_actor(self.world.get_blueprint_library().find("sensor.other.collision"),
                                        carla.Transform(), attach_to=vehicle)
        self.sensors[vehicle.id] = sensor
        vehicle_id = vehicle.id

        def on_collision(event):
            impulse = event.normal_impulse
            self.write(dict(event="collision_callback", frame=event.frame, sim_time=event.timestamp,
                vehicle_id=vehicle_id, other_actor_id=event.other_actor.id,
                other_actor_type=event.other_actor.type_id,
                impulse_x=impulse.x, impulse_y=impulse.y, impulse_z=impulse.z,
                impulse_magnitude=math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)))
        sensor.listen(on_collision)
        self.write(dict(event="sensor_attached", vehicle_id=vehicle_id, frame=frame,
                        note="Coverage starts after attachment; earlier events unknown"))

    def close(self):
        # 先停止监听，再关文件。CARLA 没有空碰撞帧的回调屏障，关闭时晚到消息
        # 不能保证完整接收，明确记录该限制，而不是用任意 sleep 声称同步完成。
        for vehicle_id, sensor in self.sensors.items():
            for action in (sensor.stop, sensor.destroy):
                try:
                    action()
                except RuntimeError as exc:
                    self.write(dict(event="sensor_cleanup_error", vehicle_id=vehicle_id, error=str(exc)))
        self.write(dict(event="recording_closed", note="Late in-flight callbacks may be absent"))
        with self.lock:
            self.closed = True
            self.file.close()
