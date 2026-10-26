"""无 CARLA 服务器的行为测试：验证记录语义，不模拟交通物理。"""

import csv
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

from evaluation.collector import FrameCollector


def vec(x=0, y=0, z=0):
    return NS(x=x, y=y, z=z)


class CollectorTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.wp = NS(road_id=18, section_id=0, lane_id=1, s=5.0,
                     is_junction=False, previous=lambda distance: [])
        self.light = NS(id=999, get_opendrive_id=lambda: "2401", get_state=lambda: "Red",
                        get_pole_index=lambda: 1, is_frozen=lambda: True)
        self.actor = NS(id=123, attributes={"role_name": "ego"}, type_id="vehicle.test",
                        bounding_box=NS(get_world_vertices=lambda transform: [vec(-7), vec(-3)]))
        self.actors = [self.actor]
        self.frame = 10
        self.world = NS(
            get_map=lambda: NS(name="Carla/Maps/Town05", get_waypoint_xodr=lambda *args: self.wp,
                               get_waypoint=lambda *args, **kwargs: self.wp),
            get_traffic_lights_in_junction=lambda junction: [self.light],
            get_settings=lambda: NS(synchronous_mode=True, fixed_delta_seconds=0.05),
            get_actors=lambda: NS(filter=lambda pattern: self.actors),
            get_snapshot=lambda: NS(frame=self.frame))
        self.inventory = self.root / "inventory.json"
        self.inventory.write_text(json.dumps(dict(map_name="Carla/Maps/Town05", junction_id=1427,
            traffic_lights=[dict(actor_id=82, opendrive_id="2401", stop_waypoints=[dict(
                road_id=18, section_id=0, lane_id=1, s=5, location=dict(x=0, y=0, z=0), yaw=0)])])), encoding="utf-8")

    def snapshot(self, frame):
        state = NS(get_transform=lambda: NS(location=vec(-5), rotation=NS(pitch=0, yaw=0, roll=0)),
                   get_velocity=lambda: vec(3, 4), get_acceleration=lambda: vec())
        return NS(frame=frame, timestamp=NS(elapsed_seconds=frame * .05, delta_seconds=.05),
                  find=lambda actor_id: state)

    def collector(self):
        collector = FrameCollector(self.world, inventory=self.inventory, output_root=self.root)
        self.addCleanup(collector.close)
        return collector

    def rows(self, collector, name):
        with (collector.output / (name + ".csv")).open(encoding="utf-8-sig", newline="") as handle:
            return list(csv.DictReader(handle))

    def test_snapshot_geometry_runtime_ids_and_flush(self):
        collector = self.collector()
        collector.sample(self.snapshot(10))
        # close 前即可读取，证明不是退出时才落盘；旧 actor 82 应重映射为 999。
        row = self.rows(collector, "vehicles")[0]
        self.assertEqual(row["signal_actor_id"], "999")
        self.assertEqual(float(row["speed_mps"]), 5)
        self.assertEqual(float(row["stop_plane_front_distance_m"]), 3)
        self.assertEqual(row["road_id"], "18")
        # 离开已知道路后应清空匹配，不能继续使用生成时的车道或信号。
        self.wp.road_id = 77
        self.frame = 11
        collector.sample(self.snapshot(11))
        row = self.rows(collector, "vehicles")[1]
        self.assertEqual(row["road_id"], "77")
        self.assertEqual(row["signal_match"], "unmapped")
        self.assertEqual(row["signal_state"], "")

    def test_empty_frame_gap_overlap_and_duplicate(self):
        collector = self.collector()
        self.actors = []
        collector.sample(self.snapshot(10))
        self.frame = 14
        collector.sample(self.snapshot(13))
        rows = self.rows(collector, "frames")
        self.assertEqual(rows[0]["vehicle_count"], "0")
        self.assertEqual(rows[1]["frame_gap"], "2")
        self.assertEqual(rows[1]["read_overlapped_tick"], "True")
        with self.assertRaises(RuntimeError):
            collector.sample(self.snapshot(13))
        collector.close("interrupted")
        metadata = json.loads((collector.output / "metadata.json").read_text(encoding="utf-8"))
        self.assertEqual(metadata["missing_frames"], 2)
        self.assertEqual(metadata["status"], "interrupted")

    def test_ambiguous_mapping_does_not_invent_signal(self):
        collector = self.collector()
        collector.lane_signals[(18, 0, 1)].add("other_signal")
        collector.sample(self.snapshot(10))
        row = self.rows(collector, "vehicles")[0]
        self.assertEqual(row["signal_match"], "ambiguous")
        self.assertEqual(row["signal_actor_id"], "")

    def test_wrong_map_rejected_before_output(self):
        data = json.loads(self.inventory.read_text(encoding="utf-8"))
        data["map_name"] = "Town03"
        self.inventory.write_text(json.dumps(data), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "map"):
            self.collector()

    def test_mixed_vehicle_geometry_and_returned_frame(self):
        collector = self.collector()
        first = collector.sample(self.snapshot(10))
        self.assertEqual(first["frame"], 10)
        self.assertEqual(first["vehicles"][0]["stop_plane_front_distance_m"], 3)
        self.actor.type_id = "vehicle.long_bus"
        self.actor.bounding_box.get_world_vertices = lambda transform: [vec(-10), vec(1)]
        self.frame = 11
        second = collector.sample(self.snapshot(11))
        self.assertEqual(second["vehicles"][0]["stop_plane_front_distance_m"], -1)
        stored = self.rows(collector, "vehicles")[1]
        self.assertEqual(json.loads(stored["bbox_vertices_world_json"]), [[-10,0,0],[1,0,0]])

    def test_geometry_probe_excluded(self):
        collector = self.collector()
        self.actor.attributes["role_name"] = "junction_geometry_probe"
        packet = collector.sample(self.snapshot(10))
        self.assertEqual(packet["vehicles"], [])
        self.assertEqual(self.rows(collector, "frames")[0]["vehicle_count"], "0")

    def test_phase_and_controls_source_labels(self):
        self.actor.get_control=lambda:NS(throttle=.2,brake=0,steer=-.1,reverse=False,hand_brake=False)
        collector=self.collector()
        packet=collector.sample(self.snapshot(10),dict(source="fixed_schedule",phase_id="2401:GREEN",
            remaining_seconds=3,next_phase_id="2401:YELLOW"))
        self.assertEqual(packet["phase"]["remaining_seconds"],3)
        self.assertEqual(self.rows(collector,"phases")[0]["next_phase_id"],"2401:YELLOW")
        row=self.rows(collector,"vehicles")[0]
        self.assertEqual(row["control_source"],"actor_cache_between_ticks")
        self.assertEqual(float(row["throttle"]),.2)


if __name__ == "__main__":
    unittest.main()
