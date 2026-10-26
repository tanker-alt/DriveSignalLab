import csv
import json
import tempfile
import unittest
from pathlib import Path

from evaluation.summarize import replay


class ReplayScopeTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.run = Path(self.temp.name)
        (self.run/"metadata.json").write_text(json.dumps(dict(map="TestMap",junction_id=1)))
        self.manifest = self.run/"fleet.json"
        self.manifest.write_text(json.dumps(dict(map_name="TestMap",junction_id=1,
            vehicles=[dict(vehicle_id=7,release_frame=3)])))
        with (self.run/"frames.csv").open("w",newline="") as handle:
            writer=csv.DictWriter(handle,fieldnames=["frame","sim_time","vehicle_count"])
            writer.writeheader()
            for frame in range(1,7):
                writer.writerow(dict(frame=frame,sim_time=frame,vehicle_count=1))
        with (self.run/"vehicles.csv").open("w",newline="") as handle:
            fields=["frame","sim_time","vehicle_id","vx","vy","signal_match","current_junction_id"]
            writer=csv.DictWriter(handle,fieldnames=fields)
            writer.writeheader()
            for frame in range(1,7):
                writer.writerow(dict(frame=frame,sim_time=frame,vehicle_id=7,vx=0,vy=0,
                                     signal_match="unique",current_junction_id=""))

    def test_setup_wait_excluded_and_scope_saved(self):
        full=replay(self.run)
        scoped=replay(self.run,spawn_manifest=self.manifest)
        summary=json.loads((scoped/"summary.json").read_text())
        self.assertEqual(summary["visits"][0]["first_observed_time"],4)
        self.assertEqual(summary["total_observed_approach_wait_s"],1)
        original=json.loads((full/"summary.json").read_text())
        self.assertEqual(original["total_observed_approach_wait_s"],4)
        self.assertEqual(json.loads((scoped/"replay_scope.json").read_text())["start_frame"],4)

    def test_unreleased_fleet_rejected(self):
        self.manifest.write_text(json.dumps(dict(map_name="TestMap",junction_id=1,
                                                vehicles=[dict(vehicle_id=7)])))
        with self.assertRaisesRegex(ValueError,"not fully released"):
            replay(self.run,spawn_manifest=self.manifest)

    def test_scope_without_frames_not_reported_as_success(self):
        self.manifest.write_text(json.dumps(dict(map_name="TestMap",junction_id=1,
                                                vehicles=[dict(vehicle_id=7,release_frame=100)])))
        with self.assertRaisesRegex(ValueError,"No recorded frames"):
            replay(self.run,spawn_manifest=self.manifest)
