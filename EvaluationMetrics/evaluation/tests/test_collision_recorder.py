import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
from evaluation.collision_recorder import CollisionRecorder


class CollisionRecorderTest(unittest.TestCase):
    def setUp(self):
        stub = patch.dict("sys.modules", {"carla": NS(Transform=lambda: None)})
        stub.start(); self.addCleanup(stub.stop)
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.path=Path(self.temp.name)
        self.callback=None
        def listen(callback):
            self.callback=callback
        self.sensor=NS(id=7,is_alive=True,is_listening=True,listen=listen,
                       stop=lambda:None,destroy=lambda:None)
        self.world=NS(get_blueprint_library=lambda:NS(find=lambda name:name),spawn_actor=lambda *a,**kw:self.sensor)
        self.vehicle=NS(id=1,type_id='vehicle.test')

    def test_callback_and_coverage_end_are_written_and_close_is_idempotent(self):
        recorder=CollisionRecorder(self.world,self.path)
        recorder.attach(self.vehicle,10)
        event=NS(frame=11,timestamp=.55,normal_impulse=NS(x=3,y=4,z=0),
                 transform=NS(location=NS(x=1,y=2,z=3)),other_actor=NS(id=2,type_id='vehicle.other'))
        self.callback(event)
        snapshot=NS(frame=11,timestamp=NS(elapsed_seconds=.55),find=lambda vid:object())
        recorder.observe(snapshot,[1])
        self.sensor.is_listening=False;snapshot.frame=12
        recorder.observe(snapshot,[1]);recorder.close();recorder.close()
        records=[json.loads(line) for line in (self.path/'collisions.jsonl').read_text().splitlines()]
        raw=next(r for r in records if r['event']=='collision_callback')
        self.assertEqual(raw['impulse_magnitude'],5)
        self.assertEqual(raw['sensor_location'],dict(x=1,y=2,z=3))
        self.assertEqual(sum(r['event']=='coverage_ended' for r in records),1)
        from evaluation.collision_events import summarize
        summarize(self.path/'collisions.jsonl',self.path)
        summary=json.loads((self.path/'collision_summary.json').read_text())
        self.assertEqual(summary['pair_event_count'],1)
        self.assertEqual(summary['coverage']['listening_eligible_ratio'],.5)

    def test_attachment_failure_is_recorded_before_raising(self):
        def fail(*args,**kwargs):
            raise RuntimeError('sensor spawn failed')
        self.world.spawn_actor=fail
        recorder=CollisionRecorder(self.world,self.path)
        with self.assertRaises(RuntimeError):
            recorder.attach(self.vehicle,10)
        recorder.close()
        from evaluation.collision_events import summarize
        summarize(self.path/'collisions.jsonl',self.path)
        summary=json.loads((self.path/'collision_summary.json').read_text())
        self.assertEqual(summary['coverage']['errors'][0]['event'],'sensor_attach_failed')
        self.assertEqual(summary['pair_event_count'],0)

    def test_synchronous_attachment_is_not_a_permanent_coverage_failure(self):
        recorder=CollisionRecorder(self.world,self.path)
        self.sensor.is_alive=False
        self.sensor.is_listening=lambda: True
        recorder.attach(self.vehicle,10)
        snapshot=NS(frame=10,timestamp=NS(elapsed_seconds=.5),find=lambda vid:object())
        recorder.observe(snapshot,[1])
        self.assertNotIn(1,recorder.ended)
        self.sensor.is_alive=True;snapshot.frame=11
        recorder.observe(snapshot,[1])
        self.sensor.is_listening=lambda: False;snapshot.frame=12
        recorder.observe(snapshot,[1])
        self.assertIn(1,recorder.ended)
        recorder.close()
        records=[json.loads(line) for line in (self.path/'collisions.jsonl').read_text().splitlines()]
        coverage=[r['listening_vehicle_ids'] for r in records if r['event']=='coverage_frame']
        self.assertEqual(coverage,[[],[1],[]])
        ends=[r for r in records if r['event']=='coverage_ended']
        self.assertEqual([r['frame'] for r in ends],[12])
