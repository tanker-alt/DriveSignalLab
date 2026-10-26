import json
import tempfile
import unittest
from pathlib import Path
from evaluation.collision_events import aggregate, summarize


def callback(a,b,time,frame=None,other_type='vehicle.test'):
    return dict(event='collision_callback',vehicle_id=a,other_actor_id=b,
                vehicle_type='vehicle.test',other_actor_type=other_type,
                sim_time=time,frame=int(time*20) if frame is None else frame,impulse_magnitude=12.)


class CollisionEventsTest(unittest.TestCase):
    def test_bilateral_repeated_and_unordered_callbacks_merge(self):
        records=[callback(2,1,1.1),callback(1,2,1),callback(1,2,1),callback(2,1,1.2)]
        events,summary=aggregate(records,1)
        self.assertEqual(summary['raw_callback_count'],4)
        self.assertEqual(summary['pair_event_count'],1)
        self.assertEqual(events[0]['actor_ids'],[1,2])
        self.assertTrue(events[0]['bidirectional_callbacks'])
        self.assertEqual(events[0]['peak_callback_impulse'],12)
        self.assertEqual(events[0]['source_lines'],[2,3,1,4])

    def test_adjacent_gap_not_total_duration_and_boundary(self):
        events,_=aggregate([callback(1,2,t) for t in (1,2,3,4.0001)],1)
        self.assertEqual([e['callback_count'] for e in events],[3,1])
        self.assertEqual(len(aggregate([callback(1,2,1),callback(1,2,1.2)],.1)[0]),2)

    def test_three_vehicle_chain_stays_two_pairs_and_unilateral_is_counted(self):
        events,summary=aggregate([callback(1,2,1),callback(2,3,1)],1)
        self.assertEqual(summary['pair_event_count'],2)
        self.assertEqual(summary['bidirectional_pair_event_count'],0)
        self.assertEqual([e['responsibility'] for e in events],['not_assigned']*2)

    def test_unknown_actor_identity_is_not_fabricated_as_pair_episode(self):
        _,summary=aggregate([callback(1,0,1,other_type='static.unknown'),callback(1,0,2)],1)
        self.assertEqual(summary['pair_event_count'],0)
        self.assertEqual(summary['unresolved_callback_count'],2)
        self.assertTrue(summary['observed_any_collision_callback'])

    def test_categories_and_missing_coverage(self):
        events,summary=aggregate([callback(1,2,1,other_type='walker.pedestrian.1'),
                                  callback(1,3,2,other_type='static.prop.wall')],1)
        self.assertEqual([e['category'] for e in events],['vehicle_pedestrian','vehicle_static'])
        self.assertEqual(summary['coverage']['status'],'unknown')
        self.assertIsNone(summary['coverage']['listening_eligible_ratio'])

    def test_attachment_frame_gap_and_listener_loss_are_not_full_coverage(self):
        records=[dict(event='sensor_attached',vehicle_id=1,frame=10),
            dict(event='coverage_frame',frame=10,observed_vehicle_ids=[1],listening_vehicle_ids=[1]),
            dict(event='coverage_frame',frame=11,observed_vehicle_ids=[1],listening_vehicle_ids=[1]),
            dict(event='coverage_frame',frame=13,observed_vehicle_ids=[1],listening_vehicle_ids=[])]
        _,summary=aggregate(records,1)
        coverage=summary['coverage']
        self.assertEqual(coverage['observed_vehicle_frames'],3)
        self.assertEqual(coverage['listening_eligible_vehicle_frames'],1)
        self.assertEqual(coverage['status'],'gaps_or_errors')
        self.assertEqual(coverage['frame_gaps'],[dict(after=11,before=13,missing_frames=1)])
        self.assertFalse(coverage['callback_delivery_complete'])

    def test_invalid_numbers_rejected(self):
        for gap in (-1,float('nan'),float('inf')):
            with self.assertRaises(ValueError):
                aggregate([],gap)
        with self.assertRaises(ValueError):
            aggregate([callback(1,2,float('nan'),frame=1)],1)

    def test_replay_keeps_raw_bytes_and_source_line_numbers(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);raw=root/'collisions.jsonl'
            data=('\n'+json.dumps(callback(1,2,1))+'\n').encode('utf-8')
            raw.write_bytes(data)
            summarize(raw,root/'analysis',1)
            self.assertEqual(raw.read_bytes(),data)
            event=json.loads((root/'analysis/collision_events.jsonl').read_text())
            self.assertEqual(event['source_lines'],[2])
            with self.assertRaises(FileExistsError):
                summarize(raw,root/'analysis',1)
