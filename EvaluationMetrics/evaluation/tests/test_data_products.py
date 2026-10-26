import json
import tempfile
import unittest
from pathlib import Path
from evaluation.data_products import AssessmentWriter
from evaluation.observations import observation_packet


class DataProductsTest(unittest.TestCase):
    def test_collision_accident_count_and_coverage_reach_assessment(self):
        from evaluation.collision_events import summarize
        callbacks = [dict(event='collision_callback', frame=2, sim_time=.1,
            vehicle_id=a, other_actor_id=b, impulse_magnitude=10,
            vehicle_type='vehicle.test', other_actor_type='vehicle.test', sensor_location=dict(x=0,y=0))
            for a, b in ((1, 2), (2, 1), (2, 3))]
        records = [dict(event='sensor_attached', vehicle_id=vid, frame=0) for vid in (1, 2)]
        records += [dict(event='coverage_frame', frame=2, sim_time=.1,
                         observed_vehicle_ids=[1, 2], listening_vehicle_ids=[1, 2])]
        with tempfile.TemporaryDirectory() as tmp:
            raw = Path(tmp)/'collisions.jsonl'
            raw.write_text(''.join(json.dumps(row)+'\n' for row in records+callbacks))
            (Path(tmp)/'metadata.json').write_text(json.dumps(dict(region_center_xy=[0,0],radius_m=120)))
            audit = summarize(raw,Path(tmp),1.)
            self.assertEqual(audit['pair_event_count'],2)
            self.assertEqual(len((Path(tmp)/'collision_accidents.jsonl').read_text().splitlines()),1)
            writer = AssessmentWriter(tmp)
            writer.sample(2, .1, [], [], [], {})
            report = writer.close('completed', {}, audit)
            collision = report['Safety']['collision']
            self.assertEqual(collision['value'], 1)
            self.assertEqual(collision['raw_callback_count'], 3)
            self.assertEqual(collision['pair_categories'], {'vehicle_vehicle': 2})
            self.assertEqual(collision['coverage']['listening_eligible_ratio'], 1.)
            self.assertFalse(collision['coverage']['callback_delivery_complete'])
            frame = json.loads((Path(tmp)/'assessment/evaluation_metrics.jsonl').read_text())
            self.assertIsNone(frame['Safety']['collision']['value'])
            self.assertEqual(frame['Safety']['collision']['status'], 'pending_final_audit')

    def test_observation_does_not_leak_future_or_outcome(self):
        packet = observation_packet(1, .1, [dict(vehicle_id=1, x=2, completed=True, reward=9,
            next_phase_id='future', travel_time_s=15)], {'a': 'Green'},
            [dict(vehicle_id=1, reward=99, next_phase_id='future')], phase={'remaining_seconds':5})
        text = json.dumps(packet)
        for name in ('completed', 'reward', 'next_phase_id', 'travel_time_s', 'remaining_seconds'):
            self.assertNotIn(name, text)
        self.assertEqual(packet['vehicles'][0], {'vehicle_id':1, 'x':2})

    def test_derivative_gaps_and_time_weighted_queue(self):
        with tempfile.TemporaryDirectory() as tmp:
            writer = AssessmentWriter(tmp)
            def sample(frame, now, accel, length, valid=True):
                writer.sample(frame, now, [dict(vehicle_id=1, longitudinal_accel_mps2=accel,
                    lateral_accel_mps2=0)], [dict(approach_id='a', queue_length_m=length,
                    metric_status='valid' if valid else 'missing_vehicle_geometry')], [], {})
            sample(1, 0, 0, 0)
            sample(2, .5, 1, 10)  # jerk 2, trapezoid queue mean 5
            sample(4, 2, 100, 100)  # gap must not contribute jerk or queue integration
            sample(5, 2.5, None, 100, False)
            report = writer.close('completed', {})
            rows = [json.loads(s) for s in (Path(tmp)/'assessment/evaluation_metrics.jsonl').read_text().splitlines()]
            self.assertEqual(rows[1]['Comfort']['vehicles'][0]['longitudinal_jerk_mps3'], 2)
            self.assertIsNone(rows[2]['Comfort']['vehicles'][0]['longitudinal_jerk_mps3'])
            self.assertIsNone(rows[3]['Comfort']['vehicles'][0]['longitudinal_jerk_mps3'])
            self.assertEqual(report['Efficiency']['mean_queue_length_m'], 5)
            self.assertEqual(report['Efficiency']['queue_covered_lane_seconds'], .5)
            self.assertIsNone(report['Safety']['collision']['value'])
            self.assertIsNone(report['Efficiency']['delay_seconds']['value'])
            self.assertIsNone(report['Efficiency']['throughput_crossings'])

    def test_reappearing_actor_does_not_bridge_jerk(self):
        with tempfile.TemporaryDirectory() as tmp:
            writer = AssessmentWriter(tmp)
            row = dict(vehicle_id=1, longitudinal_accel_mps2=3, lateral_accel_mps2=1)
            writer.sample(1, 0, [row], [], [], {})
            writer.sample(2, .1, [], [], [], {})
            writer.sample(3, .2, [row], [], [], {})
            report = writer.close('completed', {})
            self.assertNotIn('longitudinal_jerk_mps3', report['Comfort']['statistics'])
