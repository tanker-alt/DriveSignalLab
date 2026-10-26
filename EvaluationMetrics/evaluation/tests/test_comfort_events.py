import json
import tempfile
import unittest
from pathlib import Path
from evaluation.comfort_events import ComfortEvents
from evaluation.data_products import AssessmentWriter


class ComfortEventsTest(unittest.TestCase):
    def feed(self, evaluator, frame, a, speed=5):
        return evaluator.sample(frame, frame*.25, [dict(vehicle_id=1,
            longitudinal_accel_mps2=a, vx=speed, vy=0, yaw=0)])

    def test_both_events_duration_hysteresis_and_stop(self):
        for sign, name in ((-1, 'harsh_braking'), (1, 'harsh_acceleration')):
            e = ComfortEvents([1])
            for frame, a in enumerate((0, 3, 4, 4, 4, 2)):
                result = self.feed(e, frame, sign*a)
            self.assertEqual(result[name]['active_count'], 1)
            self.assertEqual(result[name]['value'], 0)
            result = self.feed(e, 6, sign, speed=0)
            self.assertEqual(result[name]['value'], 1)
            self.assertEqual(e.events[0]['start_sim_time'], .5)

    def test_short_trigger_resets(self):
        e = ComfortEvents([1])
        for frame, a in enumerate((0, 4, 4, 3, 4, 2, 0)):
            self.feed(e, frame, a)
        self.assertEqual(e.close()['harsh_acceleration']['value'], 0)
        self.assertEqual(e.events, [])

    def test_gap_reverse_missing_and_end_censor(self):
        for termination in ('gap', 'reverse', 'missing', 'end'):
            e = ComfortEvents([1])
            for frame, a in enumerate((0, 4, 4, 4)):
                self.feed(e, frame, a)
            if termination == 'gap':
                self.feed(e, 5, 0)
            elif termination == 'reverse':
                self.feed(e, 4, 0, -1)
            elif termination == 'missing':
                e.sample(4, 1., [])
            report = e.close()['harsh_acceleration']
            self.assertEqual(report['value'], 0)
            self.assertEqual(report['censored_count'], 1)

    def test_initial_event_and_no_identity(self):
        e = ComfortEvents([1])
        for frame, a in enumerate((-4, -4, -4, 0)):
            self.feed(e, frame, a)
        self.assertEqual(e.close()['harsh_braking']['censored_count'], 1)
        e = ComfortEvents()
        self.assertIsNone(self.feed(e, 1, -4)['harsh_braking']['value'])

    def test_config_and_writer_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp)/'rules.json'
            rules = ComfortEvents().config
            rules['harsh_acceleration']['minimum_duration_s'] = .25
            config.write_text(json.dumps(rules))
            writer = AssessmentWriter(tmp, comfort_vehicle_ids=[1], comfort_config=config)
            for frame, a in enumerate((0, 4, 4, 1)):
                writer.sample(frame, frame*.25, [dict(vehicle_id=1,
                    longitudinal_accel_mps2=a, vx=5, vy=0, yaw=0)], [], [], {})
            self.assertEqual(writer.close('completed', {})['Comfort']['harsh_acceleration']['value'], 1)
            events = (Path(tmp)/'assessment/comfort_events.jsonl').read_text().splitlines()
            self.assertEqual(len(events), 1)
            rules['harsh_acceleration']['release_mps2'] = 4
            config.write_text(json.dumps(rules))
            with self.assertRaises(ValueError):
                ComfortEvents([1], config)
