import tempfile
import unittest
from evaluation.task_efficiency import TaskEfficiency
from evaluation.data_products import AssessmentWriter


def task(**changes):
    result = dict(task_id='t1', vehicle_id=1, start=dict(x=0, y=0), goal=dict(x=10, y=0),
                  arrival_radius_m=1, require_stop=False, timeout_s=10,
                  reference_segments=[dict(length_m=5, reference_speed_mps=5),
                                      dict(length_m=5, reference_speed_mps=10)])
    result.update(changes)
    return result


def row(x, speed=0):
    return dict(vehicle_id=1, x=x, y=0, speed_mps=speed)


class TaskEfficiencyTest(unittest.TestCase):
    def result(self, e):
        return e.summary()['task_completion']['tasks'][0]

    def test_completed_ideal_delay_and_radius(self):
        e = TaskEfficiency()
        e.sample(1, 100, [row(0)], [task()])
        e.sample(2, 102, [row(9)])
        t = self.result(e)
        self.assertEqual(t['status'], 'completed')
        self.assertEqual(t['ideal_duration_s'], 1.5)
        self.assertEqual(t['actual_duration_s'], 2)
        self.assertEqual(t['delay_seconds'], .5)

    def test_stop_hold_resets_and_includes_confirmation(self):
        e = TaskEfficiency()
        e.sample(0, 0, [row(0)], [task(require_stop=True)])
        for frame, (now, x, speed) in enumerate(((1, 10, .1), (1.25, 10, .2),
                                               (1.5, 10, .1), (1.75, 10, .1), (2, 10, .1)), 1):
            e.sample(frame, now, [row(x, speed)])
        self.assertEqual(self.result(e)['actual_duration_s'], 2)

    def test_timeout_and_deadline_arrival(self):
        for x, expected in ((0, 'timed_out'), (10, 'completed')):
            e = TaskEfficiency()
            e.sample(0, 0, [row(0)], [task(timeout_s=1)])
            e.sample(1, 1, [row(x)])
            self.assertEqual(self.result(e)['status'], expected)
        e = TaskEfficiency()
        e.sample(0, 0, [row(0)], [task(timeout_s=1)])
        e.sample(1, 1.1, [row(10)])
        self.assertEqual(self.result(e)['status'], 'timed_out')
        self.assertIsNone(self.result(e)['delay_seconds'])

    def test_censor_gap_missing_end(self):
        for case in ('gap', 'missing', 'end'):
            e = TaskEfficiency()
            e.sample(0, 0, [row(0)], [task()])
            if case == 'gap':
                e.sample(2, 1, [row(10)])
            elif case == 'missing':
                e.sample(1, 1, [])
            e.close()
            self.assertEqual(self.result(e)['status'], 'censored')
            self.assertIsNone(self.result(e)['actual_duration_s'])

    def test_invalid_reference_keeps_task_completion(self):
        for segments in (None, [], [dict(length_m=5, reference_speed_mps=0)]):
            e = TaskEfficiency()
            e.sample(0, 0, [row(10)], [task(reference_segments=segments)])
            self.assertEqual(self.result(e)['status'], 'completed')
            self.assertIsNone(self.result(e)['delay_seconds'])

    def test_invalid_task_and_duplicate(self):
        for changes in (dict(timeout_s=0), dict(arrival_radius_m=-1), dict(require_stop='yes')):
            with self.assertRaises(ValueError):
                TaskEfficiency().sample(0, 0, [row(0)], [task(**changes)])
        e = TaskEfficiency()
        e.sample(0, 0, [row(0)], [task()])
        with self.assertRaises(ValueError):
            e.sample(1, 1, [row(0)], [task()])

    def test_writer_and_negative_delay_not_clipped(self):
        with tempfile.TemporaryDirectory() as tmp:
            writer = AssessmentWriter(tmp)
            writer.sample(0, 0, [row(0)], [], [], {}, task_starts=[task()])
            writer.sample(1, 1, [row(10)], [], [], {})
            report = writer.close('completed', {})['Efficiency']
            self.assertEqual(report['delay_seconds']['value'], -.5)
            self.assertEqual(report['task_completion']['counts']['completed'], 1)
            self.assertIsNone(report['task_completion']['value'])
