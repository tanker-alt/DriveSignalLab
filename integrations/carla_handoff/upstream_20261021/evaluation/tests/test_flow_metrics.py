import unittest
from evaluation.metrics import FlowMetrics


class FlowTest(unittest.TestCase):
    def test_window_zero_rows_and_repeated_vehicle(self):
        lines=[dict(id=k,kind="arrival",junction_id=1427,enabled=True) for k in ["a","b"]]
        f=FlowMetrics(lines,60)
        events=[dict(line_id="a",vehicle_id="1",estimated_crossing_time=t) for t in [2,1]]
        rows=f.update(2,events)
        self.assertEqual(rows[0]["window_count"],2)
        self.assertEqual(rows[0]["unique_vehicle_count"],1)
        self.assertEqual(rows[1]["window_count"],0)
        self.assertEqual(f.update(61,[])[0]["window_count"],1)
        self.assertEqual(f.update(62,[])[0]["window_count"],0)
        self.assertEqual(f.totals["a"],2)
