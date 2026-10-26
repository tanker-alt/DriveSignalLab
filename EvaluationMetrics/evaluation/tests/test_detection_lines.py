import unittest
from evaluation.detection_lines import DetectionLines


class DetectionTest(unittest.TestCase):
    def detector(self):
        return DetectionLines([dict(id="a", kind="arrival", junction_id=1427,
            center_xy=[0,0], normal_xy=[1,0], half_width_m=2, enabled=True)])

    def row(self, x, y=0, t=0):
        return dict(vehicle_id=1, x=x, y=y, sim_time=t)

    def test_crossing_interpolation(self):
        d=self.detector()
        d.update(1, 0, [self.row(-1)])
        result=d.update(2, 1, [self.row(3,t=1)])
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]["estimated_crossing_time"], .25)

    def test_gap_reverse_and_outside_not_counted(self):
        for first, second, end_frame in [(self.row(-1),self.row(1,t=1),3),
            (self.row(1),self.row(-1,t=1),2),
            (self.row(-1,y=3),self.row(1,y=3,t=1),2)]:
            d=self.detector()
            d.update(1,0,[first])
            self.assertEqual(d.update(end_frame,1,[second]),[])
