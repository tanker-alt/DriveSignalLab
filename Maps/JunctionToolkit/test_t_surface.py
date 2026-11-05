import math
import unittest
from generate import check, make_tile
from layout import ARM_NAMES


class TSurfaceTests(unittest.TestCase):
    def test_closed_edge_and_lane_width_for_all_orientations(self):
        for absent in ARM_NAMES:
            with self.subTest(absent=absent):
                c = check(dict(type='t', missing_arm=absent))
                root, moves = make_tile(c, [150.]*4)
                self.assertEqual(len(moves), 6)
                self.assertEqual(len(root.findall('./road/signals/signal')), 6)
                closed=root.find("road[@id='914']")
                self.assertIsNotNone(closed)
                g=closed.find('./planView/geometry')
                heading=float(g.get('hdg')); length=float(g.get('length'))
                mx=float(g.get('x'))+length/2*math.cos(heading)-(150+c['junction_radius_m'])
                my=float(g.get('y'))+length/2*math.sin(heading)
                angle=ARM_NAMES.index(absent)*math.pi/2
                # Arm indices point inward; absent boundary lies outward.
                self.assertAlmostEqual(mx, -7*math.cos(angle), places=5)
                self.assertAlmostEqual(my, -7*math.sin(angle), places=5)
                for rid in (1,2,3,4):
                    road=root.find("road[@id='%s']" % rid)
                    if road is None:continue
                    lanes=road.findall('./lanes/laneSection/right/lane')
                    self.assertEqual([int(l.get('id')) for l in lanes], [-1,-2,-3,-4])
                    self.assertEqual([l.get('type') for l in lanes], ['driving','driving','sidewalk','sidewalk'])


if __name__ == '__main__':
    unittest.main()
