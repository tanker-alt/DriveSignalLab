import unittest
from generate import check, _random_missing_arms
from network_plan import connected


class PureTNetwork(unittest.TestCase):
    def test_defaults_and_template(self):
        single=check(dict(type='t',rows=1,columns=1))
        network=check(dict(type='t',rows=1,columns=6))
        self.assertTrue(network['random_missing_arms'])
        self.assertFalse(single['random_missing_arms'])
        self.assertEqual(network['incoming_lanes'],2)
        for key in ('lane_width_m','junction_radius_m','sidewalk_width_m',
                    'curb_width_m','curb_height_m','corner_radius_m','standard_profile'):
            self.assertEqual(network[key],single[key])

    def test_connected_strips_and_grids(self):
        for rows,cols in [(1,1),(1,6),(6,1),(3,3)]:
            for seed in range(10):
                cfg=check(dict(type='t',rows=rows,columns=cols,seed=seed,random_missing_arms=True))
                assignments=_random_missing_arms(cfg)
                self.assertEqual(assignments,_random_missing_arms(cfg))
                self.assertTrue(connected(rows,cols,{'%d,%d'%k:v for k,v in assignments.items()}))
                if rows==1:
                    for col in range(1,cols-1):self.assertIn(assignments[(0,col)],('north','south'))
                if cols==1:
                    for row in range(1,rows-1):self.assertIn(assignments[(row,0)],('east','west'))


if __name__=='__main__':unittest.main()
