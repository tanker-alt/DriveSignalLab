import unittest
from layout import active_arms,arm_turns,ARM_NAMES,TURN_OFFSETS
class LayoutTests(unittest.TestCase):
    def test_all_t_variants(self):
        for n in range(1,5):
            for absent in ARM_NAMES:
                c=dict(type='t',missing_arm=absent,incoming_lanes=n)
                active=active_arms(c);self.assertEqual(len(active),3)
                for i,lanes in arm_turns(c).items():
                    self.assertEqual(len(lanes),n)
                    for ts in lanes:
                        self.assertTrue(ts)
                        self.assertTrue(all((i+TURN_OFFSETS[t])%4 in active for t in ts))
    def test_missing_direction_rejected(self):
        c=dict(type='t',missing_arm='north',incoming_lanes=1,
               arm_turns=dict(west=[['left']],south=[['left','right']],east=[['straight','left']]))
        with self.assertRaises(ValueError):arm_turns(c)
    def test_old_cross(self):
        c=dict(incoming_lanes=3,allowed_turns=[['left'],['straight'],['right']])
        self.assertEqual(active_arms(c),[0,1,2,3])
        self.assertTrue(all(v==c['allowed_turns'] for v in arm_turns(c).values()))
if __name__=='__main__':unittest.main()
