import unittest
from .audit_kinematics import check_frames

class PhysicalSnapshots(unittest.TestCase):
    def frame(self,before,after,time=1.):
        return dict(before=before,after=after,time=time,dt=1.)
    def v(self,x,speed=0.): return dict(x=x,y=0.,speed=speed)
    def test_physical_motion_and_first_visibility(self):
        self.assertTrue(check_frames([self.frame({}, {'a':self.v(112.,5.)}),
            self.frame({'a':self.v(112.,5.)},{'a':self.v(117.,5.)},2.)])['passed'])
    def test_default_origin_teleport_rejected(self):
        result=check_frames([self.frame({'a':self.v(0.)},{'a':self.v(112.,.49)})])
        self.assertFalse(result['passed']);self.assertEqual(result['error_count'],1)
    def test_jump_between_recorded_ticks_rejected(self):
        result=check_frames([self.frame({}, {'a':self.v(10.)}),
            self.frame({'a':self.v(100.)},{'a':self.v(100.)},2.)])
        self.assertFalse(result['passed']);self.assertEqual(result['errors'][0]['kind'],'between_snapshots')

if __name__=='__main__':unittest.main()
