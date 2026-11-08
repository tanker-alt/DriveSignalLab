import unittest
from .stage2 import score,acceptable

class LearningAcceptance(unittest.TestCase):
    def row(self,**changes):
        row=dict(unfinished=0,due=10,collision_pairs=0,red_light_violations=0,mean_observed_wait_s=20.)
        row.update(changes);return row
    def test_fast_completed_subset_does_not_pass(self):
        reference=score([self.row()])
        self.assertFalse(acceptable(score([self.row(unfinished=2,mean_observed_wait_s=1)]),reference))
    def test_efficiency_cannot_hide_safety_regression(self):
        reference=score([self.row()])
        for change in ({'collision_pairs':1},{'red_light_violations':1}):
            self.assertFalse(acceptable(score([self.row(mean_observed_wait_s=1,**change)]),reference))
    def test_safe_noninferiority_gate(self):
        reference=score([self.row(),self.row()])
        self.assertTrue(acceptable(score([self.row(mean_observed_wait_s=19),self.row()]),reference))
        self.assertFalse(acceptable(score([self.row(mean_observed_wait_s=21),self.row()]),reference))

if __name__=='__main__':unittest.main()
