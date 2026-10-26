"""No fit calls, training worker or simulator involved."""
import unittest
import numpy as np
from fit_policy import BestEpoch,target_checkpoint

class OfflinePolicyTest(unittest.TestCase):
    def test_restore_best_instead_of_last(self):
        class Model:
            def __init__(self):self.value=np.array([0.])
            def get_weights(self):return [self.value]
            def set_weights(self,w):self.value=w[0].copy()
        model=Model();best=BestEpoch()
        for epoch,loss in enumerate([.1,.2,.3]):
            model.value[:]=epoch+1;best.observe(model,epoch,{'val_loss':loss})
        report=best.restore(model)
        self.assertEqual(report['best_epoch'],0);self.assertEqual(model.value[0],1.)

    def test_no_finite_validation_refuses_save(self):
        best=BestEpoch()
        with self.assertRaises(RuntimeError):best.restore(None)

    def test_explicit_target_schedule(self):
        self.assertEqual([target_checkpoint(i,5) for i in range(7)],['initial','round_000','round_000','round_000','round_000','round_000','round_001'])
        with self.assertRaises(ValueError):target_checkpoint(0,0)

if __name__=='__main__':unittest.main()
