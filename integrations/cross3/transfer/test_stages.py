import json
import tempfile
import unittest
from pathlib import Path
from .stages import load_receipt,metric_sources,sha,checkpoint_in_lineage

class StageGates(unittest.TestCase):
    def test_reject_missing_failed_and_stale_receipts(self):
        with self.assertRaises(ValueError): load_receipt(None,1)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'gate.json'
            for receipt in [dict(stage=1,passed=False),dict(stage=1,passed=True,metric_sources={})]:
                path.write_text(json.dumps(receipt))
                with self.assertRaises(ValueError): load_receipt(path,1)
    def test_evidence_tampering_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            artifact=Path(folder)/'evidence'; artifact.write_text('verified')
            path=Path(folder)/'gate.json'
            path.write_text(json.dumps(dict(stage=1,passed=True,metric_sources=metric_sources(),
                artifacts={str(artifact):sha(artifact)})))
            self.assertTrue(load_receipt(path,1)['passed'])
            artifact.write_text('changed')
            with self.assertRaises(ValueError): load_receipt(path,1)
    def test_checkpoint_lineage_is_bound_to_accepted_hash(self):
        source=dict(source_checkpoint_sha256='new',source=dict(source_checkpoint_sha256='accepted'))
        self.assertTrue(checkpoint_in_lineage(None,source,'accepted'))
        self.assertFalse(checkpoint_in_lineage(None,source,'unrelated'))

if __name__=='__main__': unittest.main()
