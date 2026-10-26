"""Acceptance regression: internally consistent unsafe logs must still fail."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('cross3_audit',HERE/'audit_cross3.py')
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class AuditTests(unittest.TestCase):
    def fixture(self,folder):
        config=dict(egos=0,episodes=1,teacher=False,right_turn='always-green')
        (folder/'config.json').write_text(json.dumps(config))
        (folder/'map_load.json').write_text('{}')
        ep=folder/'episode_000';ep.mkdir()
        summary=dict(status='finished',vehicles=[dict(actor_id=1,role='cross3_background',completed=True,route_violation=False)],collisions=[])
        (ep/'summary.json').write_text(json.dumps(summary))
        actual={str(1000+10*r+l):'Green' if l==3 else 'Red' for r in range(1,5) for l in range(1,4)}
        record=dict(phase=None,stage='RED',actual=actual,desired=copy.deepcopy(actual))
        (ep/'signals.jsonl').write_text(json.dumps(record)+'\n')
        return ep,record,summary

    def test_consistent_conflicting_greens_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp);ep,record,_=self.fixture(folder)
            self.assertTrue(module.audit(folder)['accepted'])
            record['actual']['1012']='Green'
            record['actual']['1022']='Green'
            record['desired']=copy.deepcopy(record['actual'])
            (ep/'signals.jsonl').write_text(json.dumps(record)+'\n')
            result=module.audit(folder)
            self.assertFalse(result['accepted'])
            self.assertEqual(result['episodes'][0]['phase_semantic_errors'],1)

    def test_finished_incomplete_vehicle_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp);ep,_,summary=self.fixture(folder)
            summary['vehicles'][0]['completed']=False
            (ep/'summary.json').write_text(json.dumps(summary))
            self.assertFalse(module.audit(folder)['accepted'])

    def test_expert_never_vla_acceptance(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp);self.fixture(folder)
            config=json.loads((folder/'config.json').read_text())
            config['teacher']=True
            (folder/'config.json').write_text(json.dumps(config))
            result=module.audit(folder)
            self.assertTrue(result['accepted'])
            self.assertFalse(result['vla_accepted'])


if __name__=='__main__':unittest.main()
