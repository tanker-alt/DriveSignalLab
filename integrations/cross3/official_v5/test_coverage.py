import unittest
import json,tempfile
from pathlib import Path
from coverage_quota import combination,counts,deficit,gate,schedule,require_coverage,DEFAULT_QUOTAS

class CoverageTest(unittest.TestCase):
    def test_complete_unique_grid(self):
        specs=[combination(i) for i in range(32)]
        self.assertEqual(len({(s['road'],s['lane'],s['initial_phase']) for s in specs}),32)
        self.assertEqual([sum(s['demand_action']==a for s in specs) for a in range(4)],[8]*4)
    def test_excludes_empty_outgoing_and_other_traffic(self):
        def row(incoming,outgoing,action=0):return dict(state=dict(coming_vehicle=incoming,leaving_vehicle=outgoing),action=action)
        rows=[row([0,0,0],[0]),row([0,1,0],[0]),row([0,1,0],[1]),row([0,2,0],[0]),row([0,1,0],[0],1)]
        self.assertEqual(counts(rows,dict(road=2,lane=-2,demand_action=0),[(2,-2)]),dict(observed=2,served=1,alternative=1,actions=[1,1,0,0]))
    def test_gate_requires_every_combination_and_each_quota(self):
        entries=[dict(id=i,deficit=dict(observed=0,served=0,alternative=0)) for i in range(32)]
        self.assertTrue(gate(entries));self.assertFalse(gate(entries[:-1]))
        entries[7]['deficit']['served']=1
        self.assertFalse(gate(entries));self.assertEqual(schedule(entries),[7])
        self.assertEqual(deficit(dict(observed=20,served=1,alternative=19)),dict(observed=0,served=3,alternative=0))
    def test_serialized_gate_cannot_override_counts(self):
        with self.assertRaises(RuntimeError):require_coverage(None)
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'plan.json'
            entries=[dict(id=i,counts=dict(observed=12,served=4,alternative=8),deficit=dict(observed=0,served=0,alternative=0)) for i in range(32)]
            plan=dict(quotas=DEFAULT_QUOTAS,entries=entries,gate_passed=True)
            path.write_text(json.dumps(plan));self.assertTrue(require_coverage(path))
            entries[0]['counts']['served']=0
            path.write_text(json.dumps(plan))
            with self.assertRaises(RuntimeError):require_coverage(path)

if __name__=='__main__':unittest.main()
