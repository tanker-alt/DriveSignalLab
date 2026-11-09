import unittest
from layout import ARM_NAMES, arm_turns
from network_plan import plan, connected


class RemovalRules(unittest.TestCase):
    def test_document_tables(self):
        tables = {
            'north': {'west':'SSR','south':'LLR','east':'LLS'},
            'south': {'north':'LLR','west':'LLS','east':'SSR'},
            'east': {'north':'SSR','west':'LLR','south':'LLS'},
            'west': {'north':'LLS','south':'SSR','east':'LLR'}}
        codes = {'left':'L','straight':'S','right':'R'}
        for absent, expected in tables.items():
            actual = arm_turns(dict(type='t',missing_arm=absent,incoming_lanes=3))
            self.assertEqual({ARM_NAMES[i]:''.join(codes[t[0]] for t in lanes)
                              for i,lanes in actual.items()},expected)

    def test_seed_and_connectivity(self):
        for rows,cols in [(1,6),(3,3)]:
            for seed in range(30):
                config = dict(rows=rows,columns=cols,seed=seed)
                result,_ = plan(config)
                self.assertEqual(plan(config)[0],result)
                self.assertTrue(connected(rows,cols,result['missing_arms']))
                self.assertEqual(set(result['missing_arms']),
                                 {k for k,v in result['junction_types'].items() if v=='t'})

    def test_invalid_fixed_disconnect(self):
        with self.assertRaises(ValueError):
            plan(dict(rows=1,columns=2,junction_types={'0,0':'t','0,1':'cross'},
                      missing_arms={'0,0':'east'}))


if __name__=='__main__': unittest.main()
