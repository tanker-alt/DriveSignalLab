"""Offline numeric contract tests against extracted, unmodified upstream methods."""
import ast
import json
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
from environment import Environment
from schema import PHASE_BITS,position_bin

ROOT=Path(__file__).resolve().parents[4]
UPSTREAM=ROOT/'Drive/repos/PressLight_official'


def upstream_method(filename,name):
    tree=ast.parse((UPSTREAM/filename).read_text())
    node=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name==name)
    module=ast.Module(body=[node],type_ignores=[])
    namespace={'np':np};exec(compile(module,str(UPSTREAM/filename),'exec'),namespace)
    return namespace[name]


class AdapterTests(unittest.TestCase):
    def test_bins_match_upstream_including_boundaries(self):
        distances=[-1,0,49.99,50,99.99,100,149.99,150,151]
        expected=upstream_method('anon_env.py','lane_position_mapper')(None,distances,[0,50,100,150])
        actual=[0,0,0]
        for distance in distances:
            index=position_bin(distance)
            if index is not None:actual[index]+=1
        self.assertEqual(actual,expected)

    def test_phase_bits_match_official_action_semantics(self):
        tree=ast.parse((UPSTREAM/'runexp.py').read_text())
        phase=None
        for node in ast.walk(tree):
            if not isinstance(node,ast.Dict):continue
            for key,value in zip(node.keys,node.values):
                if isinstance(key,ast.Constant) and key.value=='PHASE':
                    candidate=ast.literal_eval(value)
                    if len(candidate.get('anon',{}))==4:phase=candidate['anon'];break
            if phase is not None:break
        self.assertIsNotNone(phase)
        self.assertEqual(PHASE_BITS,[phase[k] for k in (2,4,1,3)])

    def test_real_observe_lane_bins_reward_and_maxpressure(self):
        env=Environment.__new__(Environment)
        env.moves=json.loads((ROOT/'DriveSignalLab/integrations/carla_handoff/upstream_20261023_v4/map_dataset/junction_toolkit/maps/cross3/manifest.json').read_text())['movements']
        env.keys=[(m['road'],m['lane']) for m in env.moves]
        env.outkeys=[(r,l) for r in (1,2,3,4) for l in (1,2,3)]
        env.phase_indices=[[4,10],[3,9],[1,7],[0,6]]
        env.map=NS(get_waypoint=lambda loc:loc)
        fleet=[];counts={key:0 for key in env.keys+env.outkeys}
        expected_in=[0]*36;expected_out=[0]*36
        for index,key in enumerate(env.keys+env.outkeys):
            distance=25+50*(index%3)
            wp=NS(road_id=key[0],lane_id=key[1],s=distance if key[1]<0 else 150-distance)
            speed=0 if index%2==0 else 2
            counts[key]=int(speed==0)
            actor=NS(get_location=lambda wp=wp:wp,get_velocity=lambda speed=speed:NS(x=speed,y=0,z=0))
            fleet.append({'actor':actor})
            expected=(expected_in if key[1]<0 else expected_out)
            expected[(index%12)*3+index%3]+=1
        state,reward,pressures,queued,_=env.observe(fleet,1)
        self.assertEqual(state['coming_vehicle'],expected_in)
        self.assertEqual(state['leaving_vehicle'],expected_out)
        self.assertEqual(state['cur_phase'],PHASE_BITS[1])
        obj=NS(dic_lane_waiting_vehicle_count_current_step=counts,list_entering_lanes=env.keys,list_exiting_lanes=env.outkeys)
        original=upstream_method('anon_env.py','_get_pressure')(obj)
        self.assertEqual(reward,-.25*abs(sum(original)))
        self.assertEqual(queued,12)
        # NS left has two stopped downstream cars; EW left has two upstream.
        self.assertEqual(pressures,[0,-2,0,2])
        # Additional incoming queue makes the non-zero reward sign and scale observable.
        fleet.append(fleet[0]);self.assertEqual(env.observe(fleet,0)[1],-.25)

    def test_clearance_tracks_outgoing_tail_and_ignores_waiting_approach(self):
        env=Environment.__new__(Environment);env.stop_lines={1:144}
        def vehicle(s,lane=-1,junction=False,right=False):
            actor=NS(id=s,bounding_box=NS(extent=NS(x=2.4)))
            return ({'actor':actor,'move':{'lane':-3 if right else -1}},
                    NS(road_id=1,lane_id=lane,s=s,is_junction=junction),0)
        positions=[vehicle(140),vehicle(145),vehicle(148,lane=1),vehicle(143,lane=1),
                   vehicle(10,junction=True),vehicle(12,junction=True,right=True)]
        self.assertEqual(env.clearance_blockers(positions),[145,148,10])


if __name__=='__main__':unittest.main()
