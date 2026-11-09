"""Hand-computable acceptance cases; no simulator is required."""
import copy
import math
import unittest
from .core import Metrics
from .safety import SafetyMetrics,rectangle_ttc
from .sumo_env import sumo_departure


def box(x,y,vx,vy,yaw=0,**kwargs):
    return dict(x=x,y=y,vx=vx,vy=vy,yaw=yaw,half_length=1.,half_width=.5,
                speed=math.hypot(vx,vy),gap=None,approach=True,front_stop_distance=10.,
                lane_key=kwargs.pop('lane_key','a'),**kwargs)


def polygons_overlap(a,b,t):
    """Independent static polygon projections, not the continuous TTC solver."""
    polygons=[]
    for v in (a,b):
        c,s=math.cos(v['yaw']),math.sin(v['yaw'])
        polygons.append([(v['x']+t*v['vx']+u*c-w*s,v['y']+t*v['vy']+u*s+w*c)
                         for u,w in [(-v['half_length'],-v['half_width']),
                                     (v['half_length'],-v['half_width']),
                                     (v['half_length'],v['half_width']),
                                     (-v['half_length'],v['half_width'])]])
    for polygon in polygons:
        for i,p in enumerate(polygon):
            q=polygon[(i+1)%4]; axis=(p[1]-q[1],q[0]-p[0])
            projections=[[x*axis[0]+y*axis[1] for x,y in poly] for poly in polygons]
            if max(projections[0])<min(projections[1])-1e-8 or max(projections[1])<min(projections[0])-1e-8:
                return False
    return True


class MeasurementAcceptance(unittest.TestCase):
    def test_queue_trend_is_stable_across_repeated_observations(self):
        from .core import Signals, observation
        s=Signals();v=dict(road=2,lane=-2,speed=0.,approach=True)
        observation({'a':v},s)
        s.tick(1.)
        a=observation({'a':v,'b':v},s)['light']
        b=observation({'a':v,'b':v},s)['light']
        self.assertEqual(a.tolist(),b.tolist())
        self.assertAlmostEqual(float(a[24]),1/20)
        s.tick(1.)
        self.assertEqual(float(observation({'a':v,'b':v},s)['light'][24]),0.)

    def test_service_age_only_resets_on_actual_green(self):
        from .core import Signals
        s=Signals();s.stage='GREEN';s.age=6.;s.service_age[:]=10.
        s.request(1,[1,1,0,0])
        self.assertEqual(s.service_age[1],10.)
        s.tick(3.)
        self.assertEqual(s.service_age[0],13.)
        self.assertEqual(s.service_age[1],13.)
        s.tick(2.,occupied=True)
        self.assertEqual(s.service_age[1],15.)
        s.tick(.1,occupied=False)
        self.assertEqual(s.stage,'GREEN');self.assertEqual(s.active,1)
        self.assertEqual(s.service_age[1],0.)

    def test_new_carla_actor_without_physics_snapshot_is_not_measured(self):
        from unittest.mock import Mock
        from .carla_env import CarlaEnv
        env=CarlaEnv.__new__(CarlaEnv)
        snapshot=Mock(); snapshot.find.return_value=None
        env.world=Mock(); env.world.get_snapshot.return_value=snapshot
        actor=Mock(); actor.id=123
        env.fleet={'new':dict(actor=actor)}
        self.assertEqual(env.vehicles(),{})
        snapshot.find.assert_called_once_with(123)
        actor.get_location.assert_not_called()
        actor.get_transform.assert_not_called()
        actor.get_velocity.assert_not_called()

    def test_audit_spawn_at_next_interval_is_not_visible_retroactively(self):
        import gzip
        import json
        import tempfile
        from pathlib import Path
        from .audit_metrics import audit
        demand=[dict(id='a',time=.5,road=1,lane=-1)]
        m=Metrics(demand)
        m.sample(1,1,{},after={})
        m.spawned('a',1)
        v=dict(speed=0.,vx=0.,vy=0.,spawn_time=1.,approach=True,
               front_stop_distance=44.,raw_position=100.,road=1,lane=-1,
               leader_speed=0.,gap=None)
        m.sample(2,1,{'a':v},after={'a':v})
        m.safety=SafetyMetrics()
        frames=[dict(time=1.,dt=1.,before={},after={}),
                dict(time=2.,dt=1.,before={'a':v},after={'a':v})]
        for f in frames:
            f.update(backend='sumo',completed=[],collisions=[],
                     colors={'a':'RED'},physical_signals={'1/-1':'RED'})
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp)
            (folder/'demand.json').write_text(json.dumps(demand))
            (folder/'metrics.json').write_text(json.dumps(m.report()))
            (folder/'vehicles.json').write_text(json.dumps(list(m.rows.values())))
            with gzip.open(folder/'measurements.jsonl.gz','wt') as stream:
                for frame in frames: stream.write(json.dumps(frame)+'\n')
            result=audit(folder)
            self.assertTrue(result['passed'],result['errors'])
            self.assertEqual(result['independently_recomputed']['max_external_queue_vehicles'],1)

    def test_sumo_millisecond_rounding_never_precedes_demand(self):
        self.assertEqual(sumo_departure(21.90031143183404),21.901)
        for when in (0.,.0001,.1,21.90031143183404,200.9999):
            self.assertGreaterEqual(sumo_departure(when),when-1e-10)
            self.assertLess(sumo_departure(when)-when,.001+1e-10)

    def test_complete_pending_and_future_hand_calculation(self):
        m=Metrics([dict(id='a',time=0,road=1),dict(id='b',time=.25,road=2),dict(id='c',time=4,road=3)])
        m.spawned('a',.5)
        m.sample(1,1,{},after={'a':dict(speed=0)})
        m.sample(2,1,{'a':dict(speed=2)},['a'],after={})
        m.sample(3,1,{})
        r=m.report()
        self.assertEqual(r['completed'],1);self.assertEqual(r['unfinished'],1);self.assertEqual(r['future_demand'],1)
        self.assertEqual(r['censored_vehicle_time_s'],4.75) # a:2, b:2.75
        self.assertEqual(r['external_waiting_vehicle_seconds'],3.25) # .5 + 2.75
        self.assertEqual(r['in_sim_waiting_vehicle_seconds'],.5)
        self.assertEqual(r['waiting_vehicle_seconds'],3.75)
        self.assertEqual(r['throughput_veh_per_hour'],1200)
    def test_duplicate_sample_and_unknown_disappearance_rejected(self):
        m=Metrics([dict(id='a',time=0,road=1)])
        m.sample(1,1,{'a':dict(speed=0)})
        with self.assertRaises(ValueError): m.sample(1,1,{})
        with self.assertRaises(ValueError): m.sample(2,1,{})
    def test_duplicate_demand_and_bad_numeric_input_rejected(self):
        with self.assertRaises(ValueError): Metrics([dict(id='a',time=0),dict(id='a',time=1)])
        with self.assertRaises(ValueError): Metrics([dict(id='a',time=float('nan'))])
        m=Metrics([dict(id='a',time=0)])
        with self.assertRaises(ValueError): m.sample(1,1,{'a':dict(speed=float('nan'))})
    def test_duplicate_completion_cannot_increase_throughput(self):
        m=Metrics([dict(id='a',time=0,road=1)])
        m.sample(1,1,{'a':dict(speed=1)},['a','a'])
        m.sample(2,1,{},['a'])
        self.assertEqual(m.report()['completed'],1)
        self.assertEqual(m.report()['censored_vehicle_time_s'],1.)
    def test_red_crossing_green_and_already_crossed(self):
        for color,expected in [('RED',1),('YELLOW',0),('GREEN',0)]:
            s=SafetyMetrics(); a=box(0,0,1,0); b=dict(a,front_stop_distance=-10)
            s.sample(1,1,{'a':a},{'a':b},{'a':color})
            s.sample(2,1,{'a':b},{'a':b},{'a':'RED'})
            self.assertEqual(s.report()['red_light_violations'],expected)
            self.assertEqual(s.report()['stop_line_crossings'],1)
            self.assertEqual(s.events[0]['time'],.5)
    def test_not_crossing_and_missing_signal(self):
        a=box(0,0,0,0); s=SafetyMetrics()
        s.sample(1,1,{'a':a},{'a':a},{'a':'RED'})
        self.assertEqual(s.report()['red_light_violations'],0)
        with self.assertRaises(ValueError):
            s.sample(2,1,{'a':a},{'a':dict(a,front_stop_distance=-1)},{})
    def test_perpendicular_ttc_hand_calculation(self):
        a=box(-10,0,2,0); b=box(0,-10,0,2,math.pi/2,lane_key='b')
        self.assertAlmostEqual(rectangle_ttc(a,b),4.25)
        self.assertFalse(polygons_overlap(a,b,4.249))
        self.assertTrue(polygons_overlap(a,b,4.251))
    def test_ttc_near_miss_receding_and_current_overlap(self):
        a=box(0,0,2,0)
        self.assertIsNone(rectangle_ttc(a,box(0,4,2,0)))
        self.assertIsNone(rectangle_ttc(a,box(10,0,3,0)))
        self.assertEqual(rectangle_ttc(a,box(0,0,0,0)),0.)
    def test_rotated_boxes_against_numerical_static_overlap(self):
        a=box(-4,-1,1,0,.3); b=box(3,-.7,-.5,0,-.4)
        t=rectangle_ttc(a,b)
        self.assertIsNotNone(t)
        self.assertFalse(polygons_overlap(a,b,t-1e-4))
        self.assertTrue(polygons_overlap(a,b,t+1e-4))
    def test_crossing_pair_exposure_does_not_double_count(self):
        a=box(-2,0,1,0); b=box(0,-2,0,1,math.pi/2,lane_key='b')
        s=SafetyMetrics()
        s.sample(.25,.25,{'a':a,'b':b},{'a':a,'b':b},{'a':'RED','b':'RED'})
        self.assertAlmostEqual(s.report()['crossing_ttc_below_1_5_pair_seconds'],.25)
    def test_wait_and_collision_dedup_invariant_to_sampling(self):
        for dt in (.05,.1,.25):
            m=Metrics([dict(id='a',time=0,road=1)])
            for i in range(round(2/dt)):
                m.sample((i+1)*dt,dt,{'a':dict(speed=0)},collisions=[(1,2),(2,1)])
            self.assertAlmostEqual(m.report()['waiting_vehicle_seconds'],2)
            self.assertEqual(m.report()['collision_pairs'],1)
            self.assertEqual(m.report()['stops'],1)

if __name__=='__main__':unittest.main()
