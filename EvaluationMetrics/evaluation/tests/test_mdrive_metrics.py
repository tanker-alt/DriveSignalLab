import copy
import unittest
from types import SimpleNamespace as NS
from evaluation.mdrive_metrics import MDriveMetrics


def actor(vid, x, speed=1):
    return dict(id=vid, x=x, y=0., yaw=0., vel_x=speed, vel_y=0.,
                accel_x=0., accel_y=0., ang_vel=0., extent_x=1., extent_y=.5)


def packet(frame, second_x=20.):
    return dict(frame=frame, sim_time=frame*.1, world_snapshot=dict(frame=frame,t=frame*.1,
        complete=True, actors=[actor(1,frame*.1),actor(2,second_x,0)],
        traffic_lights={},traffic_light_polygons={}))


class MDriveTests(unittest.TestCase):
    def prepared(self):
        route=[NS(x=0.,y=0.),NS(x=30.,y=0.)]
        road_map=NS(get_waypoint=lambda *a,**kw:NS(road_id=1,lane_id=-1,lane_type=1,is_junction=False))
        e=MDriveMetrics([1],road_map,{1:route})
        module=e._load()
        # Only the CARLA geometry interface is stubbed. All scoring is real upstream code.
        module.carla=NS(Location=lambda **kw:NS(**kw),LaneType=NS(Driving=1,Bidirectional=2,Shoulder=4),
                        TrafficLightState=NS(Red=0))
        return e,module,route

    def test_exact_upstream_parity_and_other_ego_preserved(self):
        e,module,route=self.prepared()
        for frame in range(1,11):e.sample(packet(frame))
        (trace,worlds,polys),reason=e._trace('1')
        self.assertIsNone(reason)
        self.assertEqual([a['id'] for a in worlds[0]['actors']],[2])
        expected=module.compute_pdm_route_metrics(None,trace,pdm_world_trace=worlds,
            pdm_tl_polygons=polys,config=NS(trajectory=route,multi_traj=[route]),ego_id=0)
        result=e.close()
        self.assertEqual(result['vehicles']['1']['pdm'],expected)
        self.assertTrue(result['source_sha256'])
        self.assertFalse(result['vehicles']['1']['leaderboard']['available'])

    def test_collision_red_light_and_comfort_use_upstream(self):
        e,module,route=self.prepared()
        for frame in range(1,11):
            p=packet(frame,second_x=frame*.1+.5)
            p['world_snapshot']['actors'][0]['accel_x']=10.
            p['world_snapshot']['traffic_lights']={7:0}
            p['world_snapshot']['traffic_light_polygons']={7:[[-2,-2],[-2,2],[3,2],[3,-2]]}
            e.sample(p)
        result=e.close()['vehicles']['1']['pdm']
        self.assertEqual(result['no_at_fault_collisions'],0.)
        self.assertEqual(result['traffic_light_compliance'],0.)
        self.assertEqual(result['history_comfort'],0.)
        self.assertEqual(result['pdm_score'],0.)

    def test_missing_frames_or_world_are_unavailable(self):
        for kind in ('gap','world','route'):
            e,_,_=self.prepared()
            e.sample(packet(1))
            p=packet(3 if kind=='gap' else 2)
            if kind=='world':p['world_snapshot']['complete']=False
            if kind=='route':e.routes={}
            e.sample(p)
            self.assertFalse(e.close()['vehicles']['1']['pdm']['available'])

    def test_original_driving_score_preserved(self):
        e,_,_=self.prepared()
        record=dict(scores=dict(score_route=80.,score_penalty=.6,score_composed=48.),
                    infractions=dict(collisions_vehicle=['event']),status='Failed')
        e.set_route_record(1,record)
        self.assertEqual(e.close()['vehicles']['1']['leaderboard'],record)
