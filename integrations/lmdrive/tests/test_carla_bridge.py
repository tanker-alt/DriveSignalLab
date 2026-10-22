import unittest
from types import SimpleNamespace
from integrations.lmdrive.overlay.common.fixed_traffic_light import (CarlaTrafficLightRuntime, TrafficLightAgent,
                                        project_arc, camera_signal_at_frame)
from integrations.lmdrive.overlay.common.background_traffic import build_demand


class FixedSignalTests(unittest.TestCase):
    def test_exited_background_actor_is_counted_in_sink_window(self):
        class Actors(list):
            def filter(self, _pattern):
                return self
        actor = SimpleNamespace(
            id=42, is_alive=True,
            get_location=lambda: SimpleNamespace(x=3, y=4),
            get_velocity=lambda: SimpleNamespace(x=0, y=0, z=0))
        background = SimpleNamespace(
            states={0: {'exited': True, 'removed': False}},
            actors={0: actor}, demand=[{'movement': 0}])
        runtime = CarlaTrafficLightRuntime.__new__(CarlaTrafficLightRuntime)
        runtime.world = SimpleNamespace(get_actors=lambda: Actors())
        runtime.map = None
        runtime.mappings = [{}]
        runtime.background_runtime = background
        state = runtime.collect_lane_state()
        self.assertEqual(state['outgoing'], [1])
        self.assertEqual(state['vehicle_ids'][0]['outgoing'], [42])
        self.assertEqual(state['background_vehicles'][0]['movement'], 0)

    def test_manifest_movement_overrides_wrong_geometric_exit_bucket(self):
        class Actors(list):
            def filter(self, _pattern):
                return self
        actor = SimpleNamespace(
            id=42, is_alive=True,
            get_location=lambda: SimpleNamespace(x=1, y=0),
            get_velocity=lambda: SimpleNamespace(x=0, y=0, z=0))
        wp = SimpleNamespace(road_id=7, lane_id=1)
        point_0 = SimpleNamespace(x=0, y=0)
        point_1 = SimpleNamespace(x=2, y=0)
        mapping = {'stop': SimpleNamespace(road_id=1, lane_id=1),
                   'stop_arc': 0, 'points': [point_0, point_1], 'cumulative': [0, 2],
                   'exit_wp': wp}
        background = SimpleNamespace(
            states={0: {'exited': True, 'removed': False}},
            actors={0: actor}, demand=[{'movement': 1}])
        runtime = CarlaTrafficLightRuntime.__new__(CarlaTrafficLightRuntime)
        runtime.world = SimpleNamespace(get_actors=lambda: Actors([actor]))
        runtime.map = SimpleNamespace(get_waypoint=lambda _location: wp)
        runtime.mappings = [mapping, mapping]
        runtime.background_runtime = background
        state = runtime.collect_lane_state()
        self.assertEqual(state['outgoing'], [0, 1])

    def test_background_demand_profiles_are_balanced_and_reproducible(self):
        expected = {'none': 0, 'low': 4, 'medium': 10, 'high': 20}
        for profile, total in expected.items():
            first = build_demand(profile, 2026)
            self.assertEqual(first, build_demand(profile, 2026))
            self.assertEqual(len(first), total)
            self.assertEqual(sum(x['movement'] == 0 for x in first), total // 2)
            self.assertEqual(sum(x['movement'] == 1 for x in first), total // 2)
            self.assertEqual([x['demand_id'] for x in first], list(range(total)))

    def test_background_count_override(self):
        demand = build_demand('low', 1, count_override=3)
        self.assertEqual(len(demand), 6)
        self.assertEqual(sorted(x['rank'] for x in demand), [1, 1, 2, 2, 3, 3])
        self.assertEqual(sorted(x['scheduled_s'] for x in demand), [0, 0, 6, 6, 12, 12])

    def test_background_status_requires_full_demand_to_exit(self):
        from integrations.lmdrive.overlay.common.background_traffic import BackgroundTrafficRuntime
        runtime = BackgroundTrafficRuntime.__new__(BackgroundTrafficRuntime)
        runtime.profile = 'low'
        runtime.demand = [{}, {}]
        runtime.actors = {0: object(), 1: object()}
        runtime.pending = {}
        runtime.states = {0: {'exited': True}, 1: {'exited': False}}
        self.assertFalse(runtime.status()['drained'])
        runtime.states[1]['exited'] = True
        self.assertTrue(runtime.status()['drained'])

    def test_clearance_at_each_phase_boundary(self):
        agent=TrafficLightAgent()
        expected=[(0,'initial_all_red',None,'Red'),(5.99,'initial_all_red',None,'Red'),
                  (6,'ego_0_green',0,'Green'),(16,'ego_0_yellow',0,'Yellow'),
                  (19,'all_red_after_0',None,'Red'),(21,'ego_1_green',1,'Green'),
                  (31,'ego_1_yellow',1,'Yellow'),(34,'all_red_after_1',None,'Red'),
                  (36,'ego_0_green',0,'Green')]
        for time,phase,ego,state in expected:
            with self.subTest(time=time):
                decision=agent.decide(time)
                self.assertEqual((decision['phase'],decision['active_ego'],decision['state']),(phase,ego,state))
                self.assertGreater(decision['remaining_s'],0)

    def test_nonpositive_phase_duration_rejected(self):
        for key in ('green_seconds','yellow_seconds','clearance_seconds'):
            with self.assertRaises(ValueError):
                TrafficLightAgent(**{key:0})

    def test_distance_follows_turning_route(self):
        point=lambda x,y:SimpleNamespace(x=x,y=y)
        route=[point(0,0),point(10,0),point(10,10)]
        self.assertEqual(project_arc(point(10,7),route,[0,10,20]),17)
        self.assertEqual(project_arc(point(4,1),route,[0,10,20]),4)

    def test_camera_gap_only_bridged_with_unchanged_phase(self):
        left={'carla_frame':10,'simulation_time':1.0,'elapsed_s':.5,'phase':'initial_all_red',
              'phase_remaining_s':5.5,'actual_states':{1:'Red'}}
        right=dict(left,carla_frame=12,simulation_time=1.1,elapsed_s=.6,phase_remaining_s=5.4)
        history={10:left,12:right}
        result=camera_signal_at_frame(history,11,1)
        self.assertEqual(result['alignment_method'],'stable_interval')
        self.assertAlmostEqual(result['simulation_time'],1.05)
        history[12]=dict(right,phase='ego_0_green',actual_states={1:'Green'})
        self.assertIsNone(camera_signal_at_frame(history,11,1))


if __name__=='__main__':
    unittest.main()
