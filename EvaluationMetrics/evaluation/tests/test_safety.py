import copy
import json
import tempfile
import unittest
from pathlib import Path
from shapely.geometry import Polygon, box, mapping
from shapely.ops import unary_union
from evaluation.safety import SafetyEvaluator
from evaluation.safety_config import load_config
from evaluation.data_products import AssessmentWriter
from evaluation.observations import observation_packet


def row(x=-4, y=0, speed=0, vx=0, yaw=0, vid=1):
    # 4m long, 2m wide synthetic model with independently known front x+2.
    import math
    a = math.radians(yaw)
    points = [[x+u*math.cos(a)-v*math.sin(a), y+u*math.sin(a)+v*math.cos(a), z]
              for u in (-2,2) for v in (-1,1) for z in (0,1)]
    return dict(vehicle_id=vid, x=x, y=y, z=0, yaw=yaw, pitch=0,
                speed_mps=speed/3.6, vx=vx, vy=0, vz=0, road_id=1, lane_id=-1,
                bbox_vertices_world_json=json.dumps(points))


def geometry(region=None):
    return dict(driving_region=mapping(region if region is not None else box(-100,-5,100,5)),
                stop_lines=[dict(id='stop', road_id=1, lane_id=-1, signal_opendrive_id='s',
                    center_world_m=[0,0,0], forward_world=[1,0,0],
                    start_world_m=[0,-2,0], end_world_m=[0,2,0], lane_width_m=4)])


class SafetyTests(unittest.TestCase):
    def evaluator(self, geo=None):
        config=load_config()
        config['geometry']['boundary_uncertainty_m']=0
        return SafetyEvaluator(config, geo)

    def test_speeding_hysteresis_confirmation_and_two_events(self):
        e=self.evaluator()
        for frame, (now,speed) in enumerate([(0,60),(.1,66),(.4,66),(.6,66),(.7,63),(.8,60),(.9,66),(1.4,66),(1.5,59)],1):
            e.sample(frame,now,[row(speed=speed)])
        events=[x for x in e.events if x['metric']=='speeding']
        self.assertEqual([x['status'] for x in events],['completed','completed'])
        self.assertEqual(events[0]['end_sim_time'],.8)
        self.assertAlmostEqual(events[0]['observed_duration_s'],.7)
        self.assertEqual(e.close()['speeding']['value'],2)

    def test_speeding_equal65_and_short_excursion_do_not_count(self):
        e=self.evaluator()
        for f,(t,v) in enumerate([(0,60),(.1,65),(.7,65),(.8,66),(1.1,66),(1.2,64),(1.8,64),(1.9,60)],1):
            e.sample(f,t,[row(speed=v)])
        self.assertEqual(e.close()['speeding']['value'],0)

    def test_reverse_stationary_end_and_body_heading(self):
        e=self.evaluator()
        for f,(t,v) in enumerate([(0,1),(.1,-1),(.4,-1),(.6,-1),(.7,0)],1):
            e.sample(f,t,[row(vx=v)])
        self.assertEqual(e.close()['reversing']['value'],1)
        # A west-facing vehicle moving west is forward, not reversing.
        e=self.evaluator()
        for f in range(1,5):e.sample(f,f*.5,[row(vx=-2,yaw=180)])
        self.assertEqual(e.close()['reversing']['value'],0)

    def test_gap_missing_vehicle_missing_data_and_end_censor(self):
        for interruption in ('gap','missing','data','end'):
            e=self.evaluator()
            e.sample(1,0,[row(speed=60)])
            e.sample(2,.1,[row(speed=66)])
            e.sample(3,.6,[row(speed=66)])
            if interruption=='gap':e.sample(5,.7,[row(speed=60)])
            if interruption=='missing':e.sample(4,.7,[])
            if interruption=='data':e.sample(4,.7,[dict(vehicle_id=1)])
            report=e.close()['speeding']
            self.assertEqual(report['value'],0)
            self.assertEqual(report['censored_events'],1)

    def test_initial_violating_sample_is_left_censored(self):
        e=self.evaluator()
        e.sample(1,0,[row(speed=66)])
        e.sample(2,.5,[row(speed=66)])
        e.sample(3,.6,[row(speed=60)])
        self.assertEqual(e.close()['speeding']['value'],0)
        self.assertEqual(e.events[0]['status'],'censored')







    def test_config_rejected_and_override_used(self):
        for key,value in [('limit_kmh',0),('limit_kmh',float('nan')),('tolerance_kmh',-1)]:
            c=load_config();c['speeding'][key]=value
            with self.assertRaises(ValueError):load_config(c)
        c=load_config();c['speeding']['limit_kmh']=40
        e=SafetyEvaluator(load_config(c))
        for f,(t,v) in enumerate([(0,40),(.1,46),(.6,46),(.7,40)],1):e.sample(f,t,[row(speed=v)])
        self.assertEqual(e.close()['speeding']['value'],1)


    def test_jsonl_replay_preserves_saved_config_and_counts(self):
        from evaluation.safety_replay import replay
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            config=load_config();config['speeding']['limit_kmh']=40
            writer=AssessmentWriter(root,config)
            frames=[]
            for f,(t,v) in enumerate([(0,40),(.1,46),(.6,46),(.7,40)],1):
                packet=dict(frame=f,sim_time=t,vehicles=[row(speed=v)],signals={})
                frames.append(packet)
                writer.sample(f,t,packet['vehicles'],[],[],{},packet['signals'])
            original=writer.close('completed',{})
            (root/'assessment/safety_input.jsonl').write_text('\n'.join(json.dumps(p) for p in frames),encoding='utf-8')
            output=replay(root)
            result=json.loads((output/'assessment/evaluation_report.json').read_text(encoding='utf-8'))
            self.assertEqual(result['safety_config']['speeding']['limit_kmh'],40)
            self.assertEqual(result['Safety'],original['Safety'])
            cut=replay(root,start_frame=2)
            report=json.loads((cut/'assessment/evaluation_report.json').read_text(encoding='utf-8'))
            self.assertEqual(report['Safety']['speeding']['value'],0)
            self.assertEqual(report['Safety']['speeding']['censored_events'],1)

    def test_map_geometry_excludes_sidewalk_and_pavement(self):
        try:
            import carla
        except ImportError:
            self.skipTest('Optional CARLA map parser not installed')
        from evaluation.safety_config import map_geometry
        from shapely.geometry import shape, Point
        root=(Path(__file__).resolve().parents[3] / 'InformationCollector').parent/'Maps/MapDataset/maps'
        for name,count in [('cross3',12),('t3_north',9)]:
            folder=root/name
            road_map=carla.Map(name,(folder/'map.xodr').read_bytes().decode('utf-8'))
            geo=map_geometry(road_map,load_config(),folder)
            region=shape(geo['driving_region'])
            self.assertEqual(len(geo['stop_lines']),count)
            self.assertTrue(region.covers(Point(-40,1.75)))
            self.assertFalse(region.covers(Point(-40,11.5)))
            self.assertFalse(region.covers(Point(21,21)))
            self.assertEqual(geo['driving_lane_count'],36 if name=='cross3' else 27)





class AllVehicleSafetyTests(unittest.TestCase):
    def test_map_line_and_multiple_vehicles(self):
        e=SafetyEvaluator(load_config(),geometry())
        for f,x in enumerate((-5,-3,-1,2),1):
            e.sample(f,f*.1,[row(x=x,vid=1),row(x=x,vid=2)],{'s':'Red'})
        self.assertEqual(e.close()['red_light']['value'],2)

    def test_gap_and_signal_change_not_false_red(self):
        for last_frame,last_state in ((3,'Red'),(2,'Green')):
            e=SafetyEvaluator(load_config(),geometry())
            e.sample(1,0,[row(x=-3)],{'s':'Red'})
            e.sample(last_frame,.1,[row(x=0)],{'s':last_state})
            self.assertEqual(e.close()['red_light']['value'],0)

    def test_outside_duration_is_one_event_not_frames(self):
        e=SafetyEvaluator(load_config(),geometry())
        for f,y in enumerate((0,6,6,6,0),1):e.sample(f,f*.1,[row(y=y)])
        self.assertEqual(e.close()['out_of_bounds']['value'],1)

    def test_wrong_way_is_lane_velocity_not_body_reverse(self):
        e=SafetyEvaluator(load_config(),geometry())
        for f,v in enumerate((1,-2,-2,-2,1),1):
            r=row(vx=v,yaw=180);r.update(legal_direction_status='unique_lane',legal_forward_x=1,legal_forward_y=0)
            e.sample(f,f*.3,[r])
        self.assertEqual(e.close()['wrong_way']['value'],1)

class ReverseNoiseTest(unittest.TestCase):
    def test_stopped_velocity_noise_is_not_reverse(self):
        e=SafetyEvaluator(load_config())
        for f in range(30):e.sample(f,f*.1,[row(vx=-.002)])
        self.assertEqual(e.close()['reversing']['value'],0)

if __name__=='__main__':unittest.main()
