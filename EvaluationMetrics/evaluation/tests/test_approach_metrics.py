import unittest
from evaluation.approach_metrics import ApproachMetrics,union_length


def geometry():
    return dict(config=dict(queue_speed_mps=1,queue_gap_m=5,queue_head_distance_m=10,height_tolerance_m=2),
        lanes=[dict(id=str(lane),signal_ids=[str(lane)],geometry_status="complete",samples=[
            dict(x=-s,y=y,z=0,s=s,width=4,road_id=1,section_id=0,lane_id=lane) for s in range(51)])
            for lane,y in [(1,0),(2,4)]])


def car(vid,front,rear,y=0,lane=1,speed=0,z=0,width=2):
    return dict(vehicle_id=vid,x=-(front+rear)/2,y=y,z=z,road_id=1,section_id=0,lane_id=lane,
        vx=speed,vy=0,ax=2,ay=0,bbox_vertices_world_json=[[-s,y+w,z+h]
        for s in (front,rear) for w in (-width/2,width/2) for h in (0,2)])


class ApproachMetricsTest(unittest.TestCase):
    def test_survey_plane_distance_before_on_and_after_edge(self):
        surveyed=geometry()
        surveyed['lanes'][0]['stop_line']=dict(center=dict(x=0,y=0,z=0),normal_xy=[1,0])
        metrics=ApproachMetrics(surveyed)
        for distance in (.5,0.,-.5):
            metrics.sample([car(1,distance,distance+4)])
            detail=metrics.vehicle_details['1']
            self.assertAlmostEqual(detail['stop_line_front_distance_m'],distance)
            self.assertAlmostEqual(detail['stop_line_rear_distance_m'],distance+4)
        metrics=ApproachMetrics(geometry())
        metrics.sample([car(1,2,6)])
        self.assertNotIn('stop_line_front_distance_m',metrics.vehicle_details['1'])

    def test_three_different_lengths_and_slow_queue(self):
        # Hand-calculated straight lane: lengths 4/6/10 m, gaps 3/3 m.
        # Queue span [2,28] is 26 m; occupied union is 20 m, not 26 m.
        rows=[car(1,2,6,speed=.5),car(2,9,15,speed=.8),car(3,18,28,speed=1)]
        result=ApproachMetrics(geometry()).sample(rows)[0]
        self.assertEqual(result["queue_vehicle_count"],3)
        self.assertEqual(result["queue_head_id"],1)
        self.assertEqual(result["queue_tail_id"],3)
        self.assertEqual(result["queue_length_m"],26)
        self.assertAlmostEqual(result["spatial_occupancy"],.4)

    def test_queue_threshold_boundaries(self):
        cases=[
            ("speed_equal",[car(1,2,6,speed=1),car(2,9,15)],2),
            ("speed_above",[car(1,2,6,speed=1.001),car(2,9,15)],0),
            ("gap_equal",[car(1,2,6),car(2,11,17)],2),
            ("gap_above",[car(1,2,6),car(2,11.001,17.001)],1),
            ("head_equal",[car(1,10,14),car(2,17,23)],2),
            ("head_above",[car(1,10.001,14.001),car(2,17.001,23.001)],0),
            ("moving_middle",[car(1,2,6),car(2,9,15,speed=1.001),car(3,18,28)],1),
            ("isolated_tail",[car(1,2,6),car(2,9,15),car(3,21,31)],2),
        ]
        for label,rows,expected in cases:
            with self.subTest(label=label):
                result=ApproachMetrics(geometry()).sample(rows)[0]
                self.assertEqual(result["queue_vehicle_count"],expected)

    def test_cross_lane_long_vehicle_and_observed_tail_boundary(self):
        result=ApproachMetrics(geometry()).sample([car(1,2,12,y=1.8,width=2.5)])
        self.assertEqual([r["primary_vehicle_count"] for r in result],[1,0])
        self.assertEqual([r["spatial_occupancy"] for r in result],[.2,.2])
        # The rear exceeds 50 m, so coverage clips at 50 m. This is not a complete queue.
        row=ApproachMetrics(geometry()).sample([car(1,45,55)])[0]
        self.assertEqual(row["queue_vehicle_count"],0)
        self.assertAlmostEqual(row["spatial_occupancy"],.1)

    def test_vehicle_reference_matches_head_and_preserves_missing_box(self):
        metrics=ApproachMetrics(geometry())
        missing=car(2,10,14,lane=2,y=4)
        missing["bbox_vertices_world_json"]=""
        results=metrics.sample([car(1,2,6)])
        self.assertEqual(metrics.vehicle_details["1"]["front_reference_distance_m"],results[0]["lane_head_distance_m"])
        self.assertEqual(metrics.vehicle_details["1"]["origin_reference_distance_m"],4)
        metrics.sample([missing,car(3,2,6,lane=99)])
        self.assertNotIn("front_reference_distance_m",metrics.vehicle_details["2"])
        self.assertEqual(metrics.vehicle_details["3"]["geometry_match"],"unmapped")

    def test_head_queue_chain_and_independent_acceleration(self):
        result=ApproachMetrics(geometry()).sample([car(3,30,34),car(2,9,15),car(1,2,6)])[0]
        self.assertEqual(result["lane_head_id"],1)
        self.assertEqual(result["queue_head_id"],1)
        self.assertEqual(result["queue_tail_id"],2)
        self.assertEqual(result["queue_vehicle_count"],2)
        self.assertEqual(result["queue_length_m"],13)
        self.assertEqual(result["lane_head_distance_m"],2)
        self.assertEqual(result["lane_head_longitudinal_accel_mps2"],2)
        self.assertAlmostEqual(result["spatial_occupancy"],14/50)

    def test_moving_head_blocks_false_queue_behind_it(self):
        result=ApproachMetrics(geometry()).sample([car(1,2,6,speed=3),car(2,8,12)])[0]
        self.assertEqual(result["lane_head_id"],1)
        self.assertEqual(result["queue_vehicle_count"],0)
        self.assertIsNone(result["queue_head_id"])

    def test_cross_lane_overlap_separate_from_primary_lane(self):
        results=ApproachMetrics(geometry()).sample([car(1,2,6,y=1.8,width=2)])
        self.assertEqual([r["primary_vehicle_count"] for r in results],[1,0])
        self.assertEqual([r["spatial_occupancy"] for r in results],[4/50,4/50])

    def test_union_clipping_and_height(self):
        self.assertEqual(union_length([(1,5),(2,4),(4,8)]),7)
        results=ApproachMetrics(geometry()).sample([car(1,45,55),car(2,48,52),car(3,5,10,z=10)])
        self.assertAlmostEqual(results[0]["spatial_occupancy"],5/50)
        self.assertEqual(results[0]["primary_vehicle_count"],2)

    def test_missing_geometry_unknown_not_zero(self):
        row=car(1,2,6); row["bbox_vertices_world_json"]=""
        result=ApproachMetrics(geometry()).sample([row])[0]
        self.assertEqual(result["metric_status"],"missing_vehicle_geometry")
        self.assertIsNone(result["queue_vehicle_count"])
        self.assertIsNone(result["spatial_occupancy"])

    def test_empty_lane_outputs_zero_with_unknown_heads(self):
        for result in ApproachMetrics(geometry()).sample([]):
            self.assertEqual(result["queue_vehicle_count"],0)
            self.assertEqual(result["spatial_occupancy"],0)
            self.assertIsNone(result["lane_head_id"])
