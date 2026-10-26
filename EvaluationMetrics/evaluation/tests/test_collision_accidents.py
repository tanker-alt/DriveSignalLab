import unittest
from evaluation.collision_accidents import aggregate_accidents


def cb(a,b,t,x=0,y=0,kind='vehicle.test'):
    return dict(event='collision_callback',vehicle_id=a,other_actor_id=b,
                other_actor_type=kind,sim_time=t,sensor_location=dict(x=x,y=y))


class AccidentTest(unittest.TestCase):
    def run_case(self,rows,**kw):
        return aggregate_accidents(rows,(0,0),**kw)

    def test_chain_bilateral_duplicates_and_independent(self):
        events,s=self.run_case([cb(1,2,1),cb(2,1,1),cb(2,3,1.3),cb(4,5,1.3)])
        self.assertEqual(s['accident_count'],2)
        self.assertEqual(events[0]['vehicle_ids'],[1,2,3])

    def test_later_or_distant_collision_separate(self):
        self.assertEqual(len(self.run_case([cb(1,2,1),cb(2,3,2.01)])[0]),2)
        self.assertEqual(len(self.run_case([cb(1,2,1),cb(2,3,1.1,x=30)])[0]),2)

    def test_bridge_merges_existing_groups(self):
        rows=[cb(1,2,1),cb(3,4,1),cb(2,3,1.1)]
        self.assertEqual(len(self.run_case(list(reversed(rows)))[0]),1)

    def test_region_boundary_and_unknown(self):
        rows=[cb(1,2,1,x=120),cb(3,4,2,x=120.01),cb(5,6,3)]
        del rows[2]['sensor_location']
        events,s=self.run_case(rows)
        self.assertEqual(len(events),1)
        self.assertEqual(s['unknown_source_lines'],[3])
        self.assertEqual(s['outside_callback_source_lines'],[2])

    def test_shared_static_object_does_not_merge(self):
        rows=[cb(1,100,1,kind='static.wall'),cb(2,100,1,kind='static.wall')]
        self.assertEqual(len(self.run_case(rows)[0]),2)

    def test_config_and_empty(self):
        self.assertEqual(self.run_case([])[1]['accident_count'],0)
        self.assertEqual(len(self.run_case([cb(1,2,1),cb(2,3,1.5)],gap=.2)[0]),2)
        for kw in [dict(radius=-1),dict(distance=float('nan')),dict(gap=-1)]:
            with self.assertRaises(ValueError): self.run_case([],**kw)

class ContactContinuityTest(unittest.TestCase):
    def frames(self):
        from shapely.geometry import box
        return {f:dict(time=f*.1,bodies={1:(box(0,0,2,2),0,1),2:(box(1,0,3,2),0,1)}) for f in range(10,26)}

    def events(self,frames):
        a,b=cb(1,2,1),cb(1,2,2.5)
        a['frame']=10;b['frame']=25
        return aggregate_accidents([a,b],(0,0),contact_frames=frames)

    def test_contact_bridge_across_silent_callbacks(self):
        events,audit=self.events(self.frames())
        self.assertEqual(len(events),1)
        self.assertEqual(len(audit['continuous_contact_bridges']),1)

    def test_separation_missing_or_vertical_distance_prevents_bridge(self):
        from shapely.geometry import box
        for change in ('missing','separated','vertical','timestamp'):
            frames=self.frames()
            if change=='missing':del frames[17]
            if change=='separated':frames[17]['bodies'][2]=(box(4,0,6,2),0,1)
            if change=='vertical':frames[17]['bodies'][2]=(box(1,0,3,2),3,4)
            if change=='timestamp':frames[25]['time']=3.5
            self.assertEqual(len(self.events(frames)[0]),2,change)

    def test_no_evidence_preserves_time_gap_rule(self):
        self.assertEqual(len(self.events(None)[0]),2)


class StaticUnknownTest(unittest.TestCase):
    def test_static_unknown_joins_only_through_known_vehicle(self):
        events,audit=aggregate_accidents([cb(1,2,1),cb(1,0,1.1,kind='static.unknown'),cb(3,0,1.1,kind='static.unknown')],(0,0))
        self.assertEqual(len(events),2)
        self.assertEqual(events[0]['actor_ids'],[1,2])
        self.assertEqual(events[0]['unidentified_static_source_lines'],[2])
        self.assertEqual(audit['unknown_source_lines'],[])
