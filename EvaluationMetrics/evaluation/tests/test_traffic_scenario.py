import math
import json
import random
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from collections import Counter
from types import SimpleNamespace as NS

import carla
import spawn_junction_traffic as spawn
from traffic_scenario import LanePath, fleet_plan, measure_box, quotas, fit_first_distance


class Waypoint:
    is_junction = False
    road_id, section_id, lane_id = 10, 0, -1

    def __init__(self, station=0, radius=None, end=200):
        self.station, self.radius, self.end = station, radius, end

    @property
    def transform(self):
        d, r = self.station, self.radius
        if r is None:
            return carla.Transform(carla.Location(x=-d))
        angle = d/r
        return carla.Transform(carla.Location(x=-r*math.sin(angle), y=r*(1-math.cos(angle))),
                               carla.Rotation(yaw=-math.degrees(angle)))

    def previous(self, distance):
        d = self.station + distance
        return [Waypoint(d, self.radius, self.end)] if d < self.end else []


class Blueprint:
    def __init__(self, name, wheels=4):
        self.id, self.wheels, self.role = name, wheels, None

    def has_attribute(self, name):
        return name == "role_name" or (name == "number_of_wheels" and self.wheels is not None)

    def get_attribute(self, name):
        return self.wheels

    def set_attribute(self, name, value):
        self.role = value


class TrafficScenarioTest(unittest.TestCase):
    def test_short_lane_fits_mixed_fleet_without_changing_gap(self):
        path = LanePath(Waypoint(end=28.1))
        boxes = [carla.BoundingBox(carla.Location(),carla.Vector3D(2.5,1,1)),
                 carla.BoundingBox(carla.Location(x=.5),carla.Vector3D(4.5,1,1))]
        first = fit_first_distance(path,boxes,18,2)
        self.assertLess(first,18)
        _,rear = path.interval(boxes[0],path.transform(first))
        second = path.following_distance(rear,boxes[1],2)
        front,end = path.interval(boxes[1],path.transform(second))
        self.assertAlmostEqual(front-rear,2,delta=.01)
        self.assertLess(end,path.stations[-1])
        self.assertEqual(fit_first_distance(LanePath(Waypoint()),boxes,18,2),18)
        with self.assertRaisesRegex(ValueError,'reduce --vehicles-per-approach'):
            fit_first_distance(LanePath(Waypoint(end=10)),boxes,18,2)

    def test_generator_builds_full_fleet_before_release_and_cleans_failures(self):
        for fail_second in (False, True):
            with tempfile.TemporaryDirectory() as directory:
                actors, released = {}, []
                frame, sequence = [0], [0]
                bps = [Blueprint("vehicle.tesla.model3"), Blueprint("vehicle.audi.etron")]
                def snapshot():
                    states = {key: NS(get_transform=actor.get_transform) for key,actor in actors.items()}
                    return NS(frame=frame[0], timestamp=NS(elapsed_seconds=frame[0]*.05),
                              find=lambda key: states.get(key))
                def advance(timeout):
                    frame[0] += 1
                    return snapshot()
                def make_actor(bp,transform):
                    finals = [actor for actor in actors.values() if actor.role != "junction_geometry_probe"]
                    if fail_second and finals and bp.role != "junction_geometry_probe":
                        return None
                    sequence[0] += 1
                    key = sequence[0]
                    actor = NS(id=key, type_id=bp.id, role=bp.role, is_alive=True,
                        bounding_box=carla.BoundingBox(carla.Location(x=.5),carla.Vector3D(2,1,1)),
                        set_simulate_physics=lambda value: None, get_transform=lambda: transform,
                        get_velocity=lambda: carla.Vector3D(), get_traffic_light=lambda: None,
                        get_traffic_light_state=lambda: "Red")
                    actor.destroy = lambda: actors.pop(key,None) is not None
                    actor.set_autopilot = lambda *args: released.append(len(actors))
                    actors[key] = actor
                    return actor
                light = NS(id=99,get_pole_index=lambda:1,get_stop_waypoints=lambda:[Waypoint()])
                world = NS(get_map=lambda:NS(name="TestMap"),get_traffic_lights_in_junction=lambda j:[light],
                    get_blueprint_library=lambda:NS(filter=lambda pattern:bps),try_spawn_actor=make_actor,
                    get_snapshot=snapshot,wait_for_tick=advance,get_actors=lambda:list(actors.values()))
                tm = NS(set_random_device_seed=lambda seed:None,ignore_lights_percentage=lambda *a:None,
                        auto_lane_change=lambda *a:None)
                def batch(commands,tick):
                    self.assertFalse(tick)
                    for key in commands:
                        actors.pop(key,None)
                    return [NS(has_error=lambda:False) for _ in commands]
                client = NS(set_timeout=lambda t:None,get_world=lambda:world,
                            get_trafficmanager=lambda port:tm,apply_batch_sync=batch)
                argv=["spawn", "--junction-id","1427", "--output",str(Path(directory)/"telemetry.csv"),"--duration-seconds",".1"]
                with patch.object(spawn.carla,"Client",return_value=client), \
                     patch.object(spawn.carla.command,"DestroyActor",side_effect=lambda key:key), \
                     patch.object(spawn.signal,"signal"), patch("sys.argv",argv), patch("builtins.print"):
                    if fail_second:
                        with self.assertRaisesRegex(RuntimeError,"could not spawn"):
                            spawn.main()
                    else:
                        spawn.main()
                manifest=json.loads(next(Path(directory).glob("spawn_manifest_*.json")).read_text())
                self.assertEqual(actors,{})
                if fail_second:
                    self.assertEqual(released,[])
                    self.assertEqual(manifest["status"],"failed")
                    self.assertEqual(manifest["plan"][1]["status"],"spawn_failed")
                else:
                    self.assertEqual(released,[2,2])
                    self.assertEqual(manifest["actual_counts"],dict(sedan=1,suv=1))
                    self.assertAlmostEqual(manifest["vehicles"][1]["measured_gap_m"],2,delta=.05)
                    self.assertEqual(manifest["status"],"completed")

    def test_quotas_preserve_total_and_sedan_priority(self):
        weights = dict(sedan=60, suv=20, truck=15, bus=5)
        self.assertEqual(quotas(24,weights),dict(sedan=14,suv=5,truck=4,bus=1))
        self.assertEqual(quotas(100,weights),weights)
        self.assertEqual(quotas(1,weights)["sedan"],1)
        for weights in [dict(sedan=-1),dict(sedan=0),dict(sedan=float("nan"))]:
            with self.assertRaises(ValueError):
                quotas(24,weights)

    def test_fixed_plan_reproducible_and_excludes_two_wheels(self):
        config = dict(categories=dict(sedan=dict(weight=60,blueprints=["a","b"]),
                                      bus=dict(weight=40,blueprints=["bus"])))
        blueprints=[Blueprint("a"),Blueprint("b",2),Blueprint("bus",6)]
        plan, counts = fleet_plan(blueprints,10,config,random.Random(17))
        self.assertEqual(Counter(category for category,_ in plan), counts)
        self.assertNotIn("b",[bp.id for _,bp in plan])
        repeated,_ = fleet_plan(blueprints,10,config,random.Random(17))
        self.assertEqual([(key,bp.id) for key,bp in plan],[(key,bp.id) for key,bp in repeated])
        with self.assertRaises(ValueError):
            fleet_plan([Blueprint("a")],10,config,random.Random(17))
        with self.assertRaises(ValueError):
            fleet_plan([Blueprint("a",None)],1,config,random.Random(17),"a")

    def test_gap_uses_both_lengths_and_bbox_offset(self):
        path = LanePath(Waypoint(),limit=100)
        leader = carla.BoundingBox(carla.Location(x=1),carla.Vector3D(2,1,1))
        follower = carla.BoundingBox(carla.Location(x=.5),carla.Vector3D(5,1,1))
        _, rear = path.interval(leader,path.transform(20))
        distance = path.following_distance(rear,follower,2)
        self.assertAlmostEqual(rear,21,places=4)
        self.assertAlmostEqual(distance,28.5,places=4)
        front,_ = path.interval(follower,path.transform(distance))
        self.assertAlmostEqual(front-rear,2,places=4)

    def test_curved_lane_gap_and_end_rejection(self):
        path = LanePath(Waypoint(radius=50),limit=70)
        box = carla.BoundingBox(carla.Location(),carla.Vector3D(3,1,1))
        _,rear = path.interval(box,path.transform(15))
        distance = path.following_distance(rear,box,2)
        front,_ = path.interval(box,path.transform(distance))
        self.assertAlmostEqual(front-rear,2,delta=.011)
        with self.assertRaises(ValueError):
            path.following_distance(69,box,2)

    def test_probe_owned_until_confirmed_destruction(self):
        bp = Blueprint("a")
        box = object()
        probe = NS(set_simulate_physics=lambda enabled: None,bounding_box=box,destroy=lambda: True)
        world = NS(try_spawn_actor=lambda bp,transform: probe)
        owned=[]
        self.assertIs(measure_box(world,bp,LanePath(Waypoint()),18,owned,lambda: None),box)
        self.assertEqual(owned,[])
        probe.destroy = lambda: False
        with self.assertRaises(RuntimeError):
            measure_box(world,bp,LanePath(Waypoint()),18,owned,lambda: None)
        self.assertEqual(owned,[probe])
