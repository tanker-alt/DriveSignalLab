"""Configurable fleet quotas and measured initial bumper gaps.

No ticking or Traffic Manager ownership here. Geometry uses a sampled upstream
lane centerline; the gap is the free longitudinal arc interval between projected
bounding boxes, not actor-origin separation or a runtime following policy.
"""
import bisect
import math
from collections import Counter


PROBE_ROLE = "junction_geometry_probe"


def quotas(total, weights):
    if total < 0 or not weights or any(not math.isfinite(w) or w < 0 for w in weights.values()):
        raise ValueError("Fleet weights must be finite and nonnegative")
    weight_sum = sum(weights.values())
    if weight_sum <= 0:
        raise ValueError("At least one fleet weight must be positive")
    exact = {key: total * weight / weight_sum for key, weight in weights.items()}
    counts = {key: int(math.floor(value)) for key, value in exact.items()}
    order = sorted(weights, key=lambda key: (-(exact[key] - counts[key]), key))
    for key in order[:total - sum(counts.values())]:
        counts[key] += 1
    return counts


def fleet_plan(blueprints, total, config, rng, explicit="random"):
    # Unknown wheel counts are excluded as well: no unverified two-wheel actors.
    available = {bp.id: bp for bp in blueprints if bp.has_attribute("number_of_wheels")
                 and int(bp.get_attribute("number_of_wheels")) >= 4}
    if explicit != "random":
        if explicit not in available:
            raise ValueError("Requested four-wheel-or-more blueprint unavailable: " + explicit)
        return [("explicit", available[explicit])] * total, {"explicit": total}
    categories = config["categories"]
    counts = quotas(total, {key: item["weight"] for key, item in categories.items()})
    seen, pools = set(), {}
    for key, item in categories.items():
        names = item["blueprints"]
        if len(set(names)) != len(names) or seen.intersection(names):
            raise ValueError("A blueprint must belong to exactly one category")
        seen.update(names)
        pools[key] = [available[name] for name in sorted(names) if name in available]
        if counts[key] and not pools[key]:
            raise ValueError("No available blueprint for requested category: " + key)
    plan = [(key, rng.choice(pools[key])) for key in counts for _ in range(counts[key])]
    rng.shuffle(plan)
    return plan, counts


def unique_previous(waypoint, distance):
    candidates = waypoint.previous(distance)
    if len(candidates) != 1 or candidates[0].is_junction:
        raise ValueError("Upstream lane ends, branches, or crosses another junction")
    return candidates[0]


class LanePath:
    """A finite, unbranched upstream path; no silent selection at forks."""
    def __init__(self, stop, limit=150.0, step=.5):
        self.waypoints, self.stations = [stop], [0.0]
        while self.stations[-1] < limit:
            try:
                wp = unique_previous(self.waypoints[-1], step)
            except ValueError:
                break
            a, b = self.waypoints[-1].transform.location, wp.transform.location
            length = math.hypot(a.x-b.x, a.y-b.y)
            if length < 1e-5:
                break
            self.waypoints.append(wp)
            self.stations.append(self.stations[-1] + length)
        if len(self.stations) < 2:
            raise ValueError("No usable upstream lane for spawning")

    def transform(self, distance):
        if not 0 <= distance < self.stations[-1]:
            raise ValueError("Insufficient unbranched upstream road for requested fleet/gap")
        index = max(0, bisect.bisect_right(self.stations, distance) - 1)
        wp = self.waypoints[index]
        remaining = distance - self.stations[index]
        if remaining > 1e-6:
            wp = unique_previous(wp, remaining)
        transform = wp.transform
        transform.location.z += .5
        return transform

    def project(self, vertex):
        closest = None
        for index in range(len(self.stations)-1):
            a = self.waypoints[index].transform.location
            b = self.waypoints[index+1].transform.location
            dx, dy = b.x-a.x, b.y-a.y
            t = max(0., min(1., ((vertex.x-a.x)*dx + (vertex.y-a.y)*dy)/(dx*dx+dy*dy)))
            squared = (vertex.x-a.x-t*dx)**2 + (vertex.y-a.y-t*dy)**2
            station = self.stations[index] + t*(self.stations[index+1]-self.stations[index])
            if closest is None or squared < closest[0]:
                closest = (squared, station)
        if closest[1] <= 1e-6 or closest[1] >= self.stations[-1]-1e-6:
            raise ValueError("Vehicle bounding box exceeds measured lane path")
        return closest[1]

    def interval(self, box, transform):
        stations = [self.project(vertex) for vertex in box.get_world_vertices(transform)]
        return min(stations), max(stations)

    def following_distance(self, leader_rear, box, gap):
        # Fixed-point corrections account for per-model offsets and lane curvature.
        distance = leader_rear + gap + box.extent.x
        for _ in range(20):
            front, _ = self.interval(box, self.transform(distance))
            error = leader_rear + gap - front
            if abs(error) <= .01:
                return distance
            distance += error
        raise ValueError("Could not resolve requested gap on this curved lane")


def measure_box(world, blueprint, path, start_distance, owned, check_stop):
    """CARLA blueprints have no bounding box: measure one short-lived probe.

    Register immediately so failures/interrupts still pass through caller cleanup.
    The collector excludes PROBE_ROLE. All probes are destroyed before release.
    """
    blueprint.set_attribute("role_name", PROBE_ROLE)
    for attempt in range(12):
        check_stop()
        transform = path.transform(start_distance + attempt * 8.)
        actor = world.try_spawn_actor(blueprint, transform)
        if actor is None:
            continue
        owned.append(actor)
        actor.set_simulate_physics(False)
        check_stop()
        box = actor.bounding_box
        if not actor.destroy():
            raise RuntimeError("Geometry probe destruction failed")
        owned.remove(actor)
        return box
    raise RuntimeError("No free location to measure blueprint geometry")


def actual_counts(rows):
    return dict(Counter(row["category"] for row in rows if row.get("status") == "spawned"))
