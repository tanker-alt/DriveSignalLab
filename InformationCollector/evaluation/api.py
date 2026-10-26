"""Public collection boundary for the vehicle/signal agent developer.

Usage: collector = create_collector(world, RoadSpec(junction_id=1427))
The orchestrator calls collector.sample_observation(snapshot) once after each tick
for the agent-facing packet, then close(). Existing sample(snapshot, phase) returns
the legacy audit packet; use collector.last_observation for agent delivery in that mode.
This module never loads a map, ticks, spawns actors, or applies agent actions.
"""

from dataclasses import dataclass
from typing import Optional

from .collector import FrameCollector, DEFAULT_OUTPUT


@dataclass(frozen=True)
class RoadSpec:
    junction_id: int
    radius_m: float = 120.0
    expected_map: Optional[str] = None
    arrival_distance_m: Optional[float] = None
    exit_distance_m: float = 5.0


def create_collector(world, roads, output_root=DEFAULT_OUTPUT,
                     mode="orchestrator_owned_tick", collisions=False, detection_config=None,stop_reference_config=None,
                     collision_config=None):
    """detection_config optionally supplies user-calibrated counting sections.

    Config must contain map_name, junction_id, and lines. Unknown/ambiguous map
    geometry raises before output creation; it is never replaced by Town05 data.
    """
    if roads.expected_map and world.get_map().name.split("/")[-1] != roads.expected_map.split("/")[-1]:
        raise ValueError("Loaded map does not match RoadSpec.expected_map")
    return FrameCollector(world, roads.junction_id, output_root=output_root,
                          radius=roads.radius_m, mode=mode, collisions=collisions,
                          detection_config=detection_config,
                          stop_reference_config=stop_reference_config,
                          collision_config=collision_config,
                          arrival_distance_m=roads.arrival_distance_m,
                          exit_distance_m=roads.exit_distance_m)
