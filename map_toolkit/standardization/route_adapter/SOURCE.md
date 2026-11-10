# Source provenance

Prepared on 2026-11-03 from the active `D:/CoDriveSignal` project.

- `continuous_route_planner.py` and `network_itinerary.py`: `InformationCollector/`.
- `heterogeneous_route.py` and `roundabout_route.py`: `Toolkits/JunctionToolkit/`.
- Maps: all 12 direct map folders in `maps/MapDataset/`, excluding staging and backups.

The four planning modules retain their algorithms. Imports were adjusted for the standalone adapter and Python package use; specialized planners reuse `follow` and `waypoint_record` from the bundled continuous planner, removing the dependency on the full visualization/drive-check toolkit.

`adapter.py` adds explicit map-type dispatch, batch preprocessing, fingerprint/continuity/coverage validation, live-world matching, and native map loading. Each generated route records the normalized source fingerprint of these five Python files. Original map and manifest bytes were compared with the source workspace (24/24 identical). Generated routes are new offline outputs, not copied historical physical-driving acceptance reports.

The source paths above are provenance only; no runtime code reads the original workspace.

## 2026-11-10 topology extension

`topology_adapter.py` and `route_topology.py` consolidate the project's map/OD and
lane-audit implementation into this existing package. The scenario-side entries
are import bridges; no scenario modules or private acceptance rules are needed.
`contract.py`, `errors.py`, `compatibility.py`, the output schema and generic
regression tests are part of this extension. The four legacy planning modules
are unchanged. This update follows the repository's root Apache-2.0 license;
CARLA is an external dependency under its own license and is not vendored.
The current maps branch has ten base map packages; the earlier twelve-map count
above describes the original preparation, not today's release inventory.
