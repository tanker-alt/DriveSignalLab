# Source provenance

Prepared on 2026-11-03 from the active `D:/CoDriveSignal` project.

- `continuous_route_planner.py` and `network_itinerary.py`: `InformationCollector/`.
- `heterogeneous_route.py` and `roundabout_route.py`: `Toolkits/JunctionToolkit/`.
- Maps: all 12 direct map folders in `maps/MapDataset/`, excluding staging and backups.

The four planning modules retain their algorithms. Imports were adjusted for the standalone adapter and Python package use; specialized planners reuse `follow` and `waypoint_record` from the bundled continuous planner, removing the dependency on the full visualization/drive-check toolkit.

`adapter.py` adds explicit map-type dispatch, batch preprocessing, fingerprint/continuity/coverage validation, live-world matching, and native map loading. Each generated route records the normalized source fingerprint of these five Python files. Original map and manifest bytes were compared with the source workspace (24/24 identical). Generated routes are new offline outputs, not copied historical physical-driving acceptance reports.

The source paths above are provenance only; no runtime code reads the original workspace.
