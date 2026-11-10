# Experimental maps and beautification

This branch contains 10 accepted experimental maps in `maps/MapDataset/`, their configurations, precomputed routes, preview images and acceptance evidence. The standalone CARLA beautification runtime is in `maps/beautification/`.

```bash
python maps/beautification/view.py maps/MapDataset/cross3 --no-photos
```

Generator, development tools and route adapter are maintained on [toolkit/map-toolkit-2026-11-02](https://github.com/tanker-alt/DriveSignalLab/tree/toolkit/map-toolkit-2026-11-02/map_toolkit). They are no longer bundled in this branch. See the beautification README for runtime requirements.

The 10 maps have 536 signals and 1,608 passing vehicle-camera checks. Overview and street images were captured after acceptance. Camera: Lincoln MKZ 2020, 1920x1080, horizontal FOV 100 degrees, local x=-1.5 m, z=2 m, pitch 0. Tests cover stop position and available upstream distances (15/30 m, clamped on short roads). Acceptance checks full lamp visibility and rendered red/yellow/green differences, not ML recognition accuracy or adverse-weather/traffic occlusion. The old 900x256 camera still has limited distant color resolution.

This branch split does not change any map, preview, route or camera evidence. Historical generator validation belongs to the toolkit branch. Per-map acceptance JSON and `reports/acceptance_summary.json` document the map release.
