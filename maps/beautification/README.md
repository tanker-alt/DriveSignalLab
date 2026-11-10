# Map beautification runtime

Requires CARLA server and Python API 0.9.15 (Python 3.7). Uses CARLA built-in assets; no commercial binaries are bundled.

```bash
python maps/beautification/view.py maps/MapDataset/cross3 --no-photos
python maps/beautification/view.py maps/MapDataset/heterogeneous_3x3 --no-photos
```

Loading replaces the CARLA world. Single junctions use complete beautification, networks use simple beautification by default. Lamp supports follow the accepted physical lamp poses.

Source maintenance: `toolkit/map-toolkit-2026-11-02`, `map_toolkit/standardization/JunctionToolkit`.
