# Map collection and evaluation modules

- Maps/MapDataset/maps: OpenDRIVE maps and their definitions. Default: cross3, junction 10.
- InformationCollector: same-frame state acquisition and agent observation interface.
- EvaluationMetrics: metric consumer, MDrive adapter and offline tests.
- Toolkits: map generation/loading, stop-line review and packaging dependencies.

Install InformationCollector/requirements.txt and EvaluationMetrics/evaluation/requirements-mdrive.txt into the CARLA-compatible environment. Generator dependencies are documented under Toolkits/JunctionToolkit. CARLA server/client version used for validation: 0.9.15.

Run from repository root:
```shell
python -B InformationCollector/run_experiment.py --prepare-only
python -m Toolkits.CollectionTools.map_setup --help
python -m Toolkits.CollectionTools.stop_reference_review --help
```

For tests, place the repository root, InformationCollector and EvaluationMetrics on PYTHONPATH. Start a dedicated idle CARLA server before loading maps. Cross3 uses a configurable 120 m evaluation circle. Background and ego vehicles are both included.

Validation: 168 offline tests passed locally. Previous 30-second TCP integration collected 601 continuous frames with one ego and six background vehicles; TCP did not finish its route. This is not official MDrive leaderboard evaluation. SUMO all-vehicle TTC and fuel remain pending.

Historical candidate maps are retained locally but excluded from this upload, along with caches, outputs, checkpoints, third-party source trees and machine-specific Python environments. Local CARLA/TCP harnesses and validation evidence are not included. Existing repository traffic-signal/agent integrations remain independent.

For external MDrive installations, set MDRIVE_ROOT to the MDrive repository directory before running tests or evaluation. The upstream source is not bundled.
