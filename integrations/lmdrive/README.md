# CARLA + two-ego LMDrive integration

This directory contains the lightweight files needed to reproduce the Town02
signal experiment. CARLA, MDrive, LMDrive/LLaVA weights, generated checkpoints,
logs, images, and videos are intentionally external.

## Compatibility

- CARLA 0.9.12, matching the pinned MDrive/LMDrive environment used here
- Python 3.8 environment with `requirements-lmdrive.txt`
- a clean MDrive checkout compatible with `patches/mdrive_multiego_signal.patch`
- two LMDrive route actors in Town02

The patch adds lifecycle hooks required to run the signal bridge around each
synchronous CARLA tick, retire completed egos safely, and continue simulation
until reproducible background demand drains.

## Required external paths

```bash
export MDRIVE_ROOT=/path/to/MDrive
export CARLA_ROOT=/path/to/CARLA_0.9.12
export LMDRIVE_RUNTIME=/path/to/lmdrive_runtime
export LMDRIVE_LLM_DIR=/path/to/llava-v1.5-7b
export LMDRIVE_VISION_CHECKPOINT=/path/to/vision-encoder-r50.pth.tar
export LMDRIVE_CHECKPOINT=/path/to/llava-v1.5-checkpoint.pth
```

Install the integration into a clean MDrive checkout once:

```bash
bash integrations/lmdrive/scripts/install_overlay.sh
```

Run two LMDrive egos with deterministic low background demand:

```bash
bash integrations/lmdrive/scripts/run_carla_experiment.sh fixed low
bash integrations/lmdrive/scripts/run_carla_experiment.sh maxpressure low
```

Train a checkpoint outside Git and run the learned controller:

```bash
python -m traffic_signal.train_two_movement \
  --output integrations/lmdrive/checkpoints/two_movement.npz
export PRESSLIGHT_CHECKPOINT="$PWD/integrations/lmdrive/checkpoints/two_movement.npz"
bash integrations/lmdrive/scripts/run_carla_experiment.sh presslight low
```

Generated `.npz` and `.json` files remain ignored. Metadata must match
`checkpoints/metadata.example.json`; runtime rejects missing or mismatched
weights instead of silently using a random network.

## Runtime data flow

1. `fixed_signal_lmdrive.py` starts one signal runtime for both ego routes.
2. `fixed_traffic_light.py` maps routes to CARLA stop lines and counts incoming,
   outgoing, and queued actors.
3. `background_traffic.py` creates route-bound Traffic Manager vehicles from a
   deterministic demand manifest.
4. The policy selects a target movement. The safety layer inserts minimum green,
   yellow, and all-red stages before applying CARLA light states.
5. Optional camera-frame-aligned notices enter LMDrive through its notice input.
6. JSONL records and metrics are written under the run output directory.

## Evaluation

The scripts under `evaluation/` audit background demand and lane counters,
summarize observed training-state ranges, and compare matched fixed-time, Max
Pressure, and learned-controller runs after background vehicles have exited.

To inspect a live or completed run, start the read-only viewer and open the
printed local URL (use SSH port forwarding when running on a remote server):

```bash
python integrations/lmdrive/visualization/serve_mdrive_preview.py \
  --root "${EXPERIMENT_OUTPUT_ROOT:-outputs}" --port 8766
```

This is the exact two-movement CARLA prototype used by the current experiment.
It is not the upstream PressLight four-arm benchmark implementation.
