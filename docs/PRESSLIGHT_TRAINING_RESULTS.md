# PressLight training record

## Offline pretraining

The current pretrainer is `integrations/cross3/offline_train.py`. It uses the
official Cross3 state layout (80 values: 8 phase bits, 36 incoming and 36
outgoing position bins), four actions (`NS_STRAIGHT`, `NS_LEFT`,
`EW_STRAIGHT`, `EW_LEFT`), RMSprop, a 20-20 MLP, replay capacity 20,000 and
semi-Markov discount `0.8 ** (duration_seconds / 10)`. Its synthetic queue
simulator is not CARLA: arrivals, service and downstream departures are random.
The reward follows the official PressLight default, `-0.25` times the number
of incoming vehicles stopped for more than one second.

The 100,000-step seed results were:

| Seed | Fixed cost | MaxPressure cost | PressLight cost | PressLight remaining |
|---:|---:|---:|---:|---:|
| 2026 | 10191.96 | 10330.75 | 9386.30 | 358.8 |
| 2027 | 10702.54 | 10833.99 | 9570.71 | 355.5 |
| 2028 | 10134.29 | 10229.19 | 9333.39 | 351.6 |

These are simulator-only comparisons and are not CARLA results.

## CARLA fine-tuning protocol

CARLA uses the Cross3 runner with the real map, 12 lane movements, the same
four actions, yellow/all-red safety stages, and ground-truth vehicle counts.
CARLA training data consists of per-decision state transitions collected from
the live world: 28-value state (four phase one-hot values, 12 incoming counts,
12 outgoing counts), selected action, elapsed transition duration, accumulated
pressure reward, next state, and terminal flag. The runner writes signal and
vehicle JSONL logs plus `summary.json`.

The 80-value offline checkpoint is not loaded directly into the current CARLA
learner because its state layout and network input dimension differ. CARLA
fine-tuning must start from a dimension-matched 28-value CARLA checkpoint, or
the CARLA runner must first be upgraded to the official 80-value collector.

## CARLA results

The offline seed 2028 checkpoint was converted to
`outputs/cross3_offline_official/offline_seed2028_carla80.pt` with
`integrations/cross3/convert_offline_checkpoint.py`. The converter records the
Cross3 OpenDRIVE hash and the 12-lane ordering. The CARLA runner was upgraded
to the same 80-D state and accepted the checkpoint format without a weight
shape error. A 30-second CARLA fine-tuning attempt was then made with one
background vehicle per lane and seed 2026, but the CARLA RPC endpoint at
`127.0.0.1:31600` timed out before map loading. No CARLA training result is
reported from that attempt.

To be filled after a live training run with identical demand and seed:

| Controller | Completed/planned | Collisions | Mean wait | Queue cost | Output |
|---|---:|---:|---:|---:|---|
| Fixed | pending | pending | pending | pending | pending |
| MaxPressure | pending | pending | pending | pending | pending |
| PressLight fine-tuned | pending | pending | pending | pending | pending |
