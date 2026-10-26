# Official PressLight → CARLA Cross3

Uses unmodified `wingsweihua/presslight` commit `efb086782ec4aa4516f18917e39f60c25c39c64d`.
`worker.py` imports the original `SimpleDQNAgent`, directly calls `choose_action`,
`prepare_Xs_Y`, `train_network`, and HDF5 saving/loading. It runs in the existing
Python 3.6 / TensorFlow 1.12 / Keras 2.2.4 environment. CARLA Python 3.8 communicates
with that process using JSON lines. This is not the previous PyTorch approximation.

## Mapping and differences from original CityFlow experiment

- Single four-arm junction with 12 incoming and 12 unique outgoing lanes.
- Original mode-3 state: binary phase (8), three position bins per incoming lane (36),
  three bins per outgoing lane (36). Histogram bin edges are 0/50/100/150 m, adapted
  from the original 0/100/200/300 m for Cross3's shorter roads.
- Four actions: NS straight, NS left, EW straight, EW left. Rights remain green.
- Original reward form: `-.25 * abs(sum(incoming stopped)-sum(outgoing stopped))`.
  Stops use speed <0.1 m/s; average reward over the decision interval. Sampling uses
  CARLA 0.05 s steps rather than CityFlow 1 s steps.
- Nominal decisions every 10 s; switching includes 3 s yellow and at least 2 s
  clearance. A shared safety interlock holds red until controlled straight/left
  vehicles (including rear clearance) leave the junction, then supplies 5 s green.
  Staying in the same phase leaves it green. This can extend a decision interval;
  rewards are averaged over the actual interval and official per-decision gamma remains unchanged.
  No separate maximum-green override.
- Original 20/20 dense layers, RMSprop, learning rate .001, gamma .8, MSE,
  normalization factor 20, replay capacity 10000, sample size 1000, batch size20,
  up to 100 epochs with original early stopping. Target network loads round max(current_round−5,0), matching upstream from round1 onward.
  Epsilon .8 × .95^round, floor .2. Evaluation epsilon0.
- Official updater has no terminal-mask field. The adapter retains its bootstrapped
  final-state convention; this is documented rather than silently altering the algorithm.
- All traffic is CARLA BasicAgent with lane-specific native-light matching and an explicit same-lane longitudinal-gap guard (the stock angle interval can miss collinear queue leaders). This stage
  trains signal control; no LMDrive or teacher labels are used.

## Fair comparison

Identical pre-generated arrivals, routes, vehicles, signal transition durations and
measurement definitions for fixed, MaxPressure and PressLight. External spawn queues
are counted in waiting time. Correct exit lane is checked; collisions and stable
signal mismatch fail the run. Every policy has the same four executable signal phases.
Fixed nominal phase slot selected among10/20/30 s on validation only; all controllers share the same clearance interlock. MaxPressure maximizes
sum of movement incoming stopped minus downstream stopped counts, keeping ties.
Training seeds10000+, validation4101–4104, held-out test7101–7106. Demand types include
balanced, EW-heavy, NS-heavy and left-heavy. Horizon120 s arrivals, up to300 s drain (420 s total), identical for every controller.
Reported aggregate is the unweighted mean of scenario mean waits, with completion
counts alongside it; checkpoint selection first minimizes unfinished fraction, then waiting time. An incomplete policy is still evaluated and reported, but cannot qualify as a successful result.

Acceptance target: at least10% less mean waiting than validation-tuned fixed time,
strictly less than MaxPressure, all planned vehicles completed and zero collisions.
This is an empirical target, not a guarantee. One training seed/six held-out demands
is a first benchmark, not proof of general convergence or statistical significance.

## Run

CARLA must already be running and idle on31610. Map load refuses active worlds.

```bash
/mnt/disk2/users/zlj/lmdrive_runtime/env/bin/python -u \
  /home/zlj/zlj/DriveSignalLab/integrations/cross3/official/train.py \
  --rounds 60 --output /home/zlj/zlj/Drive/outputs/cross3/official_training_NEW
```

Outputs: protocol.json, official_config.json, progress.json, per-scenario demand,
summary, decisions and replay samples; models/round_NNN.h5; final comparison.json.
Use a new output directory. Smoke run adds `--smoke`, evaluates the three controllers
and executes an actual official training update. A random untrained model need not
complete traffic: smoke verifies the pipeline, not learned performance.


## v3 corrections and checks

- Phase input uses the official bit order WL/WS/EL/ES/NL/NS/SL/SS, reordered to
  CARLA actions NS straight / NS left / EW straight / EW left. Both processes
  import `schema.py`. Earlier bits were internally consistent but used a different
  permutation; old checkpoints are deliberately not reused.
- Histogram upper edge 150 m is excluded, matching upstream `np.digitize`.
- The clearance interlock records every stage change in `transitions.json` and
  checks that no controlled vehicle or tail remains when a new green is released.
  Decisions cannot replace a pending transition. The conservative boundary uses
  the manifest stop-line coordinate and vehicle length.
- `test_adapter.py` numerically compares bins, phase semantics and queue reward
  against methods extracted from the unchanged official source; it also checks
  downstream MaxPressure pairing and outgoing-tail clearance.
- `check_clearance.py --output PATH` replays the prior collision's action sequence
  and exercises all 12 directed phase changes with continuous CARLA traffic.
- Training writes source snapshots and hashes; resume rejects changed code or
  protocol. The final held-out comparison automatically runs `audit_results.py`.
- Fixed10/20/30 are nominal action slots, not pure green durations. Switching adds
  shared yellow/all-red clearance; a newly selected phase receives at least 5 s
  green. Fixed timing and MaxPressure use the same interlock as PressLight.
- v2 results predate these changes and are not pooled with v3.

## Native-light reset synchronization (v4)

The v3 run stopped at the start of `train_005`, before any vehicles spawned:
CARLA had not settled native lights after the asynchronous/synchronous reset.
An episode now starts demand/time accounting only after three consecutive matching
initial-phase snapshots with all lights frozen, capped at 20 initialization ticks.
Signal commands are explicitly flushed. In-episode stable mismatches still fail;
`signal_diagnostics.json` stores expected/actual states and initialization frames.
`check_signals.py --output PATH` stresses 40 successive empty-world resets, including
all four first actions and their yellow/red/green transitions. These are synchronization
checks, not traffic-performance benchmark cases. v4 uses a new output directory.
