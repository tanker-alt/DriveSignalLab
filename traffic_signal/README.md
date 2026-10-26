# Two-movement traffic-signal prototype

This directory contains the small, vehicle-agent-independent part of the
current Town02 T-junction prototype. It provides:

- a fixed-time baseline;
- a rule-based Max Pressure selector;
- inference for a locally trained, two-action PressLight-style DQN;
- a safety state machine that inserts yellow and all-red clearance;
- an offline queue simulator used to train a dimension-matched checkpoint.

The prototype is **not** the upstream PressLight implementation. It follows
the pressure-reward and DQN phase-selection idea but uses a six-dimensional
state and two movements. Model weights, maps, CARLA binaries, datasets, logs,
and experiment output are intentionally excluded from Git.

## Observation contract

The adaptive controllers consume an ordinary Python dictionary:

```python
observation = {
    "incoming": [5, 2],
    "outgoing": [1, 0],
    "queued": [3, 2],       # collected for evaluation; not a DQN feature yet
    "pressure": [4, 2],
}
```

Movement ordering must be stable for the whole run. Counts and phases must
refer to one CARLA frame and simulation timestamp. The controller does not
collect CARLA actors itself; an integration adapter must convert the shared
collector output into this contract.

The PressLight-style state vector is:

```text
[phase_0, phase_1, incoming_0, outgoing_0, incoming_1, outgoing_1]
```

## Train a local checkpoint

```bash
python -m traffic_signal.train_two_movement \
  --output traffic_signal/checkpoints/two_movement.npz \
  --steps 150000 \
  --decision-seconds 2 \
  --yellow-seconds 3 \
  --clearance-seconds 2 \
  --min-green-seconds 5 \
  --max-green-seconds 15
```

This trains in a synthetic two-queue environment, not online in CARLA. A JSON
metadata file is written next to the NPZ file. Both files are required for
inference. The `checkpoints/` directory is ignored; publish weights through a
separate artifact channel after review.

## Tests

```bash
python -m unittest discover -s traffic_signal/tests -v
```

See [the integration note](../docs/TRAFFIC_SIGNAL_INTEGRATION.md) for the
remaining CARLA collector, signal executor, and vehicle-message work.
