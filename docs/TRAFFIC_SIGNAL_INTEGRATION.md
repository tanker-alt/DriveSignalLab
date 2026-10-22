# Traffic-signal integration boundary

## Scope of this contribution

`traffic_signal/` contains selection, safe phase transitions, and offline
training only. It deliberately excludes the earlier MDrive-specific runtime,
CARLA maps, routes, model weights, generated traffic, and result logs.

The earlier experiment used MDrive/Leaderboard only for lifecycle wiring:

- obtaining `world` and ego actors from `CarlaDataProvider`;
- receiving per-ego global routes;
- calling signal preparation before a world tick and observation after it;
- translating a per-vehicle signal message into LMDrive `notice_text`.

Vehicle counts were not produced by MDrive perception. They were computed from
CARLA ground truth using actor location, velocity, road/lane IDs, a stop line,
and a 40 m approach/exit region.

## Target component split

The shared CARLA orchestrator should own the world tick and connect five
separate components:

1. `CollectorAdapter`: same-frame collector packet to controller observation.
2. `SignalController`: fixed, Max Pressure, or PressLight target movement.
3. `SafetyPhaseManager`: yellow, all-red, minimum green, maximum green.
4. `SignalExecutor`: validated phase state to CARLA traffic-light actors.
5. `VehicleMessageAdapter`: signal intent to a model-specific input format.

The current controller observation requires `frame`, `sim_time`,
`junction_id`, stable movement IDs, current phase, and per-movement incoming
and outgoing counts. Queue length, waiting time, downstream occupancy, and ETA
can be added in a versioned schema without changing the CARLA actor reader.

## Proposed messages

Messages should be JSON-serializable and must not carry CARLA `world`, actor,
or snapshot objects across the boundary.

### SignalObservation

```json
{
  "schema_version": "signal_observation.v1",
  "session_id": "run-001",
  "observation_id": "obs-1234",
  "frame": 1234,
  "sim_time": 61.7,
  "junction_id": 1427,
  "current_phase": 0,
  "movements": [
    {"movement_id": 0, "incoming_count": 5, "outgoing_count": 1,
     "queue_count": 3},
    {"movement_id": 1, "incoming_count": 2, "outgoing_count": 0,
     "queue_count": 2}
  ]
}
```

### SignalActionRequest and result

The learned policy requests a target green movement. It never writes raw
conflicting lamp colors.

```json
{"type": "SignalActionRequest", "request_id": "req-1234",
 "observation_id": "obs-1234", "based_on_frame": 1234,
 "target_frame": 1235, "junction_id": 1427, "target_phase": 1}
```

The executor returns whether the request was applied, rejected, expired, or
failed, plus the actual transition stage and effective frame.

### SignalIntent for a vehicle

```json
{
  "type": "SignalIntent",
  "frame": 1235,
  "sim_time": 61.75,
  "vehicle_id": 87,
  "junction_id": 1427,
  "traffic_light_id": "signal-18",
  "movement": "left",
  "actual_color": "yellow",
  "current_phase": 0,
  "target_phase": 1,
  "stage": "yellow",
  "stage_remaining_s": 2.8,
  "time_to_green_s": 4.8,
  "time_is_exact": true,
  "valid_until_frame": 1236
}
```

Yellow and all-red remaining time can be exact. During an extendable green,
time to the next policy decision is not guaranteed phase remaining time and
must be labeled separately.

## Safety and timing rules

- One orchestrator owns CARLA `tick()`.
- One executor owns each traffic-light group.
- An observation from frame `t` may request an action for `t+1`; stale actions
  are rejected rather than silently applied later.
- Actual signal state is read back after execution and kept separate from the
  requested phase.
- Vehicle messages are route- and movement-specific and include validity
  bounds to prevent future-information leakage.

The existing shared collector can provide vehicle and signal ground truth, but
its real-time API still needs a stable movement-level aggregation and the
action/message adapters above.
