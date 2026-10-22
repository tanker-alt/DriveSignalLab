#!/usr/bin/env python3
"""Train a small two-movement PressLight DQN for the CARLA adapter.

This is a local, dimension-matched checkpoint, not a pretrained artifact from
the PressLight authors.  The synthetic queue environment uses exact counts and
negative absolute movement pressure as its reward.  A phase change reproduces
the CARLA safety layer: yellow and all-red admit no vehicles, followed by the
minimum green before another decision is eligible.
"""
import argparse
import json
import random
from collections import deque
from pathlib import Path

import numpy as np
import torch
from torch import nn


class QNetwork(nn.Module):
    def __init__(self):
        super().__init__()
        self.dense_0 = nn.Linear(6, 20)
        self.dense_1 = nn.Linear(20, 20)
        self.q_values = nn.Linear(20, 2)

    def forward(self, value):
        value = torch.relu(self.dense_0(value))
        value = torch.relu(self.dense_1(value))
        return self.q_values(value)


def vector(action, queues, outgoing):
    phase = [0.0, 0.0]
    phase[action] = 1.0
    return np.asarray(phase + [queues[0], outgoing[0], queues[1], outgoing[1]], dtype='float32')


def _arrivals(queues, rates, queue_cap):
    for i in range(2):
        queues[i] = min(queue_cap, queues[i] + int(random.random() < rates[i]))


def _pressure_reward(queues, outgoing):
    return -float(sum(abs(q-o) for q, o in zip(queues, outgoing)))


def advance(queues, rates, queue_cap, outgoing_cap, current, chosen,
            decision_seconds, yellow_seconds, clearance_seconds,
            min_green_seconds, gamma):
    """Advance one policy decision and return its discounted accumulated cost."""
    switching = chosen != current
    blocked = yellow_seconds + clearance_seconds if switching else 0
    green = min_green_seconds if switching else decision_seconds
    duration = blocked + green
    reward = 0.0
    outgoing = [0, 0]
    for second in range(duration):
        outgoing = [0, 0]
        if second >= blocked:
            departed = min(queues[chosen], random.randint(1, max(1, outgoing_cap)))
            queues[chosen] -= departed
            outgoing[chosen] = departed
        _arrivals(queues, rates, queue_cap)
        reward += (gamma ** second) * _pressure_reward(queues, outgoing)
    return reward, outgoing, duration


def train(seed, steps, queue_cap, outgoing_cap, decision_seconds,
          yellow_seconds, clearance_seconds, min_green_seconds,
          max_green_seconds, gamma):
    random.seed(seed); np.random.seed(seed); torch.manual_seed(seed)
    model, target = QNetwork(), QNetwork()
    target.load_state_dict(model.state_dict())
    optimizer = torch.optim.RMSprop(model.parameters(), lr=3e-4)
    replay = deque(maxlen=20000)
    queues, outgoing, action = [0, 0], [0, 0], 0
    green_age = min_green_seconds
    forced_switches = 0
    rates = [0.25, 0.25]
    for step in range(steps):
        if step % 250 == 0:
            # Cover the low/medium/high demand regimes observed in CARLA,
            # including imbalanced approaches.
            regime = random.choice(((.08, .18), (.18, .35), (.35, .60)))
            rates = [random.uniform(*regime), random.uniform(*regime)]
        state = vector(action, queues, outgoing)
        epsilon = max(.03, 1.0-step/(steps*.7))
        with torch.no_grad():
            chosen = random.randrange(2) if random.random() < epsilon else int(model(torch.from_numpy(state)).argmax())
        if green_age >= max_green_seconds and queues[1 - action] > 0:
            chosen = 1 - action
            forced_switches += 1
        reward, outgoing, duration = advance(
            queues, rates, queue_cap, outgoing_cap, action, chosen,
            decision_seconds, yellow_seconds, clearance_seconds,
            min_green_seconds, gamma)
        next_state = vector(chosen, queues, outgoing)
        replay.append((state, chosen, reward, next_state, duration))
        green_age = min_green_seconds if chosen != action else green_age + decision_seconds
        action = chosen
        if len(replay) >= 256:
            batch = random.sample(replay, 128)
            states = torch.from_numpy(np.stack([x[0] for x in batch]))
            actions = torch.tensor([x[1] for x in batch], dtype=torch.long)
            rewards = torch.tensor([x[2] for x in batch], dtype=torch.float32)
            next_states = torch.from_numpy(np.stack([x[3] for x in batch]))
            durations = torch.tensor([x[4] for x in batch], dtype=torch.float32)
            predicted = model(states).gather(1, actions[:, None]).squeeze(1)
            with torch.no_grad():
                expected = rewards + torch.pow(torch.tensor(gamma), durations) * target(next_states).max(1).values
            loss = nn.functional.mse_loss(predicted, expected)
            optimizer.zero_grad(); loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), 5.0)
            optimizer.step()
        if step % 1000 == 0:
            target.load_state_dict(model.state_dict())
    return model, forced_switches


def validate(model, incoming_max, outgoing_max):
    correct = total = 0
    examples = []
    with torch.no_grad():
        for current in range(2):
            for a in range(incoming_max + 1):
                for b in range(incoming_max + 1):
                    for out_a in range(outgoing_max + 1):
                        for out_b in range(outgoing_max + 1):
                            pressure = [a-out_a, b-out_b]
                            if pressure[0] == pressure[1]:
                                continue
                            q = model(torch.from_numpy(
                                vector(current, [a, b], [out_a, out_b])))
                            selected = int(q.argmax())
                            correct += selected == (0 if pressure[0] > pressure[1] else 1)
                            total += 1
            for queues in ([1, 1], [0, 1], [1, 0], [0, 0],
                           [0, incoming_max], [incoming_max, 0]):
                q = model(torch.from_numpy(vector(current, queues, [0, 0])))
                examples.append({'current_phase':current,'incoming':list(queues),
                                 'q_values':q.tolist(),'action':int(q.argmax())})
    return correct/total, examples


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seed', type=int, default=2026)
    parser.add_argument('--steps', type=int, default=100000)
    parser.add_argument('--state-distribution', type=Path)
    parser.add_argument('--decision-seconds', type=int, default=2)
    parser.add_argument('--yellow-seconds', type=int, default=3)
    parser.add_argument('--clearance-seconds', type=int, default=2)
    parser.add_argument('--min-green-seconds', type=int, default=5)
    parser.add_argument('--max-green-seconds', type=int, default=15)
    parser.add_argument('--gamma', type=float, default=.8)
    args = parser.parse_args()
    distribution = None
    incoming_max, outgoing_max, queue_cap = 6, 1, 8
    if args.state_distribution:
        distribution = json.loads(args.state_distribution.read_text())
        if distribution.get('schema') != 'carla_background_state_distribution_v1':
            raise ValueError('Unsupported state-distribution schema')
        incoming_max = max(int(x['max']) for x in distribution['features']['incoming'])
        outgoing_max = max(int(x['max']) for x in distribution['features']['outgoing'])
        queue_cap = max(int(distribution['totals']['queued']['max']), incoming_max)
    timings = (args.decision_seconds, args.yellow_seconds,
               args.clearance_seconds, args.min_green_seconds,
               args.max_green_seconds)
    if any(value <= 0 for value in timings):
        raise ValueError('All signal timing values must be positive integer seconds')
    if not 0 < args.gamma < 1:
        raise ValueError('gamma must be between zero and one')
    if args.max_green_seconds < args.min_green_seconds:
        raise ValueError('max-green-seconds must be at least min-green-seconds')
    model, forced_switches = train(
        args.seed, args.steps, queue_cap, outgoing_max,
        args.decision_seconds, args.yellow_seconds,
        args.clearance_seconds, args.min_green_seconds,
        args.max_green_seconds, args.gamma)
    accuracy, examples = validate(model, incoming_max, outgoing_max)
    print(json.dumps({'validation_accuracy':accuracy,'examples':examples}, indent=2), flush=True)
    # With a real switch cost, a one-vehicle imbalance may rationally keep the
    # current green.  Require switching only for the strongest observed
    # imbalance and require equal demand to avoid needless switching.
    strong = [row for row in examples
              if row['incoming'] in ([0, incoming_max], [incoming_max, 0])]
    if any(row['action'] != (1 if row['incoming'][1] > row['incoming'][0] else 0)
           for row in strong):
        raise RuntimeError('Checkpoint failed the strong-demand phase-choice contract')
    ties = [row for row in examples if row['incoming'] == [1, 1]]
    if any(row['action'] != row['current_phase'] for row in ties):
        raise RuntimeError('Checkpoint failed the equal-demand phase-hold contract')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    state = model.state_dict()
    np.savez(args.output,
             dense_0_kernel=state['dense_0.weight'].numpy().T,
             dense_0_bias=state['dense_0.bias'].numpy(),
             dense_1_kernel=state['dense_1.weight'].numpy().T,
             dense_1_bias=state['dense_1.bias'].numpy(),
             q_values_kernel=state['q_values.weight'].numpy().T,
             q_values_bias=state['q_values.bias'].numpy())
    metadata = {
        'schema':'carla_two_movement_presslight_v1', 'state_dim':6, 'action_dim':2,
        'features':['phase_0','phase_1','incoming_0','outgoing_0','incoming_1','outgoing_1'],
        'architecture':[6,20,20,2], 'algorithm':'DQN', 'optimizer':'RMSprop',
        'reward':'negative_sum_absolute_movement_pressure',
        'training_environment':'synthetic_two_movement_queue_v3_carla_signal_timing',
        'source':'locally trained; architecture follows official PressLight SimpleDQN',
        'seed':args.seed, 'training_steps':args.steps,
        'scope':'single_junction_low_medium_high_background_profiles',
        'signal_timing_seconds': {
            'decision_interval': args.decision_seconds,
            'yellow': args.yellow_seconds,
            'all_red_clearance': args.clearance_seconds,
            'minimum_green_after_switch': args.min_green_seconds,
            'maximum_green': args.max_green_seconds,
        },
        'transition_model': {
            'keep_phase_duration_s': args.decision_seconds,
            'switch_duration_s': (args.yellow_seconds + args.clearance_seconds
                                  + args.min_green_seconds),
            'no_discharge_during_yellow_and_all_red': True,
            'reward_aggregation': 'discounted per-second negative pressure',
            'semi_markov_discount': 'gamma ** transition_duration_s',
            'gamma_per_second': args.gamma,
            'maximum_green_forced_switches_during_training': forced_switches,
        },
        'observed_ranges':{'incoming_per_movement_max':incoming_max,
                           'outgoing_per_movement_max':outgoing_max,
                           'total_queue_max':queue_cap},
        'state_distribution':str(args.state_distribution.resolve()) if args.state_distribution else None,
        # This is diagnostic only: a timing-aware controller should sometimes
        # differ from instantaneous Max Pressure because switching is costly.
        'instantaneous_max_pressure_agreement':accuracy,
        'strong_demand_choice_passed':True,
        'equal_demand_phase_hold_passed':True,
        'validation_examples':examples}
    args.output.with_suffix('.json').write_text(json.dumps(metadata, indent=2))
    print(json.dumps({'checkpoint':str(args.output),'accuracy':accuracy,'examples':examples}, indent=2))


if __name__ == '__main__':
    main()
