"""Versioned semi-Markov replay; no CARLA or TensorFlow dependency."""
import math

BASE_SECONDS = 10.0
GAMMA = .8

class IntervalReturn:
    def __init__(self):
        self.duration = 0.0
        self.discounted = 0.0
        self.integral = 0.0

    def add(self, reward, dt):
        if dt <= 0 or not math.isfinite(reward):
            raise ValueError('Invalid tick reward/duration')
        # Exact integral for piecewise constant tick reward. Divide by 10s so
        # units remain comparable to the upstream 10s average-reward target.
        rate = -math.log(GAMMA) / BASE_SECONDS
        weight = math.exp(-rate*self.duration) * (-math.expm1(-rate*dt)) / rate / BASE_SECONDS
        self.discounted += reward * weight
        self.integral += reward * dt
        self.duration += dt

    def sample(self, state, action, next_state, time, terminal, truncated):
        if self.duration <= 0 or (terminal and truncated):
            raise ValueError('Invalid interval')
        return dict(schema=2,state=state,action=action,next_state=next_state,
                    reward=self.discounted,average_reward=self.integral/self.duration,
                    duration=self.duration,discount=GAMMA**(self.duration/BASE_SECONDS),
                    terminal=bool(terminal),truncated=bool(truncated),time=time)


def epsilon(samples, decay_samples=12000):
    if samples < 0 or decay_samples <= 0:
        raise ValueError('Invalid exploration count')
    return .8 - .7 * min(1.,samples / float(decay_samples))


def bellman(sample, next_q, normal_factor=20., variant='smdp'):
    if sample['schema'] != 2:
        raise ValueError('Cannot mix legacy replay')
    if variant == 'legacy':
        return sample['average_reward']/normal_factor + GAMMA*next_q
    if variant != 'smdp':
        raise ValueError(variant)
    return sample['reward']/normal_factor + (0. if sample['terminal'] else sample['discount']*next_q)


def prepare_targets(agent, memory, variant, rng, np):
    indices=rng.sample(range(len(memory)),min(agent.dic_agent_conf['SAMPLE_SIZE'],len(memory)))
    rows=[memory[i] for i in indices]
    features=agent.dic_traffic_env_conf['LIST_STATE_FEATURE']
    xs=[np.array([r['state'][f] for r in rows]) for f in features]
    ns=[np.array([r['next_state'][f] for r in rows]) for f in features]
    q_before=agent.q_network.predict(xs)
    targets=q_before.copy()
    future=agent.q_network_bar.predict(ns).max(axis=1)
    for i,r in enumerate(rows):
        targets[i,r['action']]=bellman(r,float(future[i]),agent.dic_agent_conf['NORMAL_FACTOR'],variant)
    agent.Xs=xs;agent.Y=targets
    return [dict(replay_index=indices[i],action=r['action'],q=q_before[i].tolist(),td_error=float(targets[i,r['action']]-q_before[i,r['action']]),td_target=float(targets[i,r['action']]),reward=r['reward'],average_reward=r['average_reward'],duration=r['duration'],gamma_eff=r['discount'],terminal=r['terminal'],truncated=r['truncated'],next_q_max=float(future[i])) for i,r in enumerate(rows)]


def sparse_demand(index):
    # One late controlled vehicle, cycling all 4 movements and both approaches.
    phase=index % 4
    road=((2,4),(2,4),(1,3),(1,3))[phase][(index//4)%2]
    return [dict(time=60.,road=road,lane=-2 if phase%2==0 else -1)]
