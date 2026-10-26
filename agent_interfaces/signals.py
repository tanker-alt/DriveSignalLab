"""Signal inference contract; physical transitions remain owned by the environment."""
from abc import ABC, abstractmethod
from numbers import Integral

PHASE_ORDER = ('NS_STRAIGHT', 'NS_LEFT', 'EW_STRAIGHT', 'EW_LEFT')


class SignalAgent(ABC):
    """reset(context), act(observation) -> phase index, close()."""
    name = 'custom'

    def reset(self, context):
        """Called for each episode with schema and ordered phase names."""

    @abstractmethod
    def act(self, observation):
        """Return a target green index, never physical lamp states."""
        raise NotImplementedError

    def close(self):
        """Release model resources; called once after the run."""


class MaxPressureAgent(SignalAgent):
    name = 'maxpressure_plugin'

    def act(self, observation):
        pressures = observation['pressure']
        current = observation['current_action']
        best = max(pressures)
        return current if current is not None and pressures[current] == best else pressures.index(best)


class SelectorAgent(SignalAgent):
    """Adapt an existing select(observation, current_action) implementation."""
    def __init__(self, selector):
        self.selector = selector
        self.name = selector.name

    def act(self, observation):
        return self.selector.select(observation['raw'], observation['current_action'])


class SignalSelector:
    """Bridge the common contract to SafeAdaptivePolicy's selector API."""
    def __init__(self, agent, phase_order=PHASE_ORDER, observation_schema='cross3_counts_v1'):
        self.agent = agent
        self.name = getattr(agent, 'name', type(agent).__name__)
        self.movements = len(phase_order)
        self.phase_order = tuple(phase_order)
        self.observation_schema = observation_schema
        self.metadata = getattr(agent, 'metadata', {})
        for method in ('reset', 'act', 'close'):
            if not callable(getattr(agent, method, None)):
                raise TypeError('Signal agent requires %s()' % method)
        agent.reset(dict(schema='signal_context_v1', phase_order=list(phase_order),
                         observation_schema=observation_schema))

    def select(self, observation, current_action=None):
        if len(observation['pressure']) != self.movements:
            raise ValueError('Pressure vector must follow the configured phase order')
        packet = dict(schema='signal_observation_v1',
                      observation_schema=self.observation_schema,
                      phase_order=list(self.phase_order), current_action=current_action,
                      pressure=list(observation['pressure']),
                      elapsed_seconds=observation.get('elapsed_seconds', observation.get('elapsed')), raw=observation)
        action = self.agent.act(packet)
        if isinstance(action, bool) or not isinstance(action, Integral) or not 0 <= action < self.movements:
            raise ValueError('Signal action must be an integer in [0, %d)' % self.movements)
        return int(action)


class OfficialSignalCallback(SignalSelector):
    """Bridge Environment.run's callback; preserve official 80-D state losslessly."""
    def __init__(self, agent):
        super().__init__(agent, observation_schema='cross3_official_80_v1')

    def __call__(self, state, pressures, current, elapsed):
        return self.select(dict(state=state, pressure=pressures,
                                elapsed_seconds=elapsed), current)
