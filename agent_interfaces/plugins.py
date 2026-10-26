"""Load trusted local Python factories without importing model runtimes eagerly."""
import importlib
import json
import math
import os
from pathlib import Path


def load_plugin(path, kind):
    """JSON: schema, kind, factory (module:callable), kwargs."""
    config = json.loads(Path(path).read_text())
    if config.get('schema') != 'drive_agent_plugin_v1' or config.get('kind') != kind:
        raise ValueError('Expected drive_agent_plugin_v1 plugin of kind %s' % kind)
    module, separator, name = config.get('factory', '').partition(':')
    if not separator or not module or not name:
        raise ValueError('factory must be module:callable')
    kwargs = config.get('kwargs', {})
    if not isinstance(kwargs, dict):
        raise ValueError('kwargs must be an object')
    def expand(value):
        if isinstance(value, str):
            return os.path.expanduser(os.path.expandvars(value))
        if isinstance(value, list):
            return [expand(item) for item in value]
        if isinstance(value, dict):
            return {key: expand(item) for key, item in value.items()}
        return value
    return getattr(importlib.import_module(module), name)(**expand(kwargs))


def load_driving_agent(path, ego_count):
    """Factory returns an initialized MDrive multi-ego Leaderboard agent."""
    agent = load_plugin(path, 'driving')
    try:
        for method in ('sensors', 'set_global_plan', 'run_step', 'destroy'):
            if not callable(getattr(agent, method, None)):
                raise TypeError('Driving agent requires %s()' % method)
        if not callable(agent):
            raise TypeError('Driving agent must implement the Leaderboard __call__ contract')
        if getattr(agent, 'ego_vehicles_num', None) != ego_count:
            raise ValueError('Driving agent ego_vehicles_num does not match --egos')
        if not hasattr(agent, 'sensor_interface'):
            raise TypeError('Driving agent requires a Leaderboard sensor_interface')
        if not isinstance(getattr(agent, 'agent_name', None), str):
            raise TypeError('Driving agent requires a string agent_name for MDrive sensor setup')
    except Exception:
        if callable(getattr(agent, 'destroy', None)):
            agent.destroy()
        raise
    return agent


def validate_controls(controls, ego_count):
    """Reject missing, nonfinite or out-of-range actuator commands before application."""
    if not isinstance(controls, (list, tuple)) or len(controls) != ego_count:
        raise ValueError('Driving agent must return one VehicleControl per ego, in route order')
    for control in controls:
        for key, low, high in (('throttle', 0, 1), ('brake', 0, 1), ('steer', -1, 1)):
            value = float(getattr(control, key))
            if not math.isfinite(value) or not low <= value <= high:
                raise ValueError('Invalid driving control %s=%r' % (key, value))
    return controls
