"""Shared arm layout and movement policy, usable by both Python environments.

Compass: east=OpenDRIVE +X, north=OpenDRIVE +Y (CARLA -Y).
"""
ARM_NAMES = ('west', 'south', 'east', 'north')
TURN_OFFSETS = {'left': -1, 'straight': 2, 'right': 1}


def is_t(config):
    return config.get('type', 'cross') in ('t', 't_junction')


def active_arms(config):
    missing = config.get('missing_arm', 'north') if is_t(config) else None
    if missing is not None and missing not in ARM_NAMES:
        raise ValueError('missing_arm must be north/south/east/west')
    return [i for i, name in enumerate(ARM_NAMES) if name != missing]


def arm_turns(config):
    active = active_arms(config)
    n = config['incoming_lanes']
    explicit = config.get('arm_turns')
    if explicit is not None and (not isinstance(explicit, dict) or set(explicit) != {ARM_NAMES[i] for i in active}):
        raise ValueError('arm_turns must specify exactly the active arms')
    result = {}
    for i in active:
        available = [t for t in ('left','straight','right') if (i+TURN_OFFSETS[t]) % 4 in active]
        if explicit is not None:
            lanes = explicit[ARM_NAMES[i]]
        elif is_t(config):
            # Every incoming lane gets an exit; do not leave a dead lane when
            # removing an arm from the left/straight/right cross template.
            lanes = [available[:]] if n == 1 else [[available[k*len(available)//n]] for k in range(n)]
        else:
            lanes = config.get('allowed_turns', [['left'],['straight'],['right']] if n == 3 else [['left','straight','right'] for _ in range(n)])
        if not isinstance(lanes,list) or len(lanes) != n:
            raise ValueError('Each active arm needs one turn list per incoming lane')
        for turns in lanes:
            if not isinstance(turns,list) or not turns or any(not isinstance(t,str) for t in turns) or len(set(turns)) != len(turns) or set(turns)-set(available):
                raise ValueError('Invalid or missing-arm movement for '+ARM_NAMES[i]+': '+str(turns))
        if is_t(config) and set(t for turns in lanes for t in turns) != set(available):
            raise ValueError('T junction must connect each approach to both remaining arms: '+ARM_NAMES[i])
        result[i] = lanes
    return result
