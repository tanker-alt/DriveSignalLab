"""Seeded arm removal with uniform proposals and connected-grid rejection."""
import random
from layout import ARM_NAMES


def connected(rows, columns, missing):
    pending = [(0, 0)]
    seen = set()
    while pending:
        r, c = pending.pop()
        if (r, c) in seen:
            continue
        seen.add((r, c))
        for dr, dc, arm, opposite in ((0,-1,'west','east'),
                                      (0,1,'east','west'),
                                      (-1,0,'south','north'),
                                      (1,0,'north','south')):
            nr, nc = r+dr, c+dc
            if (0 <= nr < rows and 0 <= nc < columns
                    and missing.get('%d,%d' % (r,c)) != arm
                    and missing.get('%d,%d' % (nr,nc)) != opposite):
                if (nr,nc) not in seen:
                    pending.append((nr,nc))
    return len(seen) == rows*columns


def plan(config):
    rows, columns = int(config.get('rows',3)), int(config.get('columns',3))
    if not 1 <= rows <= 6 or not 1 <= columns <= 6:
        raise ValueError('Grid dimensions must be in [1,6]')
    probability = config.get('t_probability', .5)
    if isinstance(probability,bool) or not isinstance(probability,(int,float)) or not 0 <= probability <= 1:
        raise ValueError('t_probability must be between 0 and 1')
    rng = random.Random(int(config.get('seed',42)))
    keys = ['%d,%d' % (r,c) for r in range(rows) for c in range(columns)]
    types = dict(config.get('junction_types',{}))
    fixed = dict(config.get('missing_arms',{}))
    if (set(types)|set(fixed))-set(keys):
        raise ValueError('Unknown junction coordinate')
    for key in keys:
        if key not in types:
            types[key] = 't' if key in fixed or rng.random() < probability else 'cross'
        if types[key] not in ('t','cross'):
            raise ValueError('Unknown junction type: '+str(types[key]))
    for key,arm in fixed.items():
        if types[key] != 't' or arm not in ARM_NAMES:
            raise ValueError('missing_arms requires a T junction and a compass direction')
    free = [key for key in keys if types[key]=='t' and key not in fixed]
    for attempt in range(100000 if free else 1):
        missing = dict(fixed)
        missing.update((key,rng.choice(ARM_NAMES)) for key in free)
        if connected(rows,columns,missing):
            return dict(config,rows=rows,columns=columns,junction_types=types,
                        missing_arms=missing,t_probability=probability), attempt+1
    raise ValueError('No connected removal plan found; revise fixed types/directions or seed')
