"""Predeclared demand families; seeds are split before training."""
import json
import random
from .sumo_env import MAP,demand as balanced_demand

PROFILES=('balanced','east_west','north_south','left_heavy')


def demand(seed,horizon,rate=.035,egos=False,profile='balanced'):
    if profile=='balanced': return balanced_demand(seed,horizon,rate,egos)
    if profile not in PROFILES: raise ValueError('Unknown demand profile')
    rng=random.Random(seed); rows=[]
    moves=json.loads((MAP/'manifest.json').read_text())['movements']
    for m in moves:
        weight=1.
        if profile=='east_west': weight=1.5 if m['road'] in (1,3) else .5
        if profile=='north_south': weight=1.5 if m['road'] in (2,4) else .5
        if profile=='left_heavy': weight=2. if m['lane']==-1 else .5
        t=rng.uniform(0,5)
        while t<horizon:
            rows.append(dict(m,time=t,ego=False));t+=rng.expovariate(rate*weight)
    if egos:
        rows.extend(dict(next(m for m in moves if m['road']==road and m['lane']==-2),time=0.,ego=True) for road in (1,3))
    rows.sort(key=lambda x:(x['time'],x['road'],x['lane']))
    for i,row in enumerate(rows): row['id']='v%d'%i
    return rows
