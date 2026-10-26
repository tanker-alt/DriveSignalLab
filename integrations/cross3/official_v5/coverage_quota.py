"""Deterministic coverage quotas; no simulator or learner dependencies."""
from learning import sparse_demand
from schema import PHASE_BITS

# Engineering screening quotas, not statistical convergence guarantees.
DEFAULT_QUOTAS=dict(observed=12,served=4,alternative=4)

def combination(index):
    if not 0<=index<32:raise ValueError(index)
    d=sparse_demand(index)[0]
    return dict(id=index,road=d['road'],lane=d['lane'],initial_phase=(index//8)%4,demand_action=index%4)

def counts(rows, spec, lane_keys):
    lane=lane_keys.index((spec['road'],spec['lane']))
    # Count only one controlled incoming vehicle, no other incoming/outgoing
    # traffic: empty approach time and outgoing-only travel do not count.
    valid=[r for r in rows if sum(r['state']['coming_vehicle'])==1
           and sum(r['state']['coming_vehicle'][lane*3:lane*3+3])==1
           and sum(r['state']['leaving_vehicle'])==0]
    actions=[sum(r['action']==a for r in valid) for a in range(4)]
    return dict(observed=len(valid),served=actions[spec['demand_action']],
                alternative=len(valid)-actions[spec['demand_action']],actions=actions)

def deficit(actual, quotas=None):
    quotas=DEFAULT_QUOTAS if quotas is None else quotas
    return {key:max(0,quotas[key]-actual[key]) for key in quotas}

def gate(entries):
    ids=[e['id'] for e in entries]
    return len(ids)==32 and set(ids)==set(range(32)) and all(not any(e['deficit'].values()) for e in entries)

def schedule(entries):
    # Round-robin by id; no phase can consume the whole budget while others
    # remain uncovered. A collection runner must recheck counts after episodes.
    return [e['id'] for e in sorted(entries,key=lambda e:e['id']) if any(e['deficit'].values())]


def require_coverage(path):
    import json
    if path is None:raise RuntimeError('Coverage plan required before formal training; no collection or training started')
    plan=json.loads(path.read_text())
    if plan.get('quotas') != DEFAULT_QUOTAS:raise RuntimeError('Coverage quota protocol differs')
    entries=plan.get('entries',[])
    # Recompute deficits instead of trusting serialized passed flags.
    checked=[dict(id=e['id'],deficit=deficit(e['counts'])) for e in entries]
    if not gate(checked):raise RuntimeError('Coverage gate failed: fill all 32 combinations before formal training')
    return plan
