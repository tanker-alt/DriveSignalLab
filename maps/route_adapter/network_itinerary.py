"""Find a legal boundary-to-boundary itinerary covering every junction.

Search road entry + visited-junction mask, not just junction adjacency.
Lane selection is resolved separately on the approach before each turn.
"""
from collections import deque


def itinerary(design, required=None, start_road=None, start_lane=None):
    junctions = design['junctions']
    bits = {j['id']: 1 << i for i,j in enumerate(junctions)}
    all_bits = sum(bits[j] for j in required) if required is not None else (1 << len(junctions))-1
    seams = {}
    for seam in design['seams']:
        a,b = seam['road_a'],seam['road_b']
        seams[a]=b; seams[b]=a
    moves = {}
    for move in design['movements']:
        moves.setdefault(move['road'], []).append(move)
    queue = deque((road,0,[]) for road in sorted(moves) if (road==start_road if start_road is not None else road not in seams))
    seen = set()
    best = 0
    while queue:
        road,mask,history = queue.popleft()
        if (road,mask) in seen: continue
        seen.add((road,mask))
        for move in sorted(moves.get(road,[]), key=lambda v:(v['movement']!='straight',v['target_road'],v['lane'])):
            if not history and start_lane is not None and move['lane']!=start_lane:continue
            visited = mask | bits[move['junction_id']]
            best=max(best,bin(visited).count('1'))
            path=history+[move]
            dest=move['target_road']
            if dest not in seams:
                if visited & all_bits == all_bits: return path
            else:
                queue.append((seams[dest],visited,path))
    raise ValueError('No legal boundary-to-boundary all-junction route; maximum coverage %s/%s' % (best,len(junctions)))
