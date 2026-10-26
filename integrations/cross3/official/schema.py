"""Shared wire schema for the Python 3.8 environment and Python 3.6 agent."""
PHASE_NAMES=('NS_STRAIGHT','NS_LEFT','EW_STRAIGHT','EW_LEFT')
# Official four-phase bit order: WL, WS, EL, ES, NL, NS, SL, SS.
# Action indices follow CARLA's phase order; bits retain upstream semantics.
PHASE_BITS=[[int(j in slots) for j in range(8)] for slots in ((5,7),(4,6),(1,3),(0,2))]


def position_bin(distance):
    """Match upstream np.digitize bins [0, 50, 100, 150] (upper edge excluded)."""
    return int(distance/50) if 0<=distance<150 else None
