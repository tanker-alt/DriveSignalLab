"""One legacy import namespace backed by the two physical modules; no code copies."""
from pathlib import Path
import sys
_here = Path(__file__).resolve().parent
_workspace = _here.parents[1]
for _name in ('InformationCollector', 'EvaluationMetrics'):
    _root = _workspace / _name
    _package = _root / 'evaluation'
    if str(_package) not in __path__:
        __path__.append(str(_package))
    if str(_root) not in sys.path:
        sys.path.append(str(_root))

if str(_workspace) not in sys.path:
    sys.path.append(str(_workspace))
