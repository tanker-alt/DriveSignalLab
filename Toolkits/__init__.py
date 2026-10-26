"""Development and map tools."""
from pathlib import Path
import sys
_root=Path(__file__).resolve().parent.parent
for _p in (_root, _root/"InformationCollector", _root/"EvaluationMetrics"):
    if str(_p) not in sys.path: sys.path.append(str(_p))
