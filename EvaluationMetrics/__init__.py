from pathlib import Path
import sys
_workspace = Path(__file__).resolve().parent.parent
_collector_root = _workspace / 'InformationCollector'
if str(_collector_root) not in sys.path:
    sys.path.insert(0, str(_collector_root))
from evaluation.evaluator import MetricEvaluator
from evaluation.mdrive_metrics import MDriveMetrics
__all__ = ['MetricEvaluator','MDriveMetrics']
