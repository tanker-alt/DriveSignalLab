# EvaluationMetrics

Consumes same-frame packets from InformationCollector. Includes all-vehicle safety, normalized travel time, queues, throughput and per-junction waiting. All-vehicle SUMO TTC and fuel estimation remain pending.

Install `evaluation/requirements-mdrive.txt` and a matching CARLA Python API. MDrive PDM requires an independently installed `MDrive-main` directory at repository root; its absence must be reported as unavailable, never a passing score.

From repository root, with InformationCollector and EvaluationMetrics on PYTHONPATH:
`python -B -m unittest discover -s EvaluationMetrics/evaluation/tests -p "test_*.py"`

The two evaluation namespaces share a compatibility import path; metric algorithms are not duplicated.
