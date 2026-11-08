"""Evidence-based progression. Gates validate artifacts, never ask for approval."""
import hashlib
import json
from pathlib import Path

METRIC_FILES=('core.py','safety.py','measurements.py','sumo_env.py','carla_env.py',
              'audit_metrics.py','test_metrics_stage1.py','stages.py','stage1.py','test_stages.py')


def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def metric_sources():
    return {name:sha(Path(__file__).parent/name) for name in METRIC_FILES}


def load_receipt(path,stage):
    if path is None: raise ValueError('Stage %d acceptance evidence is required'%stage)
    data=json.loads(Path(path).read_text())
    if data.get('stage')!=stage or not data.get('passed'):
        raise ValueError('Stage %d has not passed; do not skip to the next stage'%stage)
    if data.get('metric_sources')!=metric_sources():
        raise ValueError('Metric/simulator code changed since acceptance; rerun stage 1')
    for artifact,expected in data.get('artifacts',{}).items():
        if not Path(artifact).is_file() or sha(artifact)!=expected:
            raise ValueError('Acceptance evidence missing/changed: '+artifact)
    return data


def checkpoint_in_lineage(checkpoint,provenance,expected):
    if checkpoint and sha(checkpoint)==expected: return True
    current=provenance
    while isinstance(current,dict):
        if current.get('source_checkpoint_sha256')==expected: return True
        current=current.get('source')
    return False


def validate_progress(args,source):
    if args.train or args.controller=='learned' or args.vla:
        load_receipt(getattr(args,'metrics_acceptance',None),1)
    if args.backend=='carla' and args.controller=='learned':
        gate=load_receipt(getattr(args,'sumo_acceptance',None),2)
        if gate.get('milestone')!='sumo_validation': raise ValueError('Expected SUMO validation receipt')
        if not checkpoint_in_lineage(args.checkpoint,source,gate['checkpoint_sha256']):
            raise ValueError('Checkpoint is not descended from the accepted SUMO model')
    if args.vla:
        gate=load_receipt(getattr(args,'carla_acceptance',None),2)
        if gate.get('milestone')!='carla_adaptation': raise ValueError('CARLA adaptation must pass before VLA experiments')
        if not checkpoint_in_lineage(args.checkpoint,source,gate['checkpoint_sha256']):
            raise ValueError('Joint experiment is not based on the accepted CARLA checkpoint')
