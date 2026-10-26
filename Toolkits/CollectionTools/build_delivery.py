"""Export a fresh allowlisted handoff folder and ZIP without historical raw experiments."""
import argparse
import hashlib
import json
import shutil
from datetime import datetime,timezone
from pathlib import Path


def selected(root):
    names={"DELIVERY_README.txt","run_experiment.py","control_junction_phases.py","control_four_phases.py",
           "demo_four_phase_traffic.py","spawn_junction_traffic.py","traffic_scenario.py",
           "requirements.txt","requirements-visualization.txt"}
    files={root/name for name in names}
    files.add(root/'__init__.py')
    files.add(root.parent/'Toolkits/__init__.py')
    for folder in ("evaluation",):
        files.update(p for p in (root/folder).rglob("*.py")
                     if not any(part in ("outputs","tests","__pycache__") for part in p.relative_to(root).parts))
    files.update(root.parent/'Toolkits/CollectionTools'/name for name in ('__init__.py','selfcheck.py','build_delivery.py',
                                               'map_setup.py','stop_reference_review.py'))
    files.update(root/'evaluation/configs'/name for name in ('approach_metrics.json','traffic.json'))
    files.update(root.parent/"Toolkits/MapTools"/name for name in ("__init__.py","__main__.py"))
    toolkit=root.parent/'Toolkits/JunctionToolkit'
    files.update(toolkit/name for name in ('view.py','scenery.py','lane_signals.py','layout.py',
                                          'validate.py','project_bridge.py','experiment.py','safety_rules.py'))
    for name in ('cross3',):
        files.update(root.parent/'Maps/MapDataset/maps'/name/filename for filename in (
            'map.xodr','manifest.json','config.json','topology_validation.json','safety_rules.json'))
    files.update(root/'map_profiles/generated_cross3'/name for name in (
        'profile.json','inventory.json','approach_geometry.json','detection_config.json'))
    metric_root=root.parent/'EvaluationMetrics'
    files.add(metric_root/'__init__.py')
    files.update(p for p in (metric_root/'evaluation').glob('*.py'))
    files.update(p for p in (metric_root/'evaluation/configs').glob('*.json'))
    files.add(metric_root/'evaluation/requirements-mdrive.txt')
    missing=[str(p) for p in files if not p.is_file()]
    if missing:
        raise FileNotFoundError('Required delivery files missing: '+str(missing))
    return sorted(files)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name",default="carla_handoff_"+datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ"))
    args=parser.parse_args()
    if not args.name or any(ch not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for ch in args.name):
        parser.error("name must use letters, digits, underscore or hyphen")
    root=Path(__file__).resolve().parents[2]/'InformationCollector';destination=root/"dist"/args.name
    if destination.exists() or destination.with_suffix(".zip").exists():
        parser.error("delivery already exists; choose a new name")
    destination.mkdir(parents=True)
    entries=[]
    for source in selected(root):
        relative=source.relative_to(root.parent);target=destination/relative
        target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(str(source),str(target))
        entries.append(dict(path=relative.as_posix(),bytes=source.stat().st_size,
                            sha256=hashlib.sha256(target.read_bytes()).hexdigest()))
    # Never export a sender's arbitrary active pointer or absolute Python paths.
    portable={
        'InformationCollector/evaluation/configs/active_map_profile.json':{'profile':'../../map_profiles/generated_cross3/profile.json'},
        'Toolkits/JunctionToolkit/settings.json':{'generator_python':'','carla_python':'','project':'../../InformationCollector'}}
    for relative,data in portable.items():
        target=destination/relative;target.parent.mkdir(parents=True,exist_ok=True)
        target.write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
        entries.append(dict(path=relative,bytes=target.stat().st_size,sha256=hashlib.sha256(target.read_bytes()).hexdigest()))
    from evaluation.map_profile import read_profile
    read_profile(destination/'InformationCollector/map_profiles/generated_cross3/profile.json')
    (destination/"DELIVERY_MANIFEST.json").write_text(json.dumps(dict(files=entries,
        excluded="All Markdown, experiments/raw outputs, screenshots, reports, tests, archives, caches, other maps and local environment paths",
        note="See InformationCollector/DELIVERY_README.txt. Both folders are required; install MDrive-main alongside them for PDM evaluation. topology_validation.json is retained only because the map loader requires it. Two mutually exclusive signal controllers are included."),indent=2),encoding="utf-8")
    shutil.make_archive(str(destination),"zip",str(destination))
    print(destination);print(destination.with_suffix(".zip"));print("files:",len(entries))


if __name__=="__main__":
    main()
