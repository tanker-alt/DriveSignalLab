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
    for folder in ("evaluation",):
        files.update(p for p in (root/folder).rglob("*.py")
                     if not any(part in ("outputs","tests","__pycache__") for part in p.relative_to(root).parts))
    files.update(root/'tools'/name for name in ('__init__.py','selfcheck.py','build_delivery.py',
                                               'map_setup.py','stop_reference_review.py'))
    files.update(root/'evaluation/configs'/name for name in ('approach_metrics.json','metrics.json','traffic.json'))
    files.update(root/"map_tools"/name for name in ("__init__.py","__main__.py"))
    toolkit=root/'map_dataset/junction_toolkit'
    files.update(toolkit/name for name in ('view.py','scenery.py','lane_signals.py','layout.py',
                                          'validate.py','project_bridge.py','experiment.py'))
    for name in ('cross3',):
        files.update(toolkit/'maps'/name/filename for filename in (
            'map.xodr','manifest.json','config.json','topology_validation.json'))
    files.update(root/'map_profiles/generated_cross3'/name for name in (
        'profile.json','inventory.json','approach_geometry.json','detection_config.json'))
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
    root=Path(__file__).resolve().parents[1];destination=root/"dist"/args.name
    if destination.exists() or destination.with_suffix(".zip").exists():
        parser.error("delivery already exists; choose a new name")
    destination.mkdir(parents=True)
    entries=[]
    for source in selected(root):
        relative=source.relative_to(root);target=destination/relative
        target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(str(source),str(target))
        entries.append(dict(path=relative.as_posix(),bytes=source.stat().st_size,
                            sha256=hashlib.sha256(target.read_bytes()).hexdigest()))
    # Never export a sender's arbitrary active pointer or absolute Python paths.
    portable={
        'evaluation/configs/active_map_profile.json':{'profile':'../../map_profiles/generated_cross3/profile.json'},
        'map_dataset/junction_toolkit/settings.json':{'generator_python':'','carla_python':'','project':'../..'}}
    for relative,data in portable.items():
        target=destination/relative;target.parent.mkdir(parents=True,exist_ok=True)
        target.write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
        entries.append(dict(path=relative,bytes=target.stat().st_size,sha256=hashlib.sha256(target.read_bytes()).hexdigest()))
    from evaluation.map_profile import read_profile
    read_profile(destination/'map_profiles/generated_cross3/profile.json')
    (destination/"DELIVERY_MANIFEST.json").write_text(json.dumps(dict(files=entries,
        excluded="All Markdown, experiments/raw outputs, screenshots, reports, tests, archives, caches, other maps and local environment paths",
        note="See DELIVERY_README.txt. topology_validation.json is retained only because the map loader requires it. Two mutually exclusive signal controllers are included."),indent=2),encoding="utf-8")
    shutil.make_archive(str(destination),"zip",str(destination))
    print(destination);print(destination.with_suffix(".zip"));print("files:",len(entries))


if __name__=="__main__":
    main()
