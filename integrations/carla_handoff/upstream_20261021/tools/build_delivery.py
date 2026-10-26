"""Export a fresh allowlisted handoff folder and ZIP without historical raw experiments."""
import argparse
import hashlib
import json
import shutil
from datetime import datetime,timezone
from pathlib import Path


def selected(root):
    names={"README.md","control_junction_phases.py","spawn_junction_traffic.py","traffic_scenario.py",
           "requirements.txt","requirements-visualization.txt"}
    files={root/name for name in names}
    for folder in ("evaluation","tools"):
        files.update(p for p in (root/folder).rglob("*.py")
                     if not any(part in ("outputs","__pycache__") for part in p.relative_to(root).parts))
    files.update((root/"evaluation/configs").glob("*.json"))
    files.update((root/"evaluation/configs").glob("*.png"))
    files.update(root/"map_tools"/name for name in ("__init__.py","__main__.py"))
    # Preserve all Markdown, including historical inspection reports, but never recurse into deliveries.
    files.update(p for p in root.rglob("*.md") if not any(
        part in ("dist",".git",".venv","venv","__pycache__") for part in p.relative_to(root).parts))
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
    (destination/"DELIVERY_MANIFEST.json").write_text(json.dumps(dict(files=entries,
        excluded="historical raw outputs, legacy scripts/data, caches, local manifests/logs",
        note="All project Markdown retained; historical links to excluded raw data refer to sender workspace."),indent=2),encoding="utf-8")
    shutil.make_archive(str(destination),"zip",str(destination))
    print(destination);print(destination.with_suffix(".zip"));print("files:",len(entries))


if __name__=="__main__":
    main()
