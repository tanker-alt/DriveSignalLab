"""Portable settings and workspace-contained map output paths (Python 3.7+)."""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def settings():
    values = json.loads((ROOT/'settings.json').read_text(encoding='utf-8-sig'))
    local = ROOT/'settings.local.json'
    if local.exists():
        values.update(json.loads(local.read_text(encoding='utf-8-sig')))
    return values


def workspace_root():
    configured = settings().get('workspace_root')
    if configured:
        return (ROOT/configured).resolve()
    # The development workspace keeps the two environments at its root.
    for parent in ROOT.parents:
        if (parent/'generator-env').is_dir() and (parent/'carla-env').is_dir():
            return parent
    return ROOT.parent


def map_output(path):
    resolved = Path(path).resolve()
    try:
        resolved.relative_to(workspace_root())
    except ValueError:
        raise ValueError('Map output must stay inside this toolkit workspace: '+str(workspace_root()))
    return resolved


def python_for(env):
    configured = settings().get(env+'_python')
    if configured:
        executable = (ROOT/configured).resolve()
        if not executable.is_file():
            raise ValueError('Python not found; check settings.local.json: '+str(executable))
        return str(executable)
    for parent in ROOT.parents:
        for relative in ('Scripts/python.exe', 'bin/python'):
            candidate = parent/(env+'-env')/relative
            if candidate.is_file():
                return str(candidate)
    return sys.executable
