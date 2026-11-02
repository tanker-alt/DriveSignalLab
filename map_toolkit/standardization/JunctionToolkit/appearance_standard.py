"""Shared appearance settings; no commercial assets are copied into the toolkit."""
import json
from pathlib import Path

def settings(config=None):
    root=Path(__file__).resolve().parent
    result=json.loads((root/'configs/appearance_standard.json').read_text(encoding='utf-8'))
    if config and config.get('type') in ('t','t_junction'):
        template=json.loads((root/'templates/t_cross2/appearance.json').read_text(encoding='utf-8'))
        for key,value in template.items():
            if isinstance(value,dict) and isinstance(result.get(key),dict):
                result[key].update(value)
            else:result[key]=value
    return result
