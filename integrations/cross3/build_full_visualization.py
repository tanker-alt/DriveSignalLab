#!/usr/bin/env python3
"""Build an offline, synchronized evidence viewer from saved Cross3 logs."""
import argparse
import base64
import bisect
import io
import json
from pathlib import Path
from PIL import Image

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]

def read(path):
    return json.loads(path.read_text())

def rows(path):
    with path.open() as f:
        return [json.loads(line) for line in f]

def build(root):
    run=root/'two_lmdrive_presslight_adapted_seed2027'
    ep=run/'episode_000'
    signals=rows(ep/'signals.jsonl')
    model_dir=next((run/'lmdrive').iterdir())
    audit=rows(model_dir/'model_audit.jsonl')
    control=[[r for r in audit if r['type']=='final_control' and r['ego_index']==ego] for ego in (0,1)]
    cameras=[]
    for ego in (0,1):
        images=[]
        previous=-100
        for path in sorted((model_dir/('meta_%d'%ego)).glob('*.jpg')):
            if not path.stem.isdigit():
                continue
            # LMDrive save() uses filename = per-ego step - 20.
            step=int(path.stem)+20
            if step>=len(control[ego]) or step-previous<10:
                continue
            record=control[ego][step]
            with Image.open(path) as im:
                # Omit legacy text overlay: its throttle/steer captions were swapped.
                im=im.crop((0,0,1200,690)).resize((600,345))
                b=io.BytesIO();im.save(b,format='JPEG',quality=72)
            images.append(dict(frame=record['camera_frame'],source=path.name,
                               image='data:image/jpeg;base64,'+base64.b64encode(b.getvalue()).decode()))
            previous=step
        cameras.append(images)
    training=[]
    for path in sorted((root/'train_carla_confirmed_seed2026').glob('episode_*/summary.json')):
        d=read(path)
        samples=rows(path.parent/'signals.jsonl')
        training.append(dict(episode=len(training)+1,time=d['elapsed'],completed=d['completed'],planned=d['planned'],
                             queue_seconds=round(sum(sum(s['observation']['lane_queued'])*.05 for s in samples),2),
                             collisions=len(d['collisions'])))
    manifest=read(HERE.parent/'carla_handoff/upstream_20261023_v4/map_dataset/junction_toolkit/maps/cross3/manifest.json')
    return dict(signals=signals[::2]+signals[-1:],vehicles=rows(ep/'vehicles.jsonl'),
                controls=control,cameras=cameras,summary=read(ep/'summary.json'),audit=read(run/'audit.json'),
                manifest=manifest,training=training,comparison=read(root/'initial_training_comparison.json'),
                learner=read(ROOT/'Drive/models/presslight/cross3_carla_seed2026.json'),
                adaptation=read(ROOT/'Drive/models/lmdrive/cross3_waypoint_head.json'))

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,default=ROOT/'Drive/outputs/cross3')
    p.add_argument('--output',type=Path)
    a=p.parse_args()
    data=build(a.root)
    output=a.output or a.root/'complete_process.html'
    template=(HERE/'full_visualization.html').read_text()
    payload=json.dumps(data,separators=(',',':'),ensure_ascii=False).replace('<','\\u003c')
    output.write_text(template.replace('__DATA__',payload))
    (a.root/'visualization_data.json').write_text(json.dumps(data,separators=(',',':'),ensure_ascii=False))
    print(json.dumps(dict(output=str(output),bytes=output.stat().st_size,frames=len(data['signals']),
                         camera_samples=[len(c) for c in data['cameras']],duration=data['summary']['elapsed'])))

if __name__=='__main__':main()
