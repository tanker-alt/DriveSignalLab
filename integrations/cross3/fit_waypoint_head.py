#!/usr/bin/env python3
"""Fit Cross3 waypoint output layer from explicitly expert-driven features."""
import argparse
import json
import math
from pathlib import Path
import runpy
import sys

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'Drive/configs'))
from fit_lmdrive_waypoint_head import targets_from_poses
import torch


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset',type=Path,required=True)
    p.add_argument('--endpoints',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--ridge',type=float,default=1.)
    a=p.parse_args()
    torch.set_num_threads(4)
    data=torch.load(str(a.dataset),map_location='cpu')
    if data.get('driver')!='BasicAgent' or data.get('schema')!='lmdrive_teacher_features_v1':
        raise ValueError('Expert features required')
    endpoints=json.loads(a.endpoints.read_text())
    batches=[]
    for ego,end in enumerate(endpoints):
        rows=sorted([r for r in data['rows'] if r['ego']==ego
                     and math.hypot(r['x']-end['x'],r['y']-end['y'])>12],key=lambda r:r['time'])
        if len(rows)<10:
            raise ValueError('Insufficient expert samples')
        batches.append(targets_from_poses(rows))
    x=torch.cat([b[0] for b in batches])
    y=torch.cat([b[1] for b in batches])
    cfg=runpy.run_path(str(ROOT/'Drive/configs/lmdrive_local_config.py'))['GlobalConfig']()
    state=torch.load(cfg.lmdrive_ckpt,map_location='cpu')['model']
    head={k[len('waypoints_predictor.'):]:v.float() for k,v in state.items() if k.startswith('waypoints_predictor.')}
    del state
    hidden=torch.relu(x@head['0.weight'].T+head['0.bias'])
    baseline=hidden@head['2.weight'].T+head['2.bias']
    scale=hidden.square().mean().sqrt().clamp(min=1e-4)
    features=torch.cat([hidden/scale,torch.ones(len(hidden),1)],dim=1).double()
    if len(features)<features.shape[1]:
        penalty=a.ridge*torch.eye(len(features),dtype=torch.float64)
        dual=torch.linalg.solve(features@features.T+penalty,y.double())
        weights=(features.T@dual).float()
    else:
        penalty=a.ridge*torch.eye(features.shape[1],dtype=torch.float64)
        weights=torch.linalg.solve(features.T@features+penalty,features.T@y.double()).float()
    head['2.weight']=(weights[:-1]/scale).T
    head['2.bias']=weights[-1]
    fitted=hidden@head['2.weight'].T+head['2.bias']
    if not torch.isfinite(fitted).all():
        raise ValueError('Non-finite model output')
    metadata=dict(schema='lmdrive_waypoint_head_v1',dataset=str(a.dataset.resolve()),
        map='cross3',samples=len(x),ego_count=len(endpoints),fit_mode='direct',ridge=a.ridge,
        base_rmse=float((baseline-y).square().mean().sqrt()),
        fitted_rmse=float((fitted-y).square().mean().sqrt()),
        scope='Cross3 expert scene adaptation; requires separate LMDrive closed-loop validation',
        unchanged='vision encoder, QFormer, language model, first waypoint layer; no map steering override')
    a.output.parent.mkdir(parents=True,exist_ok=True)
    torch.save(dict(metadata=metadata,state_dict=head),str(a.output))
    a.output.with_suffix('.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print(json.dumps(metadata,indent=2))


if __name__=='__main__':
    main()
