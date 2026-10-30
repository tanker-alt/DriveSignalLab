#!/usr/bin/env python3
"""Convert an offline 80-D PressLight checkpoint for the Cross3 CARLA Learner."""
import argparse, json
from pathlib import Path
import torch

def main():
    p=argparse.ArgumentParser(); p.add_argument('--input',type=Path,required=True); p.add_argument('--output',type=Path,required=True); p.add_argument('--manifest',type=Path,default=Path('integrations/carla_handoff/upstream_20261023_v4/map_dataset/junction_toolkit/maps/cross3/manifest.json')); p.add_argument('--right-turn',default='always-green'); a=p.parse_args()
    data=torch.load(str(a.input),map_location='cpu'); model=data['model'] if 'model' in data else data; target=data.get('target',model)
    def rename(weights):
        return {k[4:] if k.startswith('net.') else k: v for k,v in weights.items()}
    model=rename(model); target=rename(target)
    design=json.loads(a.manifest.read_text()); lane_order=[(m['road'],m['lane']) for m in design['movements']]
    metadata=dict(schema='cross3_presslight_carla_v2_official_80d',state_dim=80,action_dim=4,phase_order=['NS_STRAIGHT','NS_LEFT','EW_STRAIGHT','EW_LEFT'],lane_order=lane_order,map_sha256=design['opendrive_sha256'],training_environment='CARLA',scope='offline pretrained; official 80-D state',right_turn=a.right_turn,timing=dict(min_green=5,max_green=20,yellow=3,clearance=2,decision=2))
    a.output.parent.mkdir(parents=True,exist_ok=True); torch.save(dict(metadata=metadata,model=model,target=target,optimizer={},steps=0,updates=0),str(a.output)); a.output.with_suffix('.json').write_text(json.dumps(metadata,indent=2)); print(json.dumps(metadata,indent=2))
if __name__=='__main__': main()
