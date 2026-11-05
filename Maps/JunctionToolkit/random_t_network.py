"""Connected pure T networks using the existing two-lane T geometry template."""
import argparse
import json
from pathlib import Path
from generate import generate


def main(config,out):
    cfg=dict(config)
    rows,columns=int(cfg.get('rows',3)),int(cfg.get('columns',3))
    types=cfg.pop('junction_types',{})
    expected={'%d,%d'%(r,c) for r in range(rows) for c in range(columns)}
    if set(types)-expected or any(value not in ('t','t_junction') for value in types.values()):
        raise ValueError('Pure T networks cannot contain cross junctions')
    if cfg.pop('t_probability',1)!=1:
        raise ValueError('Pure T networks require t_probability=1')
    cfg.pop('allowed_turns',None)
    cfg.pop('arm_turns',None)
    cfg.update(type='t',rows=rows,columns=columns,incoming_lanes=2,
               random_missing_arms=not bool(cfg.get('missing_arms')))
    return generate(cfg,out)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',required=True);parser.add_argument('--out',required=True)
    args=parser.parse_args()
    main(json.loads(Path(args.config).read_text(encoding='utf-8-sig')),args.out)
