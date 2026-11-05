"""Static compatibility report: uses generator Python, never imports CARLA."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
from network_appearance_plan import plan


def audit(folder):
    folder=Path(folder);raw=(folder/'map.xodr').read_bytes();d=json.loads((folder/'manifest.json').read_text())
    digest=hashlib.sha256(raw).hexdigest()
    if digest!=d['opendrive_sha256']:raise ValueError('Stale manifest')
    sources={}
    root=Path(__file__).resolve().parent
    for filename in ['network_appearance_plan.py','network_preview.py','network_simple_finish.py','runtime_finish.py','lane_signals.py','ring_sidewalk_finish.py']:
        data=(root/filename).read_bytes();ast.parse(data,filename=filename)
        sources[filename]=hashlib.sha256(data).hexdigest()
    sources['configs/appearance_standard.json']=hashlib.sha256((root/'configs/appearance_standard.json').read_bytes()).hexdigest()
    modes={}
    for mode in ('simple','complete','off'):
        result=plan(d,raw,mode);signalled=[r for r in result['roads'] if r['signal_ids']]
        owned=[sid for r in signalled for sid in r['signal_ids']]
        if len(owned)!=len(set(owned)) or set(owned)!=set(d['signals']):raise ValueError('Incomplete signal coverage')
        ring={rid for ring in d.get('roundabouts',[]) for rid in ring['ring_roads']+ring['ring_continuations']}
        for r in result['roads']:
            if r['road_id'] in ring and (r['centerline'] or r['crosswalk']):raise ValueError('Circulating road has opposing centerline or crosswalk')
        result.update(enabled=mode!='off',mast_count=len(signalled) if mode!='off' else 0,
                      stop_line_count=len(signalled) if mode!='off' else 0,
                      circulating_signal_road_count=sum(r['circulating'] for r in signalled),
                      paint_road_count=len(result['roads']) if mode=='complete' else 0)
        modes[mode]=result
    report=dict(passed=True,scope='static appearance compatibility only',opendrive_sha256=digest,
                source_sha256=sources,modes=modes,carla_imported=False,carla_validation_performed=False,
                runtime_checks_pending=['native signal head alignment','asset loading and visual placement','collision finalization','render performance'])
    (folder/'appearance_compatibility_static.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('folder');args=parser.parse_args()
    result=audit(args.folder)
    print('Static appearance compatible:',result['passed'],{k:(v['mast_count'],v['stop_line_count']) for k,v in result['modes'].items()})
