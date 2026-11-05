import argparse,json
from pathlib import Path
from template_network import build

def main(cfg,out):
 rows,cols=int(cfg.get('rows',3)),int(cfg.get('columns',3))
 types=cfg.get('junction_types',{})
 pure=(cfg.get('type') in ('t','t_junction') or cfg.get('t_probability')==1 or
       len(types)==rows*cols and all(v in ('t','t_junction') for v in types.values()))
 if pure:
  from random_t_network import main as pure_t
  return pure_t(cfg,out)
 m=build(cfg,out)
 m['appearance_standard']={
  'mode':'network',
  'native_opendrive_geometry':True,
  'native_opendrive_collision':True,
  'lane_markings':True,
  'stop_lines_crosswalks':True,
  'traffic_signals':True,
  'sidewalks_curbs':True,
  'trees':False,
  'roadside_props':False,
  'grass_mesh':False,
  'debug_overlays':False,
  'runtime_mesh_collision':False
 }
 p=Path(out)/'manifest.json'; p.write_text(json.dumps(m,indent=2),encoding='utf-8')
 cfg_out=Path(out)/'config.json'; cfg_out.write_text(json.dumps(dict(m['config'],appearance_mode='network',trees=False,grass_mesh=False,roadside_props=False,debug_overlays=False),indent=2),encoding='utf-8')
 return m
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--config',required=True);p.add_argument('--out',required=True);a=p.parse_args();main(json.loads(Path(a.config).read_text(encoding='utf-8-sig')),a.out)
