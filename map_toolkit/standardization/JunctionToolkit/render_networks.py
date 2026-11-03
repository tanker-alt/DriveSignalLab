import json,time
from pathlib import Path
import view
results=[]
for name in ['t_network_random_3x3','t_network_random_6x6','cross_3x3','grid6x6_limit_check']:
    start=time.monotonic()
    print('START',name,flush=True)
    w,d=view.load(Path('standardization/maps')/name,hidden_load=False)
    results.append(dict(map=name,seconds=time.monotonic()-start,images=['overview.png','street.png'],collision_validated=False))
    Path('standardization/network_preview_results.json').write_text(json.dumps(results,indent=2))
    print('DONE',name,flush=True)
