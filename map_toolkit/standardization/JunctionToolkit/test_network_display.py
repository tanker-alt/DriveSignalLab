"""Four-stage visible CARLA test without tree decoration."""
import json,time,faulthandler
from pathlib import Path
import carla
import view,runtime_finish
folder=Path('standardization/maps/cross_3x3_connected_20261103')
report={'map':'cross_3x3','trees_enabled':False,'stages':[]}
original=runtime_finish.build
start=time.monotonic()
def save():
    (folder/'four_stage_test.json').write_text(json.dumps(report,indent=2))
def surface(*args,**kwargs):
    report['stages'].append({'stage':'load_and_signal_validation','passed':True,'seconds':time.monotonic()-start});save()
    begin=time.monotonic();print('SURFACE BEGIN',flush=True)
    result=original(*args,**kwargs)
    report['stages'].append({'stage':'surface','passed':True,'seconds':time.monotonic()-begin,'actors':len(result['actor_ids'])});save()
    return result
runtime_finish.build=surface
mast=view.build_lane_signals
def signals(*args,**kwargs):
    begin=time.monotonic();result=mast(*args,**kwargs)
    report['stages'].append({'stage':'signal_masts','passed':True,'seconds':time.monotonic()-begin});save();return result
view.build_lane_signals=signals
view.build_trees=lambda *args,**kwargs:dict(count=0,actor_ids=[],rejected={},skipped=True)
faulthandler.dump_traceback_later(60,repeat=True)
try:
    w,d=view.load(folder,hidden_load=False)
    w.get_spectator().set_transform(carla.Transform(carla.Location(350,-350,850),carla.Rotation(pitch=-90,yaw=-90)))
    report['passed']=True;save()
finally:faulthandler.cancel_dump_traceback_later()
