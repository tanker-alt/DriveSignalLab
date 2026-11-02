"""Exact 1x1 four-way grid parameters from CoTV_grid.py."""
from pathlib import Path
CONFIG = {
    "exp_tag":"CoTV_1x1grid", "network":"TrafficLightGridNetwork", "simulator":"traci",
    "rows":1,"columns":1,"inner_length":300,"long_length":300,"short_length":300,
    "horizontal_lanes":1,"vertical_lanes":1,"cars_left":1,"cars_right":1,"cars_top":1,"cars_bottom":1,
    "target_velocity":15.0,"max_accel":3.0,"max_decel":3.0,"switch_time":3,
    "num_observed":1,"num_local_edges":4,"num_local_lights":4,"safety_device":True,
    "cav_penetration_rate":1.0,"total_veh":70,"sim_step":1.0,"horizon":720,
    "inflows_veh_per_hour":{"right":120,"bottom":240,"left":288,"top":192},
    "inflow_begin_end":{"right":[1,300],"bottom":[1,300],"left":[45,345],"top":[60,360]},
    "ppo":{"gamma":0.999,"lambda":0.97,"num_sgd_iter":10,"num_workers":18,"num_rollouts":18,"train_batch_size":12960}
}
def write_json(path):
    import json
    path=Path(path); path.parent.mkdir(parents=True,exist_ok=True); path.write_text(json.dumps(CONFIG,ensure_ascii=False,indent=2))
if __name__ == '__main__':
    import argparse
    p=argparse.ArgumentParser(); p.add_argument('output',type=Path); a=p.parse_args(); write_json(a.output)
