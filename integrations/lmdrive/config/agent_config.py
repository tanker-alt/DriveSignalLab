"""Portable LMDrive configuration for the two-ego signal experiment."""
import os
from pathlib import Path
import runpy

MDRIVE_ROOT = Path(os.environ["MDRIVE_ROOT"]).resolve()
base = MDRIVE_ROOT / "simulation/leaderboard/team_code/agent_config/lmdriver_config_8_10.py"
BaseConfig = runpy.run_path(str(base))["GlobalConfig"]


class GlobalConfig(BaseConfig):
    llm_model = os.environ["LMDRIVE_LLM_DIR"]
    preception_model_ckpt = os.environ["LMDRIVE_VISION_CHECKPOINT"]
    lmdrive_ckpt = os.environ["LMDRIVE_CHECKPOINT"]
    agent_use_notice = False
    launch_assist = False
    pid_speed_mode = "first_segment"
    inference_stride = 2
    expected_vehicle_model = "vehicle.lincoln.mkz_2017"
    navigation_mode = "route_geometry"
