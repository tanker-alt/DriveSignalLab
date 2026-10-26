"""Lazy factory preserving the existing Cross3 LMDrive runtime setup."""
import os
from pathlib import Path


def create_agent(config, output, ego_vehicles_num=2):
    config = str(Path(config).resolve())
    output = str(Path(output).resolve())
    import torch
    torch.set_num_threads(4)
    os.environ.setdefault('HF_HUB_OFFLINE', '1')
    os.environ.setdefault('TRANSFORMERS_OFFLINE', '1')
    os.environ.setdefault('LMDRIVE_SKIP_BACKBONE_PRETRAIN', '1')
    os.environ['SAVE_PATH'] = output
    os.environ['ROUTES'] = 'cross3_two_ego'
    os.environ['LMDRIVE_LIGHT_MESSAGES'] = 'off'
    os.environ.pop('LMDRIVE_WAYPOINT_HEAD', None)
    os.chdir(str(Path(__file__).resolve().parents[2]/'Drive/repos/MDrive'))
    from team_code.fixed_signal_lmdrive import FixedSignalLMDrive
    return FixedSignalLMDrive(config, ego_vehicles_num)
