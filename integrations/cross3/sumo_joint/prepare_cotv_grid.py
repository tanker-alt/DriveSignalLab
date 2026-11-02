#!/usr/bin/env python3
import argparse
from pathlib import Path
from .cotv_grid_1x1 import write_json
def main():
    p=argparse.ArgumentParser(); p.add_argument('--output',type=Path,default=Path('outputs/cross3_sumo_joint/cotv_grid_1x1.json')); a=p.parse_args(); write_json(a.output); print(a.output)
if __name__ == '__main__': main()
