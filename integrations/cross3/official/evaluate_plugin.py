#!/usr/bin/env python3
"""Evaluate a signal plugin using the official CARLA environment and safety interlock."""
import argparse
import json
from pathlib import Path
import shutil
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from agent_interfaces import load_plugin, OfficialSignalCallback


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--signal-agent', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--port', type=int, default=31610)
    parser.add_argument('--seed', type=int, default=8101)
    parser.add_argument('--demand', choices=['balanced', 'east_west', 'north_south', 'left_heavy'], default='balanced')
    parser.add_argument('--horizon', type=int, default=120)
    parser.add_argument('--drain-seconds', type=int, default=300)
    args = parser.parse_args()
    if args.horizon <= 0 or args.drain_seconds <= 0:
        parser.error('horizon and drain-seconds must be positive')
    from environment import Environment, demand
    args.output.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(args.signal_agent, args.output/'signal_agent.json')
    agent = load_plugin(args.signal_agent, 'signal')
    try:
        callback = OfficialSignalCallback(agent)
        env = Environment(args.port, args.output)
        summary, _ = env.run(args.output/'episode_000', demand(args.seed, args.demand, args.horizon),
                             callback, args.seed, limit=args.horizon+args.drain_seconds)
        (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    finally:
        agent.close()


if __name__ == '__main__':
    main()
