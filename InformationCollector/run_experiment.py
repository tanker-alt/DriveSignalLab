"""Load the bundled Cross3 map and run the existing evaluation baseline."""
import argparse
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
TOOLKIT = ROOT.parent / 'Toolkits/JunctionToolkit'
MAP = ROOT.parent / 'Maps/MapDataset/maps/cross3'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepare-only', action='store_true', help='Load/decorate map without spawning traffic')
    parser.add_argument('--vehicles-per-approach', type=int, choices=range(1, 7), default=3)
    parser.add_argument('--seed', type=int, default=17)
    parser.add_argument('--blueprint', default='vehicle.tesla.model3')
    parser.add_argument('--timeout', type=float, default=600)
    parser.add_argument('--output-root', type=Path, default=ROOT / 'evaluation/outputs/experiments')
    args = parser.parse_args()
    import carla
    client = carla.Client('localhost', 2000)
    client.set_timeout(15)
    if client.get_client_version() != '0.9.15' or client.get_server_version() != '0.9.15':
        raise RuntimeError('This handoff requires CARLA client and server 0.9.15')
    subprocess.run([sys.executable, '-B', str(TOOLKIT / 'view.py'), str(MAP), '--no-photos'], check=True)
    from evaluation.default_experiment import connect
    connect()  # Check the bundled profile and signal adapter before starting children.
    if args.prepare_only:
        print('Cross3 ready: junction 10, four approaches with three incoming lanes each.')
        return
    subprocess.run([sys.executable, '-u', '-B', str(TOOLKIT / 'experiment.py'), str(MAP),
                    '--project', str(ROOT), '--output-root', str(args.output_root.resolve()),
                    '--vehicles-per-approach', str(args.vehicles_per_approach),
                    '--seed', str(args.seed), '--blueprint', args.blueprint,
                    '--timeout', str(args.timeout)], check=True)


if __name__ == '__main__':
    try:
        main()
    except subprocess.CalledProcessError as exc:
        raise SystemExit(exc.returncode)
