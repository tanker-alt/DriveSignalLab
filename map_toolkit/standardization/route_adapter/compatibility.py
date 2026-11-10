"""Isolated all-map regression; reports survive native crashes and timeouts."""
import argparse
import hashlib
import json
import subprocess
import sys
import time
from pathlib import Path


def source_hash():
    digest = hashlib.sha256()
    for path in sorted(Path(__file__).parent.glob('*.py')):
        digest.update(path.name.encode('utf-8'))
        digest.update(path.read_bytes().replace(b'\r\n', b'\n'))
    return digest.hexdigest()


def run(dataset, output, timeout=60):
    if timeout <= 0:
        raise ValueError('timeout must be positive')
    folders = sorted(p for p in Path(dataset).iterdir() if (p/'map.xodr').is_file())
    if not folders:
        raise ValueError('No map packages found')
    output = Path(output); output.parent.mkdir(parents=True, exist_ok=True)
    result = dict(schema_version='map_adapter_compatibility.1', adapter_sha256=source_hash(),
                  server_connected=False, completed=False, passed=False, maps=[])
    def save():
        temporary = output.with_suffix(output.suffix+'.tmp')
        temporary.write_text(json.dumps(result, indent=2, allow_nan=False), encoding='utf-8')
        temporary.replace(output)
    save()
    for folder in folders:
        start = time.monotonic()
        row = dict(map=folder.name, opendrive_sha256=hashlib.sha256((folder/'map.xodr').read_bytes()).hexdigest(),
                   passed=False, od_count=None, failure_kind=None)
        # Invoke this file so neither installed package names nor the caller's cwd matter.
        try:
            child = subprocess.run([sys.executable, '-B', str(Path(__file__).resolve()), '--worker', str(folder.resolve())],
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=timeout)
            if child.returncode:
                row.update(failure_kind='worker_failure', exit_code=child.returncode, error=child.stderr[-4000:])
            else:
                row.update(json.loads(child.stdout))
        except subprocess.TimeoutExpired:
            row.update(failure_kind='timeout', error='Exceeded {} seconds'.format(timeout))
        except (ValueError, OSError) as exc:
            row.update(failure_kind='infrastructure_failure', error=str(exc))
        row['elapsed_s'] = round(time.monotonic()-start, 6)
        result['maps'].append(row); save()
        print(json.dumps(row), flush=True)
    result.update(completed=True, passed=all(row['passed'] for row in result['maps']))
    save()
    return result


def worker(folder):
    from unittest.mock import patch
    import carla
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from route_adapter import adapt
    try:
        with patch.object(carla, 'Client', side_effect=AssertionError('Offline worker attempted a connection')):
            result = adapt(folder)
        failed = [r for r in result['topology_checks'] if not r['passed']]
        print(json.dumps(dict(passed=result['passed'], od_count=len(result['route_catalog']),
                              classification=result['classification'], diagnostics=result['diagnostics'],
                              failed_checks=failed, failure_kind=None if result['passed'] else 'adaptation_failure')))
    except Exception as exc:
        print(json.dumps(dict(passed=False, failure_kind='adaptation_failure',
                              error_code=getattr(exc, 'code', type(exc).__name__), error=str(exc))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--timeout', type=float, default=60)
    parser.add_argument('--worker', type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.worker:
        worker(args.worker)
    else:
        if args.dataset is None or args.output is None:
            parser.error('--dataset and --output are required')
        raise SystemExit(0 if run(args.dataset, args.output, args.timeout)['passed'] else 1)


if __name__ == '__main__':
    main()
