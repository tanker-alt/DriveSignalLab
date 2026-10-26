#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "${CROSS3_PYTHON:-python3}" -u "$HERE/serve_live.py" --launch "$@"
