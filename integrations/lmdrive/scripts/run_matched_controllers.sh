#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROFILE="${1:-low}"
for controller in fixed maxpressure presslight; do
  "$HERE/run_carla_experiment.sh" "$controller" "$PROFILE"
done
