#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
: "${MDRIVE_ROOT:?Set MDRIVE_ROOT to a clean MDrive checkout}"
git -C "$MDRIVE_ROOT" apply --check "$ROOT/integrations/lmdrive/patches/mdrive_multiego_signal.patch"
git -C "$MDRIVE_ROOT/simulation/assets/LMDrive" apply --check "$ROOT/integrations/lmdrive/patches/lmdrive_model_notice.patch"
git -C "$MDRIVE_ROOT" apply "$ROOT/integrations/lmdrive/patches/mdrive_multiego_signal.patch"
git -C "$MDRIVE_ROOT/simulation/assets/LMDrive" apply "$ROOT/integrations/lmdrive/patches/lmdrive_model_notice.patch"
cp -R "$ROOT/integrations/lmdrive/overlay/." "$MDRIVE_ROOT/"
cp "$ROOT/traffic_signal/controllers.py" "$MDRIVE_ROOT/common/traffic_signal_control.py"
echo "Installed DriveSignalLab MDrive integration into $MDRIVE_ROOT"
