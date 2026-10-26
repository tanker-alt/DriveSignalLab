#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}"
export SDL_VIDEODRIVER=dummy HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1
export LMDRIVE_SKIP_BACKBONE_PRETRAIN=1 TZ=Asia/Shanghai
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=4 MKL_NUM_THREADS=4
PY="${CROSS3_PYTHON:-python3}"
exec "$PY" -u "$HERE/run_cross3.py" "$@"
