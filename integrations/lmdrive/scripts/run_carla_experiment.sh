#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
: "${MDRIVE_ROOT:?Set MDRIVE_ROOT to the patched MDrive checkout}"
: "${CARLA_ROOT:?Set CARLA_ROOT to the CARLA 0.9.12 installation}"
: "${LMDRIVE_RUNTIME:?Set LMDRIVE_RUNTIME to the runtime directory containing env/bin/python}"
: "${LMDRIVE_LLM_DIR:?Set LMDRIVE_LLM_DIR to the LLaVA directory}"
: "${LMDRIVE_VISION_CHECKPOINT:?Set LMDRIVE_VISION_CHECKPOINT to the vision checkpoint}"
: "${LMDRIVE_CHECKPOINT:?Set LMDRIVE_CHECKPOINT to the LMDrive checkpoint}"
CONTROLLER="${1:-fixed}"
PROFILE="${2:-low}"
SEED="${EXPERIMENT_SEED:-2000}"
case "$CONTROLLER" in fixed|maxpressure|presslight) ;; *) echo "controller must be fixed, maxpressure, or presslight" >&2; exit 2;; esac
case "$PROFILE" in none|low|medium|high) ;; *) echo "profile must be none, low, medium, or high" >&2; exit 2;; esac
if [[ "$CONTROLLER" == presslight ]]; then : "${PRESSLIGHT_CHECKPOINT:?Set PRESSLIGHT_CHECKPOINT to a trained .npz file}"; fi
export PYTHONPATH="$ROOT:$CARLA_ROOT/PythonAPI/carla${PYTHONPATH:+:$PYTHONPATH}"
export LMDRIVE_SIGNAL_CONTROLLER="$CONTROLLER"
export BACKGROUND_TRAFFIC_PROFILE="$PROFILE"
export BACKGROUND_TRAFFIC_SEED="${BACKGROUND_TRAFFIC_SEED:-$SEED}"
export BACKGROUND_DRAIN_TIMEOUT_SECONDS="${BACKGROUND_DRAIN_TIMEOUT_SECONDS:-120}"
export LMDRIVE_LIGHT_MESSAGES="${LMDRIVE_LIGHT_MESSAGES:-off}"
export HF_HUB_OFFLINE="${HF_HUB_OFFLINE:-1}" TRANSFORMERS_OFFLINE="${TRANSFORMERS_OFFLINE:-1}"
export LMDRIVE_SKIP_BACKBONE_PRETRAIN=1 SDL_VIDEODRIVER=dummy PYTHONUNBUFFERED=1
TAG="lmdrive_signal_${CONTROLLER}_2ego_${PROFILE}_seed_${SEED}_$(date +%Y%m%d_%H%M%S)"
RESULTS_ROOT="${EXPERIMENT_OUTPUT_ROOT:-$ROOT/outputs}"
mkdir -p "$RESULTS_ROOT/$TAG" "$MDRIVE_ROOT/results/results_driving_custom"
ln -s "$RESULTS_ROOT/$TAG" "$MDRIVE_ROOT/results/results_driving_custom/$TAG"
cd "$MDRIVE_ROOT"
exec "$LMDRIVE_RUNTIME/env/bin/python" tools/run_custom_eval.py \
  --planner lmdrive --agent simulation/leaderboard/team_code/fixed_signal_lmdrive.py \
  --agent-config "$ROOT/integrations/lmdrive/config/agent_config.py" \
  --scenario-parameter "$ROOT/integrations/lmdrive/config/scenario.yaml" \
  --routes-dir "$ROOT/integrations/lmdrive/config/routes" \
  --start-carla --carla-quality epic --port "${CARLA_PORT:-31500}" --gpu "${CARLA_GPU:-1}" \
  --carla-seed "$SEED" --traffic-seed "$SEED" --results-tag "$TAG" \
  --timeout "${ROUTE_TIMEOUT:-120}" --scenario-retry-limit 1
