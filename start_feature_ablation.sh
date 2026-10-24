#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'EOF'
用法:
  bash start_feature_ablation.sh --map default|maze40|real \
    --mode full|semantic|distance|gain|risk \
    --model /path/to/models/best_model.pt \
    robot0_battery_capacity_ah:=VALUE \
    robot1_battery_capacity_ah:=VALUE \
    robot2_battery_capacity_ah:=VALUE [其他 launch 参数...]

full：完整 PACE，正常使用四维实际信息。
其他模式：去掉该维实际信息，遍历参考值 0/0.25/0.5/0.75/1，取最终评分平均。
所有组加载指定的同一个 MAPPO 模型，保留动态权重，不重新训练。
EOF
  exit 0
fi

MAP_VARIANT="default"
ABLATION_MODE="full"
MODEL_PATH=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --map|--mode|--model)
      if [[ $# -lt 2 ]]; then
        echo "$1 requires a value" >&2
        exit 1
      fi
      case "$1" in
        --map) MAP_VARIANT="$2" ;;
        --mode) ABLATION_MODE="$2" ;;
        --model) MODEL_PATH="$2" ;;
      esac
      shift 2
      ;;
    *) break ;;
  esac
done
case "$MAP_VARIANT" in default|maze40|real) ;; *) echo "Unsupported map: $MAP_VARIANT" >&2; exit 1 ;; esac
case "$ABLATION_MODE" in full|semantic|distance|gain|risk) ;; *) echo "Unsupported mode: $ABLATION_MODE" >&2; exit 1 ;; esac
if [[ -z "$MODEL_PATH" || ! -f "$MODEL_PATH" ]]; then
  echo "Provide an existing MAPPO checkpoint with --model /path/to/best_model.pt" >&2
  exit 1
fi

if [[ ! -f install/setup.bash ]]; then
  echo "Build the workspace before running this script." >&2
  exit 1
fi
set +u
source install/setup.bash
set -u

GROUP_NAME="mappo_without_${ABLATION_MODE}"
if [[ "$ABLATION_MODE" == "full" ]]; then GROUP_NAME="mappo_full"; fi
RUN_DIR="${RESULT_ROOT:-$(pwd)/output/mappo_feature_ablation_runs}/map_${MAP_VARIANT}/${GROUP_NAME}/$(date +%Y%m%d_%H%M%S_%N)"
LAUNCH_ARGS=(
  map_variant:="$MAP_VARIANT"
  ablation_mode:="$ABLATION_MODE"
  mappo_model_path:="$MODEL_PATH"
  infer_save_root:="$RUN_DIR"
  metrics_csv_path:="$RUN_DIR/energy_mission_metrics.csv"
  metrics_summary_path:="$RUN_DIR/summary.json"
  "$@"
)
python3 tools/record_ablation_run.py "$RUN_DIR" "$MAP_VARIANT" "$ABLATION_MODE" "${LAUNCH_ARGS[@]}"
echo "[feature_ablation] map=$MAP_VARIANT mode=$ABLATION_MODE result=$RUN_DIR"
ros2 launch pace_ablation feature_ablation.launch.py "${LAUNCH_ARGS[@]}" 2>&1 | tee "$RUN_DIR/launch.log"
