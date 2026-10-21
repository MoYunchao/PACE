#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'EOF'
用法:
  bash start_feature_ablation.sh --map default|maze40|real \
    --mode full|semantic|distance|gain|risk \
    robot0_battery_capacity_ah:=VALUE \
    robot1_battery_capacity_ah:=VALUE \
    robot2_battery_capacity_ah:=VALUE [其他 launch 参数...]

full：四维固定等权重对照。
其他模式：去掉该维实际信息，遍历参考值 0/0.25/0.5/0.75/1，取最终评分平均。
所有组固定权重为 0.25/0.25/0.25/0.25，不加载或运行 MAPPO 模型。
EOF
  exit 0
fi

MAP_VARIANT="default"
ABLATION_MODE="full"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --map|--mode)
      if [[ $# -lt 2 ]]; then
        echo "$1 requires a value" >&2
        exit 1
      fi
      if [[ "$1" == "--map" ]]; then MAP_VARIANT="$2"; else ABLATION_MODE="$2"; fi
      shift 2
      ;;
    *) break ;;
  esac
done
case "$MAP_VARIANT" in default|maze40|real) ;; *) echo "Unsupported map: $MAP_VARIANT" >&2; exit 1 ;; esac
case "$ABLATION_MODE" in full|semantic|distance|gain|risk) ;; *) echo "Unsupported mode: $ABLATION_MODE" >&2; exit 1 ;; esac

if [[ ! -f install/setup.bash ]]; then
  echo "Build the workspace before running this script." >&2
  exit 1
fi
set +u
source install/setup.bash
set -u

GROUP_NAME="fixed_without_${ABLATION_MODE}"
if [[ "$ABLATION_MODE" == "full" ]]; then GROUP_NAME="fixed_full"; fi
RUN_DIR="${RESULT_ROOT:-$(pwd)/output/feature_ablation_runs}/map_${MAP_VARIANT}/${GROUP_NAME}/$(date +%Y%m%d_%H%M%S_%N)"
LAUNCH_ARGS=(
  map_variant:="$MAP_VARIANT"
  ablation_mode:="$ABLATION_MODE"
  metrics_csv_path:="$RUN_DIR/energy_mission_metrics.csv"
  metrics_summary_path:="$RUN_DIR/summary.json"
  "$@"
)
python3 tools/record_ablation_run.py "$RUN_DIR" "$MAP_VARIANT" "$ABLATION_MODE" "${LAUNCH_ARGS[@]}"
echo "[feature_ablation] map=$MAP_VARIANT mode=$ABLATION_MODE result=$RUN_DIR"
ros2 launch pace_ablation feature_ablation.launch.py "${LAUNCH_ARGS[@]}" 2>&1 | tee "$RUN_DIR/launch.log"
