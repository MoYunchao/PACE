#!/usr/bin/env bash
set -u

cd "$(dirname "$0")/.."
set +u
source install/setup.bash
set -u

failures=0

check_topic() {
  local rid="$1"
  local topic="/robot_${rid}/scan"
  echo "=== ${topic} ==="
  if ! ros2 topic list | grep -Fxq "$topic"; then
    echo "[FAIL] topic missing: $topic"
    failures=$((failures + 1))
    return
  fi
  local type
  type="$(ros2 topic type "$topic" 2>/dev/null || true)"
  if [[ "$type" != "sensor_msgs/msg/LaserScan" ]]; then
    echo "[FAIL] unexpected type: ${type:-unknown}"
    failures=$((failures + 1))
  else
    echo "[OK] type=$type"
  fi
  timeout 6 ros2 topic hz "$topic" --window 10 2>/dev/null || true
  timeout 5 ros2 topic echo "$topic" --once --field header 2>/dev/null || true

  local base="robot_${rid}/base_link"
  local lidar="robot_${rid}/robot_${rid}_livox_frame"
  if timeout 5 ros2 run tf2_ros tf2_echo "$base" "$lidar" 2>/dev/null | grep -q 'Translation'; then
    echo "[OK] TF ${base} -> ${lidar}"
  else
    echo "[FAIL] TF missing: ${base} -> ${lidar}"
    failures=$((failures + 1))
  fi
}

for rid in 0 1 2; do
  check_topic "$rid"
done

if (( failures > 0 )); then
  echo "[2D-LIDAR] failed checks: $failures"
  exit 1
fi

echo "[2D-LIDAR] all topic/type/TF checks passed"
