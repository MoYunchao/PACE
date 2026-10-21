#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
if ! command -v colcon >/dev/null 2>&1; then
  echo "colcon is unavailable. Source your ROS 2 Humble environment first." >&2
  exit 1
fi
colcon build --symlink-install "$@"
