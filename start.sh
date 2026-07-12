#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi

if [[ -f "$SCRIPT_DIR/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "$SCRIPT_DIR/install/setup.bash"
fi

exec ros2 launch rviz_car_sim bringup.launch.py
