#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROS2_WS_ROOT="${ROS2_WS_ROOT:-/home/ecm/ros2_rust_ws/ros2_ws}"

if [[ -f "/opt/ros/jazzy/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "/opt/ros/jazzy/setup.bash"
fi

if [[ -f "$ROS2_WS_ROOT/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$ROS2_WS_ROOT/install/setup.bash"
fi

TESTS=(
  "scripts/test/roslibrust/lifecycle/test_transition_graph.sh"
  "scripts/test/roslibrust/lifecycle/test_python_lifecycle_manager.sh"
  "scripts/test/roslibrust/lifecycle/test_nav2_bond.sh"
  "scripts/test/roslibrust/lifecycle/test_lifecycle_stress.sh"
)

failures=()

for test_path in "${TESTS[@]}"; do
  echo "==> ${test_path}"
  if "$ROOT_DIR/$test_path"; then
    echo "PASS: ${test_path}"
  else
    echo "FAIL: ${test_path}"
    failures+=("${test_path}")
  fi
  echo ""
done

if (( ${#failures[@]} > 0 )); then
  echo "Test summary: ${#failures[@]} failed"
  for test_path in "${failures[@]}"; do
    echo " - ${test_path}"
  done
  exit 1
fi

echo "Test summary: all tests passed"
