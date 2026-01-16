#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
DEFAULT_ROS2_WS_ROOT="$ROOT_DIR/../ros2_rust_ws/ros2_ws"
if [[ -d "/home/ecm/fcsl/ros2_rust_ws/ros2_ws" ]]; then
  DEFAULT_ROS2_WS_ROOT="/home/ecm/fcsl/ros2_rust_ws/ros2_ws"
fi

if [[ -z "${ROS2_WS_ROOT:-}" ]]; then
  if [[ -d "$DEFAULT_ROS2_WS_ROOT" ]]; then
    ROS2_WS_ROOT="$DEFAULT_ROS2_WS_ROOT"
  else
    ROS2_WS_ROOT="/home/ecm/ros2_rust_ws/ros2_ws"
  fi
fi
export ROS2_WS_ROOT

source_ros_setup() {
  local setup_file="$1"
  if [[ -f "$setup_file" ]]; then
    # ROS setup scripts are not nounset-safe.
    set +u
    # shellcheck disable=SC1091
    source "$setup_file"
    set -u
  fi
}

source_ros_setup "/opt/ros/jazzy/setup.bash"
source_ros_setup "$ROS2_WS_ROOT/install/setup.bash"

TESTS=(
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
