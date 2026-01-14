#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
NODE_START_TIMEOUT="${NODE_START_TIMEOUT_OVERRIDE:-60}"
NODE_STOP_TIMEOUT="${NODE_STOP_TIMEOUT_OVERRIDE:-60}"
ROS2_TIMEOUT="${ROS2_TIMEOUT_OVERRIDE:-10}"
SERVICE_TIMEOUT="${SERVICE_TIMEOUT_OVERRIDE:-20}"
NAV2_WAIT_TIMEOUT="${NAV2_WAIT_TIMEOUT_OVERRIDE:-60}"

TESTS=(
  "scripts/test/ros2_rust/lifecycle/test_transition_graph.sh"
  "scripts/test/ros2_rust/lifecycle/test_lifecycle_cli.sh"
  "scripts/test/ros2_rust/lifecycle/test_change_state_timing.sh"
  "scripts/test/ros2_rust/lifecycle/test_busy_rejection.sh"
  "scripts/test/ros2_rust/lifecycle/test_change_state_failure.sh"
  "scripts/test/ros2_rust/lifecycle/test_change_state_error_processing_unconfigured.sh"
  "scripts/test/ros2_rust/lifecycle/test_change_state_error_processing_finalized.sh"
  "scripts/test/ros2_rust/lifecycle/test_bond.sh"
  "scripts/test/ros2_rust/lifecycle/test_nav2_lifecycle_manager.sh"
  "scripts/test/ros2_rust/lifecycle/test_rosrustext_rosrs_user_parity.sh"
)

failures=()

for test_path in "${TESTS[@]}"; do
  echo "==> ${test_path}"
  if NODE_START_TIMEOUT="${NODE_START_TIMEOUT}" \
    NODE_STOP_TIMEOUT="${NODE_STOP_TIMEOUT}" \
    ROS2_TIMEOUT="${ROS2_TIMEOUT}" \
    SERVICE_TIMEOUT="${SERVICE_TIMEOUT}" \
    NAV2_WAIT_TIMEOUT="${NAV2_WAIT_TIMEOUT}" \
    "$ROOT_DIR/$test_path"; then
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
