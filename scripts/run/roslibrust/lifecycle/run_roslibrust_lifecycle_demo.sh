#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
DEMO_NODE_NAME="${DEMO_NODE_NAME:-rosrustext_lifecycle_demo}"
BRIDGE_URL="${BRIDGE_URL:-ws://localhost:9090}"
ROS2_WS_ROOT="${ROS2_WS_ROOT:-/home/ecm/ros2_ws}"

export DEMO_NODE_NAME
export BRIDGE_URL
export ROS2_WS_ROOT

exec "$ROOT_DIR/scripts/test/roslibrust/lifecycle/test_lifecycle_demo.sh" "$@"
