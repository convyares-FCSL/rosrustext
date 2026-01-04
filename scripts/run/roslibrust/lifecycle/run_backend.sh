#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
DEFAULT_ROS2_WS_ROOT="$ROOT_DIR/../ros2_rust_ws/ros2_ws"

if [[ -z "${ROS2_WS_ROOT:-}" ]]; then
  if [[ -d "$DEFAULT_ROS2_WS_ROOT" ]]; then
    ROS2_WS_ROOT="$DEFAULT_ROS2_WS_ROOT"
  else
    ROS2_WS_ROOT="/home/ecm/ros2_rust_ws/ros2_ws"
  fi
fi

BACKEND_DIR="${BACKEND_DIR:-$ROS2_WS_ROOT/src/hyfleet_ring_roslibrust}"
TARGET_NODE="${TARGET_NODE:-hyfleet_ring_roslibrust}"
BRIDGE_URL="${BRIDGE_URL:-ws://localhost:9090}"

export ROSRUSTEXT_INTERFACES_PATH="${ROSRUSTEXT_INTERFACES_PATH:-$ROOT_DIR/interfaces/rosrustext_interfaces}"

if [[ ! -d "$BACKEND_DIR" ]]; then
  echo "Backend directory not found: $BACKEND_DIR" >&2
  exit 1
fi

cd "$BACKEND_DIR"
export HYFLEET_NODE_NAME="$TARGET_NODE"
export ROSLIBRUST_BRIDGE_URL="$BRIDGE_URL"
exec cargo run
