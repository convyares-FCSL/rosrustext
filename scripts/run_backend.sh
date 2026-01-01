#!/usr/bin/env bash
set -euo pipefail

BACKEND_DIR="${BACKEND_DIR:-/home/ecm/ros2_rust_ws/ros2_ws/src/hyfleet_ring_roslibrust}"
TARGET_NODE="${TARGET_NODE:-hyfleet_ring_roslibrust}"
BRIDGE_URL="${BRIDGE_URL:-ws://localhost:9090}"

if [[ ! -d "$BACKEND_DIR" ]]; then
  echo "Backend directory not found: $BACKEND_DIR" >&2
  exit 1
fi

cd "$BACKEND_DIR"
export HYFLEET_NODE_NAME="$TARGET_NODE"
export ROSLIBRUST_BRIDGE_URL="$BRIDGE_URL"
exec cargo run
