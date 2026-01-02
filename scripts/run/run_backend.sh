#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BACKEND_DIR="${BACKEND_DIR:-/home/ecm/ros2_rust_ws/ros2_ws/src/hyfleet_ring_roslibrust}"
TARGET_NODE="${TARGET_NODE:-hyfleet_ring_roslibrust}"
BRIDGE_URL="${BRIDGE_URL:-ws://localhost:9090}"

export ROSRUSTEXT_INTERFACES_PATH="${ROSRUSTEXT_INTERFACES_PATH:-$ROOT_DIR/crates/rosrustext_roslibrust/tools/rosrustext_lifecycle_proxy/interfaces}"

if [[ ! -d "$BACKEND_DIR" ]]; then
  echo "Backend directory not found: $BACKEND_DIR" >&2
  exit 1
fi

cd "$BACKEND_DIR"
export HYFLEET_NODE_NAME="$TARGET_NODE"
export ROSLIBRUST_BRIDGE_URL="$BRIDGE_URL"
exec cargo run
