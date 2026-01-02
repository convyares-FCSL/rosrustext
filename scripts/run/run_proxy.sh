#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TARGET_NODE="${TARGET_NODE:-hyfleet_ring_roslibrust}"
BRIDGE_URL="${BRIDGE_URL:-ws://localhost:9090}"

export ROSRUSTEXT_INTERFACES_PATH="${ROSRUSTEXT_INTERFACES_PATH:-$ROOT_DIR/crates/rosrustext_roslibrust/tools/rosrustext_lifecycle_proxy/interfaces}"

args=(--target-node "$TARGET_NODE" --bridge-url "$BRIDGE_URL")
if [[ -n "${NODE_NAME:-}" ]]; then
  args+=(--node-name "$NODE_NAME")
fi

cd "$ROOT_DIR"
exec cargo run -p rosrustext_lifecycle_proxy -- "${args[@]}"
