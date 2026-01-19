#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
TARGET_NODE="${TARGET_NODE:-hyfleet_ring_roslibrust}"
BRIDGE_URL="${BRIDGE_URL:-ws://localhost:9090}"
PROXY_FEATURES="${PROXY_FEATURES:-}"

args=(--target-node "$TARGET_NODE" --bridge-url "$BRIDGE_URL")
if [[ -n "${NODE_NAME:-}" ]]; then
  args+=(--node-name "$NODE_NAME")
fi

feature_args=()
if [[ -n "$PROXY_FEATURES" ]]; then
  feature_args=(--features "$PROXY_FEATURES")
fi

cd "$ROOT_DIR"
exec cargo run --manifest-path crates/rosrustext_roslibrust/tools/rosrustext_lifecycle_proxy/Cargo.toml \
  "${feature_args[@]}" -- "${args[@]}"
