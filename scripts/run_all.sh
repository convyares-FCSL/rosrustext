#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET_NODE="${TARGET_NODE:-hyfleet_ring_roslibrust}"
BRIDGE_URL="${BRIDGE_URL:-ws://localhost:9090}"
LOG_DIR="${LOG_DIR:-$ROOT_DIR/logs/run_all}"
STARTUP_DELAY="${STARTUP_DELAY:-1}"
STARTUP_TIMEOUT="${STARTUP_TIMEOUT:-6}"
SKIP_PROCESS_CHECK="${SKIP_PROCESS_CHECK:-0}"
AUTO_KILL_ROSBRIDGE="${AUTO_KILL_ROSBRIDGE:-0}"
ROSBRIDGE_NODE_NAME="${ROSBRIDGE_NODE_NAME:-$TARGET_NODE}"

mkdir -p "$LOG_DIR"
: >"$LOG_DIR/rosbridge.log"
: >"$LOG_DIR/backend.log"
: >"$LOG_DIR/proxy.log"

PIDS=()
PGIDS=()
cleanup() {
  for (( idx=${#PGIDS[@]}-1; idx>=0; idx-- )); do
    pgid="${PGIDS[idx]}"
    if kill -0 -- "-$pgid" 2>/dev/null; then
      kill -INT -- "-$pgid" 2>/dev/null || true
    fi
  done
  for (( idx=${#PIDS[@]}-1; idx>=0; idx-- )); do
    pid="${PIDS[idx]}"
    if kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
    fi
  done
  sleep 0.4
  for (( idx=${#PGIDS[@]}-1; idx>=0; idx-- )); do
    pgid="${PGIDS[idx]}"
    if kill -0 -- "-$pgid" 2>/dev/null; then
      kill -- "-$pgid" 2>/dev/null || true
    fi
  done
  for (( idx=${#PIDS[@]}-1; idx>=0; idx-- )); do
    pid="${PIDS[idx]}"
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
      pkill -P "$pid" 2>/dev/null || true
    fi
  done
  BRIDGE_URL="$BRIDGE_URL" FORCE=1 "$ROOT_DIR/scripts/kill_rosbridge.sh" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

start_bg() {
  local name="$1"
  shift
  if command -v setsid >/dev/null 2>&1; then
    setsid "$@" >"$LOG_DIR/${name}.log" 2>&1 &
    local pid=$!
    PIDS+=("$pid")
    PGIDS+=("$pid")
  else
    "$@" >"$LOG_DIR/${name}.log" 2>&1 &
    PIDS+=($!)
  fi
}

port_from_url() {
  local url="$1"
  if [[ "$url" =~ ^ws://[^:/]+:([0-9]+)$ ]]; then
    echo "${BASH_REMATCH[1]}"
  else
    return 1
  fi
}

port_in_use() {
  local port="$1"
  if command -v ss >/dev/null 2>&1; then
    ss -ltn | awk '{print $4}' | grep -qE ":${port}$"
  elif command -v lsof >/dev/null 2>&1; then
    lsof -iTCP:"$port" -sTCP:LISTEN -n >/dev/null 2>&1
  elif command -v netstat >/dev/null 2>&1; then
    netstat -ltn 2>/dev/null | awk '{print $4}' | grep -qE ":${port}$"
  else
    return 1
  fi
}

wait_for_port() {
  local port="$1"
  local timeout_s="$2"
  local start
  start="$(date +%s)"
  while true; do
    if port_in_use "$port"; then
      return 0
    fi
    if (( $(date +%s) - start >= timeout_s )); then
      return 1
    fi
    sleep 0.2
  done
}

log() {
  echo "[$(date +%H:%M:%S)] $*"
}

if [[ "$SKIP_PROCESS_CHECK" != "1" ]]; then
  if pgrep -f "hyfleet_ring_roslibrust" >/dev/null 2>&1; then
    echo "hyfleet_ring_roslibrust already running; stop it or set SKIP_PROCESS_CHECK=1." >&2
    exit 1
  fi
  if pgrep -f "rosrustext_lifecycle_proxy" >/dev/null 2>&1; then
    echo "rosrustext_lifecycle_proxy already running; stop it or set SKIP_PROCESS_CHECK=1." >&2
    exit 1
  fi
fi

if ! BRIDGE_PORT="$(port_from_url "$BRIDGE_URL")"; then
  echo "BRIDGE_URL must be ws://host:port (got: $BRIDGE_URL)" >&2
  exit 1
fi
if [[ "$SKIP_PROCESS_CHECK" != "1" ]]; then
  if port_in_use "$BRIDGE_PORT" && pgrep -f "rosbridge_websocket" >/dev/null 2>&1; then
    echo "rosbridge_websocket already running; stop it or set SKIP_PROCESS_CHECK=1." >&2
    exit 1
  fi
fi
if port_in_use "$BRIDGE_PORT"; then
  if [[ "$AUTO_KILL_ROSBRIDGE" == "1" ]]; then
    BRIDGE_URL="$BRIDGE_URL" FORCE=1 "$ROOT_DIR/scripts/kill_rosbridge.sh" >/dev/null 2>&1 || true
  else
    echo "Port ${BRIDGE_PORT} already in use; stop existing rosbridge or set BRIDGE_URL to a free port." >&2
    exit 1
  fi
fi

log "starting rosbridge (node name: $ROSBRIDGE_NODE_NAME)"
ROSBRIDGE_NODE_NAME="$ROSBRIDGE_NODE_NAME" TARGET_NODE="$TARGET_NODE" BRIDGE_URL="$BRIDGE_URL" \
  start_bg rosbridge "$ROOT_DIR/scripts/run_rosbridge.sh"

if ! wait_for_port "$BRIDGE_PORT" "$STARTUP_TIMEOUT"; then
  echo "rosbridge did not bind to port ${BRIDGE_PORT} within ${STARTUP_TIMEOUT}s (see $LOG_DIR/rosbridge.log)" >&2
  exit 1
fi

sleep "$STARTUP_DELAY"

log "starting backend"
TARGET_NODE="$TARGET_NODE" BRIDGE_URL="$BRIDGE_URL" start_bg backend "$ROOT_DIR/scripts/run_backend.sh"

sleep "$STARTUP_DELAY"

log "starting proxy"
TARGET_NODE="$TARGET_NODE" BRIDGE_URL="$BRIDGE_URL" start_bg proxy "$ROOT_DIR/scripts/run_proxy.sh"

sleep "$STARTUP_DELAY"

log "running lifecycle test"
TARGET_NODE="$TARGET_NODE" "$ROOT_DIR/scripts/run_lifecycle_test.sh"

log "done (logs: $LOG_DIR)"
