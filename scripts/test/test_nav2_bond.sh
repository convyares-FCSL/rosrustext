#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TARGET_NODE="${TARGET_NODE:-hyfleet_ring_roslibrust}"
BRIDGE_URL="${BRIDGE_URL:-ws://localhost:9090}"
BRIDGE_PORT="${BRIDGE_PORT:-}"
LOG_DIR="${LOG_DIR:-$ROOT_DIR/logs/run_all}"
STARTUP_DELAY="${STARTUP_DELAY:-1}"
STARTUP_TIMEOUT="${STARTUP_TIMEOUT:-30}"
NAV2_WAIT_TIMEOUT="${NAV2_WAIT_TIMEOUT:-20}"
NON_BRIDGE_GRACE_SECONDS="${NON_BRIDGE_GRACE_SECONDS:-2}"
ROSBRIDGE_GRACE_SECONDS="${ROSBRIDGE_GRACE_SECONDS:-4}"
SKIP_PROCESS_CHECK="${SKIP_PROCESS_CHECK:-0}"
AUTO_KILL_ROSBRIDGE="${AUTO_KILL_ROSBRIDGE:-1}"
AUTO_KILL_BACKEND="${AUTO_KILL_BACKEND:-1}"
ROSBRIDGE_NODE_NAME="${ROSBRIDGE_NODE_NAME:-$TARGET_NODE}"
ROS2_WS_ROOT="${ROS2_WS_ROOT:-/home/ecm/ros2_rust_ws/ros2_ws}"

mkdir -p "$LOG_DIR"
: >"$LOG_DIR/rosbridge.log"
: >"$LOG_DIR/backend.log"
: >"$LOG_DIR/proxy.log"
: >"$LOG_DIR/nav2_manager.log"

NAMES=()
PIDS=()
PGIDS=()
wait_for_pids() {
  local timeout_s="$1"
  shift
  local start
  start="$(date +%s)"
  while true; do
    local alive=0
    local pid
    for pid in "$@"; do
      if kill -0 "$pid" 2>/dev/null; then
        alive=1
        break
      fi
    done
    if (( alive == 0 )); then
      return 0
    fi
    if (( $(date +%s) - start >= timeout_s )); then
      return 1
    fi
    sleep 0.2
  done
}
cleanup() {
  log "shutting down..."

  local non_bridge_pids=()
  local bridge_pids=()
  for (( idx=${#PIDS[@]}-1; idx>=0; idx-- )); do
    if [[ "${NAMES[idx]}" == "rosbridge" ]]; then
      bridge_pids+=("${PIDS[idx]}")
    else
      non_bridge_pids+=("${PIDS[idx]}")
    fi
  done

  for (( idx=${#PGIDS[@]}-1; idx>=0; idx-- )); do
    name="${NAMES[idx]}"
    pgid="${PGIDS[idx]}"
    if [[ "$name" != "rosbridge" ]] && kill -0 -- "-$pgid" 2>/dev/null; then
      kill -INT -- "-$pgid" 2>/dev/null || true
    fi
  done
  for (( idx=${#PIDS[@]}-1; idx>=0; idx-- )); do
    name="${NAMES[idx]}"
    pid="${PIDS[idx]}"
    if [[ "$name" != "rosbridge" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
    fi
  done

  wait_for_pids "$NON_BRIDGE_GRACE_SECONDS" "${non_bridge_pids[@]}" || true

  for (( idx=${#PGIDS[@]}-1; idx>=0; idx-- )); do
    name="${NAMES[idx]}"
    pgid="${PGIDS[idx]}"
    if [[ "$name" == "rosbridge" ]] && kill -0 -- "-$pgid" 2>/dev/null; then
      kill -INT -- "-$pgid" 2>/dev/null || true
    fi
  done
  for (( idx=${#PIDS[@]}-1; idx>=0; idx-- )); do
    name="${NAMES[idx]}"
    pid="${PIDS[idx]}"
    if [[ "$name" == "rosbridge" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
    fi
  done

  wait_for_pids "$ROSBRIDGE_GRACE_SECONDS" "${bridge_pids[@]}" || true

  if [[ "$AUTO_KILL_ROSBRIDGE" == "1" ]]; then
    if [[ -n "${BRIDGE_PORT:-}" ]] && port_in_use "$BRIDGE_PORT"; then
      BRIDGE_URL="$BRIDGE_URL" TIMEOUT=2 FORCE=1 "$ROOT_DIR/scripts/kill/kill_rosbridge.sh" >/dev/null 2>&1 || true
    fi
  fi
}
trap cleanup EXIT INT TERM

start_bg() {
  local name="$1"
  shift
  if command -v setsid >/dev/null 2>&1; then
    setsid "$@" >"$LOG_DIR/${name}.log" 2>&1 &
    local pid=$!
    NAMES+=("$name")
    PIDS+=("$pid")
    PGIDS+=("$pid")
  else
    "$@" >"$LOG_DIR/${name}.log" 2>&1 &
    NAMES+=("$name")
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

wait_for_service() {
  local service="$1"
  local timeout_s="$2"
  local start
  start="$(date +%s)"
  while true; do
    if ros2 service list 2>/dev/null | rg -q "^${service}$"; then
      return 0
    fi
    if (( $(date +%s) - start >= timeout_s )); then
      return 1
    fi
    sleep 0.2
  done
}

wait_for_log() {
  local file="$1"
  local pattern="$2"
  local timeout_s="$3"
  local start
  start="$(date +%s)"
  while true; do
    if [[ -f "$file" ]] && grep -q "$pattern" "$file"; then
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
  if pgrep -f "$TARGET_NODE" >/dev/null 2>&1; then
    if [[ "$AUTO_KILL_BACKEND" == "1" ]]; then
      TARGET_NODE="$TARGET_NODE" FORCE=1 "$ROOT_DIR/scripts/kill/kill_backend.sh" >/dev/null 2>&1 || true
    else
      echo "${TARGET_NODE} already running; stop it or set SKIP_PROCESS_CHECK=1 or AUTO_KILL_BACKEND=1." >&2
      exit 1
    fi
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
    BRIDGE_URL="$BRIDGE_URL" FORCE=1 "$ROOT_DIR/scripts/kill/kill_rosbridge.sh" >/dev/null 2>&1 || true
  else
    echo "Port ${BRIDGE_PORT} already in use; stop existing rosbridge or set BRIDGE_URL to a free port." >&2
    exit 1
  fi
fi

log "starting rosbridge (node name: $ROSBRIDGE_NODE_NAME)"
ROSBRIDGE_NODE_NAME="$ROSBRIDGE_NODE_NAME" TARGET_NODE="$TARGET_NODE" BRIDGE_URL="$BRIDGE_URL" \
  start_bg rosbridge "$ROOT_DIR/scripts/run/run_rosbridge.sh"

if ! wait_for_port "$BRIDGE_PORT" "$STARTUP_TIMEOUT"; then
  echo "rosbridge did not bind to port ${BRIDGE_PORT} within ${STARTUP_TIMEOUT}s (see $LOG_DIR/rosbridge.log)" >&2
  exit 1
fi

sleep "$STARTUP_DELAY"

log "starting backend"
TARGET_NODE="$TARGET_NODE" BRIDGE_URL="$BRIDGE_URL" start_bg backend "$ROOT_DIR/scripts/run/run_backend.sh"

sleep "$STARTUP_DELAY"

if ! wait_for_log "$LOG_DIR/backend.log" "Running \`target/debug/hyfleet_ring_roslibrust\`" "$STARTUP_TIMEOUT"; then
  echo "backend did not start within ${STARTUP_TIMEOUT}s (see $LOG_DIR/backend.log)" >&2
  exit 1
fi

log "starting proxy"
TARGET_NODE="$TARGET_NODE" BRIDGE_URL="$BRIDGE_URL" start_bg proxy "$ROOT_DIR/scripts/run/run_proxy.sh"

sleep "$STARTUP_DELAY"

set +u
source /opt/ros/jazzy/setup.bash
if [[ -f "$ROS2_WS_ROOT/install/setup.bash" ]]; then
  source "$ROS2_WS_ROOT/install/setup.bash"
fi
set -u

if ! wait_for_service "/${TARGET_NODE}/change_state" "$STARTUP_TIMEOUT"; then
  echo "service /${TARGET_NODE}/change_state not visible after ${STARTUP_TIMEOUT}s (see $LOG_DIR/proxy.log)" >&2
  exit 1
fi

log "starting nav2 lifecycle manager"
start_bg nav2_manager bash -lc "source /opt/ros/jazzy/setup.bash; ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -p node_names:=[${TARGET_NODE}] -p autostart:=true"

if ! wait_for_log "$LOG_DIR/nav2_manager.log" "Managed nodes are active" "$NAV2_WAIT_TIMEOUT"; then
  echo "nav2 lifecycle manager did not reach active within ${NAV2_WAIT_TIMEOUT}s (see $LOG_DIR/nav2_manager.log)" >&2
  exit 1
fi

log "nav2 lifecycle manager reports active"
if rg -i -q "failed to transition|failed to change state|transitioning.*failed" "$LOG_DIR/nav2_manager.log"; then
  echo "nav2 lifecycle manager log shows transition failures (see $LOG_DIR/nav2_manager.log)" >&2
  exit 1
fi
log "done (logs: $LOG_DIR)"
