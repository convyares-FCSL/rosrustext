#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TARGET_NODE="${TARGET_NODE:-hyfleet_ring_roslibrust}"
BRIDGE_URL="${BRIDGE_URL:-ws://localhost:9090}"
BRIDGE_PORT="${BRIDGE_PORT:-}"
LOG_DIR="${LOG_DIR:-$ROOT_DIR/logs/run_all}"
STARTUP_DELAY="${STARTUP_DELAY:-1}"
STARTUP_TIMEOUT="${STARTUP_TIMEOUT:-30}"
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

if ! command -v python3 >/dev/null 2>&1; then
  echo "python3 is required for the lifecycle manager test." >&2
  exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
if [[ -f "$ROS2_WS_ROOT/install/setup.bash" ]]; then
  source "$ROS2_WS_ROOT/install/setup.bash"
elif [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
  source "$ROOT_DIR/install/setup.bash"
fi
set -u

log "running python lifecycle manager test"
TARGET_NODE="$TARGET_NODE" python3 - <<'PY'
import os
import sys
import time

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition

TARGET_NODE = os.environ.get("TARGET_NODE", "hyfleet_ring_roslibrust")
CHANGE_SRV = f"/{TARGET_NODE}/change_state"
GET_SRV = f"/{TARGET_NODE}/get_state"

TRANSITIONS = [
    (Transition.TRANSITION_CONFIGURE, "configure", 2, "Inactive"),
    (Transition.TRANSITION_ACTIVATE, "activate", 3, "Active"),
    (Transition.TRANSITION_DEACTIVATE, "deactivate", 2, "Inactive"),
    (Transition.TRANSITION_CLEANUP, "cleanup", 1, "Unconfigured"),
]

class Manager(Node):
    def __init__(self):
        super().__init__("rosrustext_lifecycle_test_manager")
        self.change_client = self.create_client(ChangeState, CHANGE_SRV)
        self.get_client = self.create_client(GetState, GET_SRV)

    def wait_for_clients(self, timeout_sec: float) -> bool:
        start = time.time()
        while time.time() - start < timeout_sec:
            if self.change_client.wait_for_service(timeout_sec=0.1) and self.get_client.wait_for_service(timeout_sec=0.1):
                return True
        return False

    def call_change_state(self, transition_id: int, label: str, timeout_sec: float = 2.0):
        req = ChangeState.Request()
        req.transition.id = transition_id
        req.transition.label = label
        future = self.change_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            return False
        resp = future.result()
        return resp.success

    def call_get_state(self, timeout_sec: float = 2.0):
        req = GetState.Request()
        future = self.get_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            return None
        return future.result().current_state

    def wait_for_state(self, target_id: int, timeout_sec: float = 6.0):
        start = time.time()
        last = None
        while time.time() - start < timeout_sec:
            state = self.call_get_state()
            if state is None:
                continue
            last = state
            if state.id == target_id:
                return True, last
            time.sleep(0.1)
        return False, last

rclpy.init()
node = Manager()

if not node.wait_for_clients(6.0):
    print("services not available")
    rclpy.shutdown()
    sys.exit(1)

for transition_id, label, goal_id, goal_label in TRANSITIONS:
    ok = node.call_change_state(transition_id, label)
    if not ok:
        print(f"change_state failed id={transition_id} label={label}")
        rclpy.shutdown()
        sys.exit(1)
    ok, last = node.wait_for_state(goal_id)
    if not ok:
        last_id = getattr(last, "id", None)
        last_label = getattr(last, "label", "")
        print(f"state did not reach {goal_label} (id {goal_id}); last={last_id} {last_label}")
        rclpy.shutdown()
        sys.exit(1)

print("python lifecycle manager ok")
node.destroy_node()
rclpy.shutdown()
PY

log "done (logs: $LOG_DIR)"
