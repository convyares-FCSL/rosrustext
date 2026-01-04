#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
DEV_WS_ROOT="${DEV_WS_ROOT:-$HOME/fcsl/rosrustext_dev_ws}"
EXAMPLE_DIR="${DEV_WS_ROOT}/examples/ros2_rust_lifecycle_minimal"
NODE_NAME="/ros2_rust_lifecycle_gate_minimal"
ROS2_TIMEOUT="${ROS2_TIMEOUT:-5}"
NODE_START_TIMEOUT="${NODE_START_TIMEOUT:-60}"
NODE_STOP_TIMEOUT="${NODE_STOP_TIMEOUT:-60}"
CHANGE_STATE_DELAY_MS="${CHANGE_STATE_DELAY_MS:-500}"
STATE_WAIT_TIMEOUT="${STATE_WAIT_TIMEOUT:-5}"
EVENT_TIMEOUT="${EVENT_TIMEOUT:-5}"
LOG_DIR="${LOG_DIR:-$ROOT_DIR/logs/run_all/ros2_rust}"
SCRIPT_NAME="$(basename "$0" .sh)"
NODE_LOG="${LOG_DIR}/${SCRIPT_NAME}.log"
EVENT_LOG="${LOG_DIR}/${SCRIPT_NAME}_event.log"

fail() { echo "error: $*" >&2; exit 1; }

source_setup() {
  local setup_file="$1"
  if [[ -f "$setup_file" ]]; then
    set +u
    # shellcheck disable=SC1091
    source "$setup_file"
    set -u
  fi
}

command -v timeout >/dev/null 2>&1 || fail "timeout not found (install coreutils)."
command -v python3 >/dev/null 2>&1 || fail "python3 not found"

mkdir -p "${LOG_DIR}"

source_setup /opt/ros/jazzy/setup.bash
if [[ -f "${DEV_WS_ROOT}/scripts/source_env.sh" ]]; then
  source_setup "${DEV_WS_ROOT}/scripts/source_env.sh"
elif [[ -f "${DEV_WS_ROOT}/source_env.sh" ]]; then
  source_setup "${DEV_WS_ROOT}/source_env.sh"
fi

if [[ ! -d "${EXAMPLE_DIR}" ]]; then
  fail "example dir not found: ${EXAMPLE_DIR}"
fi

ros2_daemon_ready() {
  timeout "${ROS2_TIMEOUT}s" ros2 daemon status >/dev/null 2>&1 || \
    timeout "${ROS2_TIMEOUT}s" ros2 daemon start >/dev/null 2>&1 || \
    fail "ros2 daemon not responding"
}

restart_ros2_daemon() {
  timeout "${ROS2_TIMEOUT}s" ros2 daemon stop >/dev/null 2>&1 || true
  timeout "${ROS2_TIMEOUT}s" ros2 daemon start >/dev/null 2>&1 || true
}

force_kill_node_processes() {
  if command -v pkill >/dev/null 2>&1; then
    pkill -TERM -f "ros2_rust_lifecycle_gate_minimal" >/dev/null 2>&1 || true
    sleep 1
    pkill -KILL -f "ros2_rust_lifecycle_gate_minimal" >/dev/null 2>&1 || true
  fi
}

ros2_node_list() {
  local out
  if ! out="$(timeout "${ROS2_TIMEOUT}s" ros2 node list 2>/dev/null)"; then
    fail "ros2 node list timed out"
  fi
  printf '%s\n' "${out}"
}

wait_for_node() {
  local node="$1"
  local timeout_s="$2"
  local start
  start="$(date +%s)"
  while true; do
    if ros2_node_list | grep -qx "${node}"; then
      return 0
    fi
    if (( $(date +%s) - start >= timeout_s )); then
      return 1
    fi
    sleep 0.1
  done
}

wait_for_node_gone() {
  local node="$1"
  local timeout_s="$2"
  local start
  start="$(date +%s)"
  while true; do
    if ! ros2_node_list | grep -qx "${node}"; then
      return 0
    fi
    if (( $(date +%s) - start >= timeout_s )); then
      return 1
    fi
    sleep 0.1
  done
}

wait_for_service() {
  local service="$1"
  local timeout_s="$2"
  local start
  start="$(date +%s)"
  while true; do
    local out
    if out="$(timeout "${ROS2_TIMEOUT}s" ros2 service list 2>/dev/null)"; then
      if echo "${out}" | grep -qx "${service}"; then
        return 0
      fi
    fi
    if (( $(date +%s) - start >= timeout_s )); then
      return 1
    fi
    sleep 0.1
  done
}

ros2_daemon_ready

if ros2_node_list | grep -qx "${NODE_NAME}"; then
  echo "info: detected ${NODE_NAME}; attempting shutdown"
  timeout 10s ros2 lifecycle set "${NODE_NAME}" shutdown >/dev/null 2>&1 || true
  echo "info: waiting up to ${NODE_STOP_TIMEOUT}s for ${NODE_NAME} to stop"
  wait_for_node_gone "${NODE_NAME}" "${NODE_STOP_TIMEOUT}" || true
  if ros2_node_list | grep -qx "${NODE_NAME}"; then
    echo "info: restarting ros2 daemon to clear stale node list"
    restart_ros2_daemon
  fi
  if ros2_node_list | grep -qx "${NODE_NAME}"; then
    echo "info: forcing shutdown of ${NODE_NAME} process"
    force_kill_node_processes
    wait_for_node_gone "${NODE_NAME}" "${NODE_STOP_TIMEOUT}" || true
    restart_ros2_daemon
  fi
  if ros2_node_list | grep -qx "${NODE_NAME}"; then
    fail "${NODE_NAME} already running; stop it before running this script"
  fi
fi

echo "== start demo node (cargo run) =="
pushd "${EXAMPLE_DIR}" >/dev/null
if command -v setsid >/dev/null 2>&1; then
  ROSRUSTEXT_RCLRS_CHANGE_STATE_DELAY_MS="${CHANGE_STATE_DELAY_MS}" \
    setsid cargo run --quiet >"${NODE_LOG}" 2>&1 &
  NODE_PGID=$!
else
  ROSRUSTEXT_RCLRS_CHANGE_STATE_DELAY_MS="${CHANGE_STATE_DELAY_MS}" \
    cargo run --quiet >"${NODE_LOG}" 2>&1 &
fi
NODE_PID=$!
popd >/dev/null

cleanup() {
  if [[ -n "${EVENT_PID:-}" ]]; then
    kill "${EVENT_PID}" >/dev/null 2>&1 || true
    wait "${EVENT_PID}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${NODE_PGID:-}" ]]; then
    kill -INT -- "-${NODE_PGID}" >/dev/null 2>&1 || true
  fi
  kill "${NODE_PID}" >/dev/null 2>&1 || true
  wait "${NODE_PID}" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

if ! wait_for_node "${NODE_NAME}" "${NODE_START_TIMEOUT}"; then
  if ! kill -0 "${NODE_PID}" >/dev/null 2>&1; then
    echo "error: node process exited before appearing; see ${NODE_LOG}" >&2
    if [[ -f "${NODE_LOG}" ]]; then
      tail -n 40 "${NODE_LOG}" >&2
    fi
  fi
  fail "node not detected within ${NODE_START_TIMEOUT}s: ${NODE_NAME}"
fi

wait_for_service "${NODE_NAME}/get_state" "${ROS2_TIMEOUT}" || \
  fail "service not detected: ${NODE_NAME}/get_state"
wait_for_service "${NODE_NAME}/change_state" "${ROS2_TIMEOUT}" || \
  fail "service not detected: ${NODE_NAME}/change_state"

python3 - <<'PY'
import os
import sys
import time

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, TransitionEvent

node_name = os.environ.get("NODE_NAME", "/ros2_rust_lifecycle_gate_minimal")
delay_ms = int(os.environ.get("CHANGE_STATE_DELAY_MS", "500"))
state_wait_timeout = float(os.environ.get("STATE_WAIT_TIMEOUT", "5"))
event_timeout = float(os.environ.get("EVENT_TIMEOUT", "5"))

threshold_ms = max(200, delay_ms - 150)

rclpy.init()
node = Node("rosrustext_change_state_timing_checker")

change_client = node.create_client(ChangeState, f"{node_name}/change_state")
get_client = node.create_client(GetState, f"{node_name}/get_state")

if not change_client.wait_for_service(timeout_sec=5.0):
    print(f"error: change_state service not available at {node_name}/change_state", file=sys.stderr)
    sys.exit(1)
if not get_client.wait_for_service(timeout_sec=5.0):
    print(f"error: get_state service not available at {node_name}/get_state", file=sys.stderr)
    sys.exit(1)

event_msg = {"msg": None}
def on_event(msg):
    event_msg["msg"] = msg

node.create_subscription(TransitionEvent, f"{node_name}/transition_event", on_event, 10)

def get_state_label():
    req = GetState.Request()
    future = get_client.call_async(req)
    while rclpy.ok() and not future.done():
        rclpy.spin_once(node, timeout_sec=0.01)
    if not future.done():
        return None
    return future.result().current_state.label

def wait_for_state(label, timeout_s):
    start = time.monotonic()
    while time.monotonic() - start < timeout_s:
        cur = get_state_label()
        if cur == label:
            return True
        time.sleep(0.05)
    return False

cur = get_state_label()
if cur not in ("Unconfigured", "Inactive", "Active"):
    print(f"error: unexpected initial state {cur}", file=sys.stderr)
    sys.exit(1)
if cur == "Active":
    # deactivate then cleanup
    req = ChangeState.Request()
    req.transition.id = Transition.TRANSITION_DEACTIVATE
    change_client.call(req)
    wait_for_state("Inactive", 2.0)
    req.transition.id = Transition.TRANSITION_CLEANUP
    change_client.call(req)
    wait_for_state("Unconfigured", 2.0)
elif cur == "Inactive":
    req = ChangeState.Request()
    req.transition.id = Transition.TRANSITION_CLEANUP
    change_client.call(req)
    wait_for_state("Unconfigured", 2.0)

req = ChangeState.Request()
req.transition.id = Transition.TRANSITION_CONFIGURE

start = time.monotonic()
future = change_client.call_async(req)
while rclpy.ok() and not future.done():
    rclpy.spin_once(node, timeout_sec=0.01)
elapsed_ms = int((time.monotonic() - start) * 1000)

if elapsed_ms >= threshold_ms:
    print(f"error: change_state took {elapsed_ms}ms (expected < {threshold_ms}ms)", file=sys.stderr)
    sys.exit(1)

cur = get_state_label()
if cur != "Unconfigured":
    print(f"error: expected Unconfigured immediately after change_state, got {cur}", file=sys.stderr)
    sys.exit(1)

if not wait_for_state("Inactive", state_wait_timeout):
    print(f"error: state did not converge to Inactive within {state_wait_timeout}s", file=sys.stderr)
    sys.exit(1)

start = time.monotonic()
while time.monotonic() - start < event_timeout:
    rclpy.spin_once(node, timeout_sec=0.05)
    if event_msg["msg"] is not None:
        break
if event_msg["msg"] is None:
    print(f"error: transition_event not observed within {event_timeout}s", file=sys.stderr)
    sys.exit(1)
if event_msg["msg"].transition.label != "configure":
    print(f"error: transition_event label mismatch: {event_msg['msg'].transition.label}", file=sys.stderr)
    sys.exit(1)

print("ok: change_state returned quickly; state + event converged")
rclpy.shutdown()
PY

echo "== done =="
