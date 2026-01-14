#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
DEV_WS_ROOT="${DEV_WS_ROOT:-$HOME/fcsl/rosrustext_dev_ws}"
EXAMPLE_DIR="${DEV_WS_ROOT}/examples/ros2_rust_parameters_minimal"
NODE_NAME="/rosrustext_parameters_minimal"
ROS2_TIMEOUT="${ROS2_TIMEOUT:-5}"
NODE_START_TIMEOUT="${NODE_START_TIMEOUT:-60}"
NODE_STOP_TIMEOUT="${NODE_STOP_TIMEOUT:-60}"
LOG_DIR="${LOG_DIR:-$ROOT_DIR/logs/run_all/ros2_rust}"
SCRIPT_NAME="$(basename "$0" .sh)"
NODE_LOG="${LOG_DIR}/${SCRIPT_NAME}.log"

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
    pkill -TERM -f "rosrustext_parameters_minimal" >/dev/null 2>&1 || true
    sleep 1
    pkill -KILL -f "rosrustext_parameters_minimal" >/dev/null 2>&1 || true
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

ros2_daemon_ready

if ros2_node_list | grep -qx "${NODE_NAME}"; then
  echo "info: detected ${NODE_NAME}; attempting shutdown"
  echo "info: forcing shutdown of ${NODE_NAME} process"
  force_kill_node_processes
  wait_for_node_gone "${NODE_NAME}" "${NODE_STOP_TIMEOUT}" || true
  if ros2_node_list | grep -qx "${NODE_NAME}"; then
    echo "info: restarting ros2 daemon to clear stale node list"
    restart_ros2_daemon
  fi
  if ros2_node_list | grep -qx "${NODE_NAME}"; then
    fail "${NODE_NAME} already running; stop it before running this script"
  fi
fi

echo "== start demo node (cargo run) =="
pushd "${EXAMPLE_DIR}" >/dev/null
if command -v setsid >/dev/null 2>&1; then
  if [[ "${EXAMPLE_VERBOSE:-0}" == "1" ]]; then
    setsid cargo run --bin parameter_node &
  else
    setsid cargo run --bin parameter_node --quiet >"${NODE_LOG}" 2>&1 &
  fi
  NODE_PGID=$!
else
  if [[ "${EXAMPLE_VERBOSE:-0}" == "1" ]]; then
    cargo run --bin parameter_node &
  else
    cargo run --bin parameter_node --quiet >"${NODE_LOG}" 2>&1 &
  fi
fi
NODE_PID=$!
popd >/dev/null

cleanup() {
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

python3 - <<'PY'
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_parameter_events
from rcl_interfaces.msg import ParameterEvent, Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters, SetParametersAtomically

node_name = os.environ.get("NODE_NAME", "/rosrustext_parameters_minimal")

rclpy.init()
node = Node("rosrustext_param_event_checker")

set_client = node.create_client(SetParameters, f"{node_name}/set_parameters")
atomic_client = node.create_client(SetParametersAtomically, f"{node_name}/set_parameters_atomically")

if not set_client.wait_for_service(timeout_sec=5.0):
    print("error: set_parameters not available", file=sys.stderr)
    sys.exit(1)
if not atomic_client.wait_for_service(timeout_sec=5.0):
    print("error: set_parameters_atomically not available", file=sys.stderr)
    sys.exit(1)

seen = {"count": 0, "last": None}

def on_event(msg: ParameterEvent):
    seen["count"] += 1
    seen["last"] = msg

node.create_subscription(ParameterEvent, f"{node_name}/parameter_events", on_event, qos_profile_parameter_events)

p = Parameter()
p.name = "timer_period_s"
val = ParameterValue()
val.type = ParameterType.PARAMETER_DOUBLE
val.double_value = 0.75
p.value = val

req = SetParameters.Request()
req.parameters = [p]
future = set_client.call_async(req)
while rclpy.ok() and not future.done():
    rclpy.spin_once(node, timeout_sec=0.05)
resp = future.result()
if resp is None or not all(r.successful for r in resp.results):
    print("error: set_parameters failed", file=sys.stderr)
    sys.exit(1)

start = time.monotonic()
while time.monotonic() - start < 5.0 and seen["count"] < 1:
    rclpy.spin_once(node, timeout_sec=0.05)

if seen["count"] != 1:
    print(f"error: expected 1 parameter_event after set, got {seen['count']}", file=sys.stderr)
    sys.exit(1)

last = seen["last"]
changed_names = {p.name for p in last.changed_parameters}
if "timer_period_s" not in changed_names and "timer_period_s" not in {p.name for p in last.new_parameters}:
    print("error: parameter_event missing timer_period_s", file=sys.stderr)
    sys.exit(1)

p2 = Parameter()
p2.name = "max_count"
val2 = ParameterValue()
val2.type = ParameterType.PARAMETER_STRING
val2.string_value = "bad"
p2.value = val2

req2 = SetParametersAtomically.Request()
req2.parameters = [p2]
future = atomic_client.call_async(req2)
while rclpy.ok() and not future.done():
    rclpy.spin_once(node, timeout_sec=0.05)
resp2 = future.result()
if resp2 is None or resp2.result.successful:
    print("error: expected atomic set to fail", file=sys.stderr)
    sys.exit(1)

start = time.monotonic()
while time.monotonic() - start < 2.0:
    rclpy.spin_once(node, timeout_sec=0.05)

if seen["count"] != 1:
    print("error: unexpected parameter_event on rejected atomic update", file=sys.stderr)
    sys.exit(1)

print("ok: parameter_events emitted on success only")
rclpy.shutdown()
PY

echo "== done =="
