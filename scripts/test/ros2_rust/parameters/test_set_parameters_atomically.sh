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
from rcl_interfaces.srv import SetParametersAtomically, GetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

node_name = os.environ.get("NODE_NAME", "/rosrustext_parameters_minimal")

rclpy.init()
node = Node("rosrustext_param_atomic_checker")

set_client = node.create_client(SetParametersAtomically, f"{node_name}/set_parameters_atomically")
get_client = node.create_client(GetParameters, f"{node_name}/get_parameters")

if not set_client.wait_for_service(timeout_sec=5.0):
    print("error: set_parameters_atomically not available", file=sys.stderr)
    sys.exit(1)
if not get_client.wait_for_service(timeout_sec=5.0):
    print("error: get_parameters not available", file=sys.stderr)
    sys.exit(1)

def call_get(names):
    req = GetParameters.Request(names=names)
    future = get_client.call_async(req)
    while rclpy.ok() and not future.done():
        rclpy.spin_once(node, timeout_sec=0.05)
    return future.result()

before = call_get(["timer_period_s", "max_count"])
if before is None:
    print("error: get_parameters failed", file=sys.stderr)
    sys.exit(1)

p1 = Parameter()
p1.name = "timer_period_s"
val1 = ParameterValue()
val1.type = ParameterType.PARAMETER_DOUBLE
val1.double_value = 0.25
p1.value = val1

p2 = Parameter()
p2.name = "max_count"
val2 = ParameterValue()
val2.type = ParameterType.PARAMETER_STRING
val2.string_value = "not_an_int"
p2.value = val2

req = SetParametersAtomically.Request()
req.parameters = [p1, p2]

future = set_client.call_async(req)
while rclpy.ok() and not future.done():
    rclpy.spin_once(node, timeout_sec=0.05)
resp = future.result()
if resp is None:
    print("error: set_parameters_atomically failed", file=sys.stderr)
    sys.exit(1)
if resp.result.successful:
    print("error: expected atomic update to fail", file=sys.stderr)
    sys.exit(1)

after = call_get(["timer_period_s", "max_count"])
if after is None:
    print("error: get_parameters failed after set", file=sys.stderr)
    sys.exit(1)

if before.values[0] != after.values[0] or before.values[1] != after.values[1]:
    print("error: parameters changed despite atomic failure", file=sys.stderr)
    sys.exit(1)

print("ok: atomic rejection kept state unchanged")
rclpy.shutdown()
PY

echo "== done =="
