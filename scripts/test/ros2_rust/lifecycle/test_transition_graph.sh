#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
DEV_WS_ROOT="${DEV_WS_ROOT:-$HOME/fcsl/rosrustext_dev_ws}"
EXAMPLE_DIR="${DEV_WS_ROOT}/examples/ros2_rust_lifecycle_minimal"
EXAMPLE_CARGO="${EXAMPLE_DIR}/Cargo.toml"
NODE_NAME="/ros2_rust_lifecycle_gate_minimal"
SERVICE_NAME="${NODE_NAME}/get_transition_graph"
ROS2_TIMEOUT="${ROS2_TIMEOUT:-5}"
NODE_START_TIMEOUT="${NODE_START_TIMEOUT:-60}"
NODE_STOP_TIMEOUT="${NODE_STOP_TIMEOUT:-60}"
SERVICE_TIMEOUT="${SERVICE_TIMEOUT:-10}"
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

if [[ ! -f "${EXAMPLE_CARGO}" ]]; then
  fail "example Cargo.toml not found: ${EXAMPLE_CARGO}"
fi

if ! rg -q 'rosrustext_rosrs.*features.*transition_graph' "${EXAMPLE_CARGO}"; then
  fail "transition_graph feature not enabled in ${EXAMPLE_CARGO} (enable features = [\"transition_graph\", ...])"
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

ros2_service_list() {
  local out
  if ! out="$(timeout "${ROS2_TIMEOUT}s" ros2 service list 2>/dev/null)"; then
    fail "ros2 service list timed out"
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
    if ros2_service_list | grep -qx "${service}"; then
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
  if [[ "${EXAMPLE_VERBOSE:-0}" == "1" ]]; then
    setsid cargo run &
  else
    setsid cargo run --quiet >"${NODE_LOG}" 2>&1 &
  fi
  NODE_PGID=$!
else
  if [[ "${EXAMPLE_VERBOSE:-0}" == "1" ]]; then
    cargo run &
  else
    cargo run --quiet >"${NODE_LOG}" 2>&1 &
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

if ! wait_for_service "${SERVICE_NAME}" "${SERVICE_TIMEOUT}"; then
  fail "service not detected within ${SERVICE_TIMEOUT}s: ${SERVICE_NAME}"
fi

response="$(timeout "${ROS2_TIMEOUT}s" ros2 service call "${SERVICE_NAME}" \
  rosrustext_interfaces/srv/GetTransitionGraph "{}")" || \
  fail "service call timed out"

echo "${response}" | grep -q "label='Unconfigured'"
echo "${response}" | grep -q "label='Inactive'"
echo "${response}" | grep -q "label='Active'"
echo "${response}" | grep -q "label='Finalized'"
echo "${response}" | grep -q "label='configure'"
echo "${response}" | grep -q "label='activate'"
echo "${response}" | grep -q "label='deactivate'"
echo "${response}" | grep -q "label='cleanup'"

echo "ok: ${SERVICE_NAME} returned primary states + transitions"
