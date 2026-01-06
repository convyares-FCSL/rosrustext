#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
DEV_WS_ROOT="${DEV_WS_ROOT:-$HOME/fcsl/rosrustext_dev_ws}"
EXAMPLE_DIR="${DEV_WS_ROOT}/examples/ros2_rust_lifecycle_minimal"
EXAMPLE_CARGO="${EXAMPLE_DIR}/Cargo.toml"
NODE_NAME="/ros2_rust_lifecycle_gate_minimal"
NODE_ID="${NODE_NAME#/}"
ROS2_TIMEOUT="${ROS2_TIMEOUT:-5}"
NODE_START_TIMEOUT="${NODE_START_TIMEOUT:-60}"
NODE_STOP_TIMEOUT="${NODE_STOP_TIMEOUT:-60}"
NAV2_WAIT_TIMEOUT="${NAV2_WAIT_TIMEOUT:-40}"
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

if ! rg -q 'rosrustext_ros2_rust.*features.*bond' "${EXAMPLE_CARGO}"; then
  fail "bond feature not enabled in ${EXAMPLE_CARGO} (enable features = [\"bond\", ...])"
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

ros2_node_list_contains() {
  local node="$1"
  local out=""
  out="$(ros2_node_list)"
  grep -Fxq "${node}" <<< "${out}"
}

ros2_lifecycle_get() {
  timeout "${ROS2_TIMEOUT}s" ros2 lifecycle get "${NODE_NAME}"
}

wait_for_node() {
  local node="$1"
  local timeout_s="$2"
  local start
  start="$(date +%s)"
  while true; do
    if ros2_node_list_contains "${node}"; then
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
    if ! ros2_node_list_contains "${node}"; then
      return 0
    fi
    if (( $(date +%s) - start >= timeout_s )); then
      return 1
    fi
    sleep 0.1
  done
}

bond_echo_once() {
  timeout 3s ros2 topic echo /bond bond/msg/Status \
    --qos-reliability reliable \
    --qos-durability transient_local \
    --qos-history keep_last \
    --qos-depth 1 \
    --once
}

expect_bond_active() {
  local expected="$1"
  local out=""
  local last_msg=""
  for _ in {1..10}; do
    if out="$(bond_echo_once 2>/dev/null || true)"; then
      local id_ok=false
      if echo "${out}" | grep -Fq "id: ${NODE_ID}" || echo "${out}" | grep -Fq "id: ${NODE_NAME}"; then
        id_ok=true
      fi
      if ${id_ok} && echo "${out}" | grep -Eiq "active: ${expected}"; then
        echo "info: bond active=${expected} observed for id=${NODE_ID}"
        return 0
      fi
      last_msg="${out}"
    fi
  done
  if [[ -n "${last_msg}" ]]; then
    echo "info: last /bond message seen:" >&2
    echo "${last_msg}" >&2
  fi
  fail "did not see bond active=${expected} for id=${NODE_ID}"
}

wait_for_active_state() {
  local timeout_s="$1"
  local start
  start="$(date +%s)"
  while true; do
    local out=""
    if out="$(ros2_lifecycle_get 2>/dev/null || true)"; then
      if echo "${out}" | grep -q "Active \\[3\\]"; then
        return 0
      fi
    fi
    if (( $(date +%s) - start >= timeout_s )); then
      return 1
    fi
    sleep 0.2
  done
}

ros2_daemon_ready
restart_ros2_daemon

set +o pipefail
if ! timeout "${ROS2_TIMEOUT}s" ros2 pkg list 2>/dev/null | grep -qx "nav2_lifecycle_manager"; then
  echo "skip: nav2_lifecycle_manager package not found; install nav2 or source its overlay" >&2
  exit 0
fi
set -o pipefail

if ros2_node_list_contains "${NODE_NAME}"; then
  echo "info: detected ${NODE_NAME}; attempting shutdown"
  timeout 10s ros2 lifecycle set "${NODE_NAME}" shutdown >/dev/null 2>&1 || true
  echo "info: waiting up to ${NODE_STOP_TIMEOUT}s for ${NODE_NAME} to stop"
  wait_for_node_gone "${NODE_NAME}" "${NODE_STOP_TIMEOUT}" || true
  if ros2_node_list_contains "${NODE_NAME}"; then
    echo "info: restarting ros2 daemon to clear stale node list"
    restart_ros2_daemon
  fi
  if ros2_node_list_contains "${NODE_NAME}"; then
    echo "info: forcing shutdown of ${NODE_NAME} process"
    force_kill_node_processes
    wait_for_node_gone "${NODE_NAME}" "${NODE_STOP_TIMEOUT}" || true
    restart_ros2_daemon
  fi
  if ros2_node_list_contains "${NODE_NAME}"; then
    fail "${NODE_NAME} already running; stop it before running this script"
  fi
fi

echo "== start demo node (cargo run) =="
pushd "${EXAMPLE_DIR}" >/dev/null
if command -v setsid >/dev/null 2>&1; then
  if [[ "${NAV2_VERBOSE:-0}" == "1" ]]; then
    setsid cargo run &
  else
    setsid cargo run --quiet >"${NODE_LOG}" 2>&1 &
  fi
  NODE_PGID=$!
else
  if [[ "${NAV2_VERBOSE:-0}" == "1" ]]; then
    cargo run &
  else
    cargo run --quiet >"${NODE_LOG}" 2>&1 &
  fi
fi
NODE_PID=$!
popd >/dev/null

cleanup() {
  if [[ -n "${NAV2_PGID:-}" ]]; then
    kill -INT -- "-${NAV2_PGID}" >/dev/null 2>&1 || true
    wait "${NAV2_PGID}" >/dev/null 2>&1 || true
  elif [[ -n "${NAV2_PID:-}" ]]; then
    kill "${NAV2_PID}" >/dev/null 2>&1 || true
    wait "${NAV2_PID}" >/dev/null 2>&1 || true
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

echo "== start nav2 lifecycle manager =="
if command -v setsid >/dev/null 2>&1; then
  setsid ros2 run nav2_lifecycle_manager lifecycle_manager \
    --ros-args -p node_names:="['ros2_rust_lifecycle_gate_minimal']" -p autostart:=True &
  NAV2_PGID=$!
  NAV2_PID=$!
else
  ros2 run nav2_lifecycle_manager lifecycle_manager \
    --ros-args -p node_names:="['ros2_rust_lifecycle_gate_minimal']" -p autostart:=True &
  NAV2_PID=$!
fi

if ! wait_for_active_state "${NAV2_WAIT_TIMEOUT}"; then
  fail "node did not reach Active [3] within ${NAV2_WAIT_TIMEOUT}s"
fi

expect_bond_active "true"

echo "== done =="
