#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
DEV_WS_ROOT="${DEV_WS_ROOT:-$HOME/fcsl/rosrustext_dev_ws}"
EXAMPLE_DIR="${DEV_WS_ROOT}/examples/rosrustext_rosrs"
NODE_NAME="/rosrustext_lifecycle_publisher"
TOPIC_NAME="/lifecycle_chatter"
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
    pkill -TERM -f "rosrustext_lifecycle_publisher" >/dev/null 2>&1 || true
    sleep 1
    pkill -KILL -f "rosrustext_lifecycle_publisher" >/dev/null 2>&1 || true
  fi
}

ros2_node_list() {
  local out
  if ! out="$(timeout "${ROS2_TIMEOUT}s" ros2 node list 2>/dev/null)"; then
    fail "ros2 node list timed out"
  fi
  printf '%s\n' "${out}"
}

ros2_lifecycle_set() {
  local cmd="$1"
  timeout 10s ros2 lifecycle set "${NODE_NAME}" "${cmd}" >/dev/null || \
    fail "ros2 lifecycle set ${cmd} timed out"
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

state_expect() {
  local label="$1"
  local id="$2"
  local out
  out="$(timeout "${ROS2_TIMEOUT}s" ros2 lifecycle get "${NODE_NAME}")" || \
    fail "ros2 lifecycle get timed out"
  echo "${out}" | grep -q "${label} \\[${id}\\]" || \
    fail "expected ${label} [${id}], got: ${out}"
}

wait_for_log_token() {
  local token="$1"
  local timeout_s="$2"
  local start
  start="$(date +%s)"
  while true; do
    if [[ -f "${NODE_LOG}" ]] && grep -q "${token}" "${NODE_LOG}"; then
      return 0
    fi
    if (( $(date +%s) - start >= timeout_s )); then
      return 1
    fi
    sleep 0.1
  done
}

echo "== ensure ROS daemon =="
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

echo "== start demo node =="
if command -v setsid >/dev/null 2>&1; then
  if [[ "${EXAMPLE_VERBOSE:-0}" == "1" ]]; then
    setsid cargo run --manifest-path "${EXAMPLE_DIR}/Cargo.toml" --bin lifecycle_publisher &
  else
    setsid cargo run --manifest-path "${EXAMPLE_DIR}/Cargo.toml" --bin lifecycle_publisher --quiet >"${NODE_LOG}" 2>&1 &
  fi
  NODE_PGID=$!
else
  if [[ "${EXAMPLE_VERBOSE:-0}" == "1" ]]; then
    cargo run --manifest-path "${EXAMPLE_DIR}/Cargo.toml" --bin lifecycle_publisher &
  else
    cargo run --manifest-path "${EXAMPLE_DIR}/Cargo.toml" --bin lifecycle_publisher --quiet >"${NODE_LOG}" 2>&1 &
  fi
fi
NODE_PID=$!

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

state_expect "Unconfigured" "1"

echo "== configure =="
ros2_lifecycle_set configure
state_expect "Inactive" "2"

wait_for_log_token "SUPPRESSED_INACTIVE" "${ROS2_TIMEOUT}" || \
  fail "expected SUPPRESSED_INACTIVE log token after configure"

if timeout 2s ros2 topic echo "${TOPIC_NAME}" --once >/dev/null 2>&1; then
  fail "unexpected publish while inactive"
fi

echo "== activate =="
ros2_lifecycle_set activate
state_expect "Active" "3"

timeout "${ROS2_TIMEOUT}s" ros2 topic echo "${TOPIC_NAME}" --once >/dev/null 2>&1 || \
  fail "expected publish while active"

echo "== deactivate =="
ros2_lifecycle_set deactivate
state_expect "Inactive" "2"

if timeout 2s ros2 topic echo "${TOPIC_NAME}" --once >/dev/null 2>&1; then
  fail "unexpected publish while inactive"
fi

echo "== cleanup =="
ros2_lifecycle_set cleanup
state_expect "Unconfigured" "1"

echo "== reconfigure =="
ros2_lifecycle_set configure
state_expect "Inactive" "2"

echo "== reactivate =="
ros2_lifecycle_set activate
state_expect "Active" "3"

timeout "${ROS2_TIMEOUT}s" ros2 topic echo "${TOPIC_NAME}" --once >/dev/null 2>&1 || \
  fail "expected publish after reconfigure+activate"

echo "== deactivate (final) =="
ros2_lifecycle_set deactivate
state_expect "Inactive" "2"

echo "== cleanup =="
ros2_lifecycle_set cleanup
state_expect "Unconfigured" "1"

echo "== done =="
