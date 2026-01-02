#!/usr/bin/env bash
set -euo pipefail

TARGET_NODE="${TARGET_NODE:-hyfleet_ring_roslibrust}"
LIFECYCLE_TIMEOUT="${LIFECYCLE_TIMEOUT:-5}"
SERVICE_WAIT_TIMEOUT="${SERVICE_WAIT_TIMEOUT:-6}"
SERVICE_READY_DELAY="${SERVICE_READY_DELAY:-1}"
TOPIC_WAIT_TIMEOUT="${TOPIC_WAIT_TIMEOUT:-6}"
STATE_WAIT_TIMEOUT="${STATE_WAIT_TIMEOUT:-5}"
STATE_POLL_INTERVAL="${STATE_POLL_INTERVAL:-0.2}"

log() {
  echo "[$(date +%H:%M:%S)] $*"
}

run_cmd() {
  local label="$1"
  shift
  log "$label"
  set +e
  local output
  output=$(timeout "${LIFECYCLE_TIMEOUT}s" "$@" 2>&1)
  local status=$?
  set -e
  if [[ $status -eq 124 ]]; then
    log "$label timed out after ${LIFECYCLE_TIMEOUT}s"
  elif [[ $status -ne 0 ]]; then
    log "$label failed (exit $status)"
  fi
  if [[ -n "$output" ]]; then
    echo "$output"
  fi
}

capture_state() {
  timeout "${LIFECYCLE_TIMEOUT}s" ros2 lifecycle get "/${TARGET_NODE}" 2>&1
}

assert_state_not() {
  local forbidden="$1"
  local label="$2"
  log "$label"
  set +e
  local output
  output=$(capture_state)
  local status=$?
  set -e
  if [[ $status -eq 124 ]]; then
    log "$label timed out after ${LIFECYCLE_TIMEOUT}s"
    return 1
  fi
  if [[ $status -ne 0 ]]; then
    log "$label failed (exit $status)"
    [[ -n "$output" ]] && echo "$output"
    return 1
  fi
  if echo "$output" | rg -q "^${forbidden} \\["; then
    log "$label expected not ${forbidden}"
    echo "$output"
    return 1
  fi
  [[ -n "$output" ]] && echo "$output"
}

wait_for_state() {
  local expected="$1"
  local timeout_s="$2"
  local start
  start="$(date +%s)"
  while true; do
    set +e
    local output
    output=$(capture_state)
    local status=$?
    set -e
    if [[ $status -eq 0 ]] && echo "$output" | rg -q "^${expected} \\["; then
      echo "$output"
      return 0
    fi
    if (( $(date +%s) - start >= timeout_s )); then
      log "state did not reach ${expected} within ${timeout_s}s"
      [[ -n "$output" ]] && echo "$output"
      return 1
    fi
    sleep "$STATE_POLL_INTERVAL"
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

wait_for_topic() {
  local topic="$1"
  local timeout_s="$2"
  local start
  start="$(date +%s)"
  while true; do
    if ros2 topic list 2>/dev/null | rg -q "^${topic}$"; then
      return 0
    fi
    if (( $(date +%s) - start >= timeout_s )); then
      return 1
    fi
    sleep 0.2
  done
}

set +u
source /opt/ros/jazzy/setup.bash
set -u

# Wait for lifecycle services to appear on the ROS graph.
if ! wait_for_service "/${TARGET_NODE}/change_state" "$SERVICE_WAIT_TIMEOUT"; then
  log "service /${TARGET_NODE}/change_state not visible after ${SERVICE_WAIT_TIMEOUT}s"
else
  sleep "$SERVICE_READY_DELAY"
fi

# Capture transition events while we exercise lifecycle commands.
echo_pid=""
if wait_for_topic "/${TARGET_NODE}/transition_event" "$TOPIC_WAIT_TIMEOUT"; then
  log "topic /${TARGET_NODE}/transition_event visible; subscribing (capture ${LIFECYCLE_TIMEOUT}s)"
  sleep 0.2
  timeout "${LIFECYCLE_TIMEOUT}s" ros2 topic echo "/${TARGET_NODE}/transition_event" &
  echo_pid=$!
else
  log "topic /${TARGET_NODE}/transition_event not visible after ${TOPIC_WAIT_TIMEOUT}s"
fi

run_cmd "lifecycle set /${TARGET_NODE} configure" ros2 lifecycle set "/${TARGET_NODE}" configure
assert_state_not "Unconfigured" "lifecycle get /${TARGET_NODE} (post-configure)"
sleep 1

run_cmd "lifecycle set /${TARGET_NODE} activate" ros2 lifecycle set "/${TARGET_NODE}" activate
assert_state_not "Inactive" "lifecycle get /${TARGET_NODE} (post-activate)"

wait_for_state "Active" "$STATE_WAIT_TIMEOUT"

if [[ -n "$echo_pid" ]]; then
  wait "$echo_pid" || true
fi
