#!/usr/bin/env bash
set -euo pipefail

TARGET_NODE="${TARGET_NODE:-hyfleet_ring_roslibrust}"
LIFECYCLE_TIMEOUT="${LIFECYCLE_TIMEOUT:-5}"
SERVICE_WAIT_TIMEOUT="${SERVICE_WAIT_TIMEOUT:-6}"
SERVICE_READY_DELAY="${SERVICE_READY_DELAY:-1}"
TOPIC_WAIT_TIMEOUT="${TOPIC_WAIT_TIMEOUT:-6}"

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
sleep 1

run_cmd "lifecycle set /${TARGET_NODE} activate" ros2 lifecycle set "/${TARGET_NODE}" activate

run_cmd "lifecycle get /${TARGET_NODE}" ros2 lifecycle get "/${TARGET_NODE}"

if [[ -n "$echo_pid" ]]; then
  wait "$echo_pid" || true
fi
