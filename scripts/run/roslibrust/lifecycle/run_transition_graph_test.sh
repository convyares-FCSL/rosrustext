#!/usr/bin/env bash
set -euo pipefail

TARGET_NODE="${TARGET_NODE:-hyfleet_ring_roslibrust}"
LIFECYCLE_TIMEOUT="${LIFECYCLE_TIMEOUT:-10}"
SERVICE_WAIT_TIMEOUT="${SERVICE_WAIT_TIMEOUT:-6}"
SERVICE_READY_DELAY="${SERVICE_READY_DELAY:-1}"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
ROS2_WS_ROOT="${ROS2_WS_ROOT:-/home/ecm/ros2_rust_ws/ros2_ws}"
PYTHON_BIN="${PYTHON_BIN:-}"

log() {
  echo "[$(date +%H:%M:%S)] $*"
}

if [[ -z "$PYTHON_BIN" ]]; then
  if command -v python3 >/dev/null 2>&1; then
    PYTHON_BIN="python3"
  elif command -v python >/dev/null 2>&1; then
    PYTHON_BIN="python"
  else
    echo "python3 or python is required for transition graph validation." >&2
    exit 1
  fi
fi

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

set +u
source /opt/ros/jazzy/setup.bash
if [[ -f "$ROS2_WS_ROOT/install/setup.bash" ]]; then
  source "$ROS2_WS_ROOT/install/setup.bash"
elif [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
  source "$ROOT_DIR/install/setup.bash"
fi

# If the overlay lifecycle_msgs .msg is missing (broken symlink), re-prepend
# the system install so ros2cli uses the .msg instead of .idl.
if [[ -f "$ROS2_WS_ROOT/install/lifecycle_msgs/share/lifecycle_msgs/msg/State.msg" ]]; then
  if ! [[ -r "$ROS2_WS_ROOT/install/lifecycle_msgs/share/lifecycle_msgs/msg/State.msg" ]]; then
    log "overlay lifecycle_msgs msg unreadable; preferring /opt/ros/jazzy for ros2cli"
    source /opt/ros/jazzy/setup.bash
  fi
else
  log "overlay lifecycle_msgs msg missing; preferring /opt/ros/jazzy for ros2cli"
  source /opt/ros/jazzy/setup.bash
fi
set -u

if ! wait_for_service "/${TARGET_NODE}/get_transition_graph" "$SERVICE_WAIT_TIMEOUT"; then
  log "service /${TARGET_NODE}/get_transition_graph not visible after ${SERVICE_WAIT_TIMEOUT}s"
  exit 1
fi
sleep "$SERVICE_READY_DELAY"

log "calling get_transition_graph"
set +e
output=$(timeout "${LIFECYCLE_TIMEOUT}s" \
  ros2 service call "/${TARGET_NODE}/get_transition_graph" \
  rosrustext_interfaces/srv/GetTransitionGraph "{}" 2>&1)
status=$?
set -e

if [[ $status -eq 124 ]]; then
  log "get_transition_graph timed out after ${LIFECYCLE_TIMEOUT}s"
  [[ -n "$output" ]] && echo "$output"
  exit 1
fi
if [[ $status -ne 0 ]]; then
  log "get_transition_graph failed (exit $status)"
  [[ -n "$output" ]] && echo "$output"
  exit 1
fi

OUTPUT="$output" "$PYTHON_BIN" - <<'PY'
import os
import re
import sys

text = os.environ.get("OUTPUT", "")
response_match = re.search(r"response:\s*(.*)", text, re.S)
if response_match:
    text = response_match.group(1).strip()

m = re.search(r"states:\s*(.*?)\n\s*transitions:", text, re.S)
if m:
    states_block = m.group(1)
    state_labels = set(re.findall(r"label:\s*\"?([A-Za-z]+)\"?", states_block))
else:
    m = re.search(r"states=\[(.*?)\],\s*transitions=", text, re.S)
    if not m:
        print("missing states section")
        sys.exit(1)
    states_block = m.group(1)
    state_labels = set(re.findall(r"label='([^']+)'", states_block))
required_states = {
    "Unconfigured",
    "Inactive",
    "Active",
    "Finalized",
    "Configuring",
    "CleaningUp",
    "Activating",
    "Deactivating",
    "ShuttingDown",
    "ErrorProcessing",
}
missing_states = sorted(required_states - state_labels)
if missing_states:
    print(f"missing states: {', '.join(missing_states)}")
    sys.exit(1)

transitions = set()
blocks = re.split(r"\n\s*-\s*transition:", text)
if len(blocks) > 1:
    for block in blocks[1:]:
        label_match = re.search(r"label:\s*\"?([A-Za-z_]+)\"?", block)
        start_match = re.search(r"start_state:.*?label:\s*\"?([A-Za-z]+)\"?", block, re.S)
        goal_match = re.search(r"goal_state:.*?label:\s*\"?([A-Za-z]+)\"?", block, re.S)
        if not (label_match and start_match and goal_match):
            continue
        transitions.add((label_match.group(1), start_match.group(1), goal_match.group(1)))
else:
    for match in re.finditer(
        r"TransitionDescription\(transition=.*?label='([^']+)'.*?"
        r"start_state=.*?label='([^']+)'.*?"
        r"goal_state=.*?label='([^']+)'.*?\)",
        text,
        re.S,
    ):
        transitions.add((match.group(1), match.group(2), match.group(3)))

if not transitions:
    print("missing transitions")
    sys.exit(1)

expected = {
    ("configure", "Unconfigured", "Inactive"),
    ("shutdown", "Unconfigured", "Finalized"),
    ("activate", "Inactive", "Active"),
    ("cleanup", "Inactive", "Unconfigured"),
    ("shutdown", "Inactive", "Finalized"),
    ("deactivate", "Active", "Inactive"),
    ("shutdown", "Active", "Finalized"),
}

missing = expected - transitions
extra = transitions - expected
if missing:
    print("missing transitions:")
    for item in sorted(missing):
        print(f"  {item}")
    sys.exit(1)
if extra:
    print("unexpected transitions:")
    for item in sorted(extra):
        print(f"  {item}")
    sys.exit(1)

print("transition graph ok")
PY
