#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
ROS2_WS_ROOT="${ROS2_WS_ROOT:-/home/ecm/ros2_rust_ws/ros2_ws}"
ROSBRIDGE_NODE_NAME="${ROSBRIDGE_NODE_NAME:-rosbridge_websocket}"
ROSBRIDGE_NODE_NAME="${ROSBRIDGE_NODE_NAME#/}"
ROSBRIDGE_LOG_LEVEL="${ROSBRIDGE_LOG_LEVEL:-warn}"
BRIDGE_URL="${BRIDGE_URL:-ws://localhost:9090}"
BRIDGE_PORT="${BRIDGE_PORT:-}"
ROSRUSTEXT_INTERFACES_PATH="${ROSRUSTEXT_INTERFACES_PATH:-$ROOT_DIR/crates/rosrustext_roslibrust/tools/rosrustext_lifecycle_proxy/interfaces}"

if [[ "$BRIDGE_URL" =~ ^ws://[^:/]+:([0-9]+)$ ]]; then
  if [[ -z "$BRIDGE_PORT" ]]; then
    BRIDGE_PORT="${BASH_REMATCH[1]}"
  fi
else
  echo "BRIDGE_URL must be ws://host:port (got: $BRIDGE_URL)" >&2
  exit 1
fi

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

if [[ ! -d "$ROS2_WS_ROOT" ]]; then
  echo "ROS2 workspace not found: $ROS2_WS_ROOT" >&2
  exit 1
fi

# Source ROS to get colcon/ros2.
set +u
source /opt/ros/jazzy/setup.bash
set -u

filter_prefix_path() {
  local var_name="$1"
  local value="${!var_name-}"
  [[ -z "$value" ]] && return 0
  local filtered=()
  local IFS=':'
  read -r -a parts <<<"$value"
  for part in "${parts[@]}"; do
    if [[ "$part" == *"rosidl_generator_rs"* ]]; then
      if [[ ! -f "$part/share/rosidl_generator_rs/cmake/register_rs.cmake" ]]; then
        continue
      fi
    fi
    filtered+=("$part")
  done
  local joined
  joined=$(IFS=':'; echo "${filtered[*]}")
  printf -v "$var_name" '%s' "$joined"
  export "$var_name"
}

if [[ "${SKIP_BUILD:-0}" != "1" ]]; then
  filter_prefix_path AMENT_PREFIX_PATH
  filter_prefix_path CMAKE_PREFIX_PATH
  if [[ -n "$ROSRUSTEXT_INTERFACES_PATH" ]]; then
    build_dir="$ROS2_WS_ROOT/build/rosrustext_interfaces"
    if [[ -f "$build_dir/CMakeCache.txt" ]]; then
      current_source=$(rg -n "^CMAKE_HOME_DIRECTORY:INTERNAL=" "$build_dir/CMakeCache.txt" \
        | head -n 1 | cut -d= -f2-)
      desired_source=$(readlink -f "$ROSRUSTEXT_INTERFACES_PATH" || true)
      if [[ -n "$current_source" && -n "$desired_source" && "$current_source" != "$desired_source" ]]; then
        rm -rf "$build_dir"
      fi
    fi
  fi
  packages=(hyfleet_interfaces)
  base_paths=("$ROS2_WS_ROOT/src")
  if [[ -d "$ROSRUSTEXT_INTERFACES_PATH" ]]; then
    packages+=(rosrustext_interfaces)
    base_paths+=("$ROSRUSTEXT_INTERFACES_PATH")
  fi
  (cd "$ROS2_WS_ROOT" && colcon build --packages-select "${packages[@]}" --base-paths "${base_paths[@]}")
fi

set +u
source "$ROS2_WS_ROOT/install/setup.bash"
set -u

if port_in_use "$BRIDGE_PORT"; then
  echo "Port ${BRIDGE_PORT} already in use; stop existing rosbridge or set BRIDGE_URL to a free port." >&2
  exit 1
fi

exec ros2 run rosbridge_server rosbridge_websocket \
  --port "$BRIDGE_PORT" \
  --ros-args -r "__node:=${ROSBRIDGE_NODE_NAME}" --log-level "$ROSBRIDGE_LOG_LEVEL"
