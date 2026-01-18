# roslibrust lifecycle tests

This folder contains end-to-end lifecycle tests for the roslibrust + rosbridge
stack, including a ROS 2 backend node and a lifecycle proxy.

## Run all tests

These tests require a ROS 2 workspace that includes the backend package and
`hyfleet_interfaces`. The script can be run from any working directory, but it
must be able to locate the rosrustext repo (it is repo-relative) and the ROS 2
workspace via `ROS2_WS_ROOT`.

Recommended (from your dev workspace):

```bash
ROS2_WS_ROOT=/home/ecm/fcsl/rosrustext_dev_ws \
  /home/ecm/fcsl/rosrustext/scripts/test/roslibrust/run_all_tests.sh
```

Alternative (from the rosrustext repo root):

```bash
ROS2_WS_ROOT=/home/ecm/fcsl/rosrustext_dev_ws \
  ./scripts/test/roslibrust/run_all_tests.sh
```

The script:
- sources `/opt/ros/jazzy/setup.bash`
- sources `$ROS2_WS_ROOT/install/setup.bash` if present
- runs the lifecycle tests in `scripts/test/roslibrust/lifecycle/`

## Required components

These tests start and stop the following processes automatically:
- rosbridge websocket server (`rosbridge_server`)
- backend node binary (default: `hyfleet_ring_roslibrust`)
- lifecycle proxy (`rosrustext_lifecycle_proxy`)
- ROS 2 tools used by tests (`ros2`, `nav2_lifecycle_manager`, `python3` + `rclpy`)

You must have a ROS 2 workspace that contains the backend package:

```
$ROS2_WS_ROOT/src/hyfleet_ring_roslibrust
```

The backend is launched via `scripts/run/roslibrust/lifecycle/run_backend.sh` and
uses:
- `HYFLEET_NODE_NAME` (defaults to `TARGET_NODE`)
- `ROSLIBRUST_BRIDGE_URL` (defaults to `BRIDGE_URL`)

## Environment variables

Common overrides:

- `ROS2_WS_ROOT`: ROS 2 workspace to source and build from.
- `TARGET_NODE`: node name under test (default: `hyfleet_ring_roslibrust`).
- `BRIDGE_URL`: rosbridge websocket URL (default: `ws://localhost:9090`).
- `ROSBRIDGE_NODE_NAME`: rosbridge node name (default: `TARGET_NODE`).
- `LOG_DIR`: log output directory (default: `logs/run_all`).

Process control (use with care):
- `SKIP_PROCESS_CHECK=1`: skip checks for already-running processes.
- `AUTO_KILL_ROSBRIDGE=0`: do not kill rosbridge on exit.
- `AUTO_KILL_BACKEND=0`: do not kill the backend on exit.

Timeouts:
- `STARTUP_TIMEOUT`, `STARTUP_DELAY`
- `CHANGE_STATE_TIMEOUT`, `GET_STATE_TIMEOUT`, `STATE_WAIT_TIMEOUT`
- `NAV2_WAIT_TIMEOUT`, `LIFECYCLE_TIMEOUT`

Rosbridge build control (in `run_rosbridge.sh`):
- `SKIP_BUILD=1` to skip `colcon build --packages-select hyfleet_interfaces`

## Logs

Logs are written to:

```
logs/run_all/
```

Key files:
- `rosbridge.log`
- `backend.log`
- `proxy.log`
- `nav2_manager.log`
- `transition_event.log`

If a test fails, check these logs first.
