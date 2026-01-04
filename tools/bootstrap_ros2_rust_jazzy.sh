#!/usr/bin/env bash
set -euo pipefail

# Where to create the ros2_rust vendor workspace (default: ./ros2_rust_ws next to this repo)
ROS2_RUST_WS="${ROS2_RUST_WS:-$(pwd)/ros2_rust_ws}"

# Upstream source of truth
ROS2_RUST_REPO_URL="https://github.com/ros2-rust/ros2_rust.git"
ROS2_RUST_REF="${ROS2_RUST_REF:-main}"
# set this explicitly when you want:
# ROS2_RUST_REF="v0.6.0"

mkdir -p "$ROS2_RUST_WS"
cd "$ROS2_RUST_WS"

if [ ! -d ".git" ]; then
  git init
  git remote add origin "$ROS2_RUST_REPO_URL"
fi

git fetch --depth 1 --tags origin "$ROS2_RUST_REF"
git checkout -f "$ROS2_RUST_REF"

# Pull the pinned Jazzy repos list from upstream and import
vcs import < ros2_rust_jazzy.repos

# Build + install the interface crates (uses upstream script)
mkdir -p "$ROS2_RUST_WS/install"

set +u
source /opt/ros/jazzy/setup.bash
set -u

colcon build --merge-install --install-base "$ROS2_RUST_WS/install"

python3 rclrs/vendor_interfaces.py "$ROS2_RUST_WS/install"

echo ""
echo "Done."
echo "Install is at: $ROS2_RUST_WS/install"
echo "Next: point your Cargo [patch.crates-io] at that install tree."
