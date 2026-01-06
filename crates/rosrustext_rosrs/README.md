# rosrustext_rosrs

Native `rclrs` adapter for `rosrustext_core` lifecycle semantics.

**Dev workspace only:** this crate depends on ROS 2 message crates generated in a
colcon workspace and is not publishable on crates.io.

## Build (dev_ws)

```bash
cargo build --manifest-path crates/rosrustext_rosrs/Cargo.toml --features ros2,bond,transition_graph
```

## System tests (dev_ws)

```bash
./scripts/test/ros2_rust/run_all_tests.sh
```
