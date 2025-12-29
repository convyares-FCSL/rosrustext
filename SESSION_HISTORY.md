# rclrust – Session History

## Session 1 – Library bootstrap
- Decided to host rclrust outside ros2_ws as a vendor-style library
- Chose Cargo workspace layout:
  - rclrust_core (pure logic)
  - rclrust_wrapper (ROS adapter)
- Agreed to avoid rclrs; use roslibrust in wrapper only
- Established error conventions (CoreError)

## Session 2 – Lifecycle foundations
- Implemented minimal lifecycle state machine
- Expanded state enum to include ROS2 transition states
- Confirmed intent to match ROS2 lifecycle semantics fully in wrapper
- Clarified that transition states (Configuring, ShuttingDown, ErrorProcessing)
  are not primary states but must be representable

Guiding principle:
Model lifecycle as an explicit state machine first, then map to ROS,
never the other way around.
