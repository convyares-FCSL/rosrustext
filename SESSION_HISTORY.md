# rclrust – Session History

## Session 1 – Library bootstrap
- Established vendor-style Cargo workspace
- Split into:
  - rclrust_core (pure logic)
  - rclrust_wrapper (ROS adapter)
- Chose roslibrust over rclrs
- Defined CoreError model

## Session 2 – Lifecycle semantics
- Implemented full ROS2-style lifecycle state machine
- Added explicit transition states
- Added begin → callback → finish model
- Added ErrorProcessing semantics
- Added deterministic unit tests

## Session 3 – Wrapper abstraction
- Implemented LifecycleNode
- Added activation gating
- Implemented ChangeState / GetState handlers
- Mapped ROS transition IDs
- Ensured parity with lifecycle manager expectations

## Session 4 – Managed resources
- Added activation-gated publishers
- Added activation-gated timers (tokio)
- Isolated roslibrust transport behind feature flags
- Validated behavior with real rosbridge nodes

Guiding principle:
**Model lifecycle truth once, then adapt to ROS.
Never let transport shape the semantics.**
