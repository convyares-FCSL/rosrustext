# Changelog

## 0.4.1 - 2026-01-17

### Breaking
- `LifecycleNode::create_publisher`, `LifecycleNode::create_subscription`, and
  `LifecycleNode::create_timer_repeating_gated` were removed in favor of builders.

Migration:
```rust
// Old
// let pub_ = lifecycle.create_publisher::<T>("topic")?;
// let sub = lifecycle.create_subscription::<T, _>("topic", callback)?;
// let timer = lifecycle.create_timer_repeating_gated(period, callback)?;

// New
let pub_ = lifecycle.publisher::<T>("topic").create()?;
let sub = lifecycle.subscription::<T>("topic").callback(callback).create()?;
let timer = lifecycle.timer_repeating(period).callback(callback).create()?;
```

### Added
- Managed builder APIs for publishers, subscriptions, and timers with `rclrs` options parity.
- Raw node builder extension trait `NodeBuilderExt` with publisher/subscription/timer builders.

### Notes
- Lifecycle gating semantics are unchanged: publishers are gated at publish, timers at callback entry,
  and subscriptions remain ungated by default.
