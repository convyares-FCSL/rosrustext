# rosrustext_rosrs

An extension layer for **rclrs** that targets **ROS 2 tool parity**
(ROS-facing behavior) and **user parity** (developer ergonomics),
without becoming a new client library.

> Note on naming: this crate name predates the final naming convention.
> “rosrs” here refers to the native **rclrs-based** transport.

This crate **builds on rclrs**. It does not replace it.

---

## Scope and intent

`rosrustext_rosrs` exists to close the gap between:

* what Rust ROS nodes can *compile and run*, and
* what ROS 2 tools and managers *expect to observe*.

Parity is judged by **observable ROS behavior**
(`ros2 lifecycle`, `ros2 param`, Nav2, Python/C++ managers),
not by API similarity to `rclcpp`.

---

## Extension areas and status

| Area       | Tool parity | User parity |
|------------|-------------|-------------|
| Lifecycle  | Implemented (services, events, bond, gating) | Partial (gaps documented) |
| Parameters | Implemented (services, events) | Partial (watcher; no set-time validation hook) |
| Actions    | Not implemented | Not implemented |
| Executor   | Not implemented | Not implemented |

---

## Lifecycle parity (implemented)

* Lifecycle services:
  * `change_state`
  * `get_state`
  * `get_available_states`
  * `get_available_transitions`
* `transition_event` publisher  
  (one event per accepted attempt, published after completion)
* Optional `/bond` heartbeats for Nav2 compatibility (feature `bond`)
* Lifecycle-gated publishers and timers:
  * silent suppression while inactive
  * no DDS entity enable/disable tricks

Detailed tool vs user parity:
`docs/adapters/ros2rust/lifecycle/parity.md`

---

## Parameters parity (implemented baseline)

* Parameter services:
  * `get_parameters`
  * `get_parameter_types`
  * `list_parameters`
  * `describe_parameters`
  * `set_parameters`
  * `set_parameters_atomically`
* `parameter_events` publisher (emitted on successful updates only)
* `ParameterWatcher` helper for event-driven updates (no polling)

Known limitation:
* No user-defined **set-time validation hook**  
  (blocked by current `rclrs` surface; validation is observable post-apply)

Detailed tool vs user parity:
`docs/adapters/ros2rust/parameters/parity.md`

---

## Compatibility / parity notes

* Parameter support uses `rclrs::vendor::rcl_interfaces`
  for parameter services and `ParameterEvent`.
* This couples the crate to:
  * `rclrs` 0.6.x
  * `rosidl_runtime_rs` 0.5.x (transitively)
* Versions are pinned deliberately to preserve ROS tool compatibility.

This dependency is an implementation detail and will be updated as upstream evolves.

---

## Known differences vs rclcpp

* `change_state` returns `success=true` once a transition is **accepted**.
  Final outcome is reflected via:
  * `get_state`
  * `transition_event`

This matches ROS 2 service contracts and is compatible with tooling.

---

## Publish outcome visibility

When you need to observe lifecycle gating explicitly, use
`publish_with_outcome()`.

```rust
use rosrustext_rosrs::lifecycle::{PublishOutcome, ManagedPublisher};

match publisher.publish_with_outcome(msg)? {
    PublishOutcome::Published => {}
    PublishOutcome::SuppressedInactive => {
        // Message was gated due to inactive lifecycle state.
    }
}
````

`publish()` remains silent and returns `Ok(())` either way.

---

## Lifecycle user-parity gaps (current)

* Callbacks must be provided at construction time.

  * No set/replace callbacks API yet.
  * This is tracked and documented in parity docs.

---

## Other extensions (not yet implemented)

* Actions parity
* Executor extensions

These are planned and tracked under the project-level specs.

---

## Build notes (ROS installed)

This crate is published on crates.io, but **building requires a ROS 2
workspace** to provide generated message crates.

This is standard for `rclrs`-based projects.

```bash
cargo build \
  --manifest-path crates/rosrustext_rosrs/Cargo.toml \
  --features ros2,bond
```

* Docs build on docs.rs without ROS (`docsrs` + `rclrs/use_ros_shim`)
* Runtime builds/tests require ROS 2 Jazzy with the environment sourced

---

## Feature flags

* `ros2`
  Enables ROS-facing tooling parity (lifecycle + parameters)
* `bond`
  Enables `/bond` heartbeats for Nav2 lifecycle manager compatibility
  (requires `ros2`)
* `lifecycle_msgs`
  Exposes lifecycle message/service types explicitly
* `docsrs`
  Docs-only shim for ROS-free builds (not intended for users)

---

## Minimal lifecycle usage

Recommended constructor:
`LifecycleNode::create_with_callbacks`.

```rust
use rclrs::{Context, CreateBasicExecutor, SpinOptions};
use rosrustext_rosrs::lifecycle::{
    CallbackResult, LifecycleCallbacksWithNode, LifecycleNode
};
use rosrustext_rosrs::State;

struct Callbacks;

impl LifecycleCallbacksWithNode for Callbacks {
    fn on_configure(&mut self, node: &LifecycleNode, _: &State) -> CallbackResult {
        let _ = node.name();
        CallbackResult::Success
    }
    fn on_activate(&mut self, _: &LifecycleNode, _: &State) -> CallbackResult { CallbackResult::Success }
    fn on_deactivate(&mut self, _: &LifecycleNode, _: &State) -> CallbackResult { CallbackResult::Success }
    fn on_cleanup(&mut self, _: &LifecycleNode, _: &State) -> CallbackResult { CallbackResult::Success }
    fn on_shutdown(&mut self, _: &LifecycleNode, _: &State) -> CallbackResult { CallbackResult::Success }
    fn on_error(&mut self, _: &LifecycleNode, _: &State) -> CallbackResult { CallbackResult::Success }
}

fn main() -> rosrustext_rosrs::Result<()> {
    let context = Context::default();
    let mut executor = context.create_basic_executor();

    let _lifecycle =
        LifecycleNode::create_with_callbacks(&executor, "lifecycle_demo", Box::new(Callbacks))?;
    executor.spin(SpinOptions::default());
    Ok(())
}
```

---

## Builder APIs

### Managed (lifecycle-gated)

```rust
use rclrs::{Context, CreateBasicExecutor};
use rosrustext_rosrs::lifecycle::LifecycleNode;
use rosrustext_rosrs::lifecycle_msgs::msg::State;

fn main() -> rosrustext_rosrs::Result<()> {
    let context = Context::default();
    let executor = context.create_basic_executor();
    let lifecycle = LifecycleNode::create(&executor, "demo")?;

    let _pub = lifecycle.publisher::<State>("state").create()?;
    let _timer = lifecycle
        .timer_repeating(std::time::Duration::from_millis(100))
        .callback(|| {})
        .create()?;
    Ok(())
}
```

### Raw (non-managed)

```rust
use rclrs::{Context, CreateBasicExecutor};
use rosrustext_rosrs::NodeBuilderExt;
use rosrustext_rosrs::lifecycle_msgs::msg::State;

fn main() -> rosrustext_rosrs::Result<()> {
    let context = Context::default();
    let executor = context.create_basic_executor();
    let node = executor.create_node("demo")?;

    let _pub = node.publisher::<State>("state").create()?;
    let _timer = node
        .timer_repeating(std::time::Duration::from_millis(100))
        .callback(|| {})
        .create()?;
    Ok(())
}
```

---

## Summary

`rosrustext_rosrs` extends **rclrs** to make Rust nodes behave predictably
under real ROS 2 tooling.

It prioritizes:

* Observable parity
* Deterministic lifecycle behavior
* Explicit limitations
* Compatibility with existing ROS ecosystems

This crate is one part of the broader `rosrustext` project.
