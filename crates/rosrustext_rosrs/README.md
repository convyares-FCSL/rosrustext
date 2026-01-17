# rosrustext_rosrs

rclrs extension layer that targets tool parity (ROS-facing behavior) and user
parity (developer ergonomics) for ROS 2 features, centered on the ROS-facing
adapter and application-facing APIs.

## Extension areas and status

| Area | Tool parity | User parity |
| ---- | ---------- | ---------- |
| Lifecycle | Implemented (services/events/bond/gating) | Partial (gaps documented) |
| Parameters | Implemented (services/events) | Partial (watcher; no set-time validation hook) |
| Actions | Not implemented | Not implemented |
| Executor | Not implemented | Not implemented |

## Lifecycle parity (implemented)

* Lifecycle services: `change_state`, `get_state`, `get_available_states`,
  `get_available_transitions`
* `transition_event` publisher (one per accepted attempt, after completion)
* Optional `/bond` heartbeats for Nav2 compatibility (feature `bond`)
* ManagedPublisher/ManagedTimer gating while inactive (local suppression)

See `docs/adapters/ros2rust/lifecycle/parity.md` for tool vs user parity details.

## Parameters parity (implemented baseline)

* Parameter services: `get_parameters`, `get_parameter_types`, `list_parameters`,
  `describe_parameters`, `set_parameters`, `set_parameters_atomically`
* `parameter_events` publisher (emitted on successful updates only)
* `ParameterWatcher` helper for event-driven updates without polling
* No set-time validation hook (updates are observable after apply)

See `docs/adapters/ros2rust/parameters/parity.md` for tool vs user parity details.

## Compatibility / Parity Notes

* Parameters parity uses `rclrs::vendor::rcl_interfaces` for `ParameterEvent`
  and parameter services.
* This couples the adapter to `rclrs` 0.6.x (and `rosidl_runtime_rs` 0.5.x
  transitively).
* This is an implementation detail dependency; if `rclrs` changes the vendored
  surface, we will pin/update accordingly.

## Known differences vs rclcpp

* `change_state` returns `success=true` once a transition is accepted; callback
  outcome is reflected in the final state and `transition_event`.

## Publish outcome visibility

Use `publish_with_outcome()` when you need to know whether a message was
suppressed due to lifecycle inactivity. `publish()` remains silent and returns
`Ok(())` either way.

```rust
use rosrustext_rosrs::lifecycle::{PublishOutcome, ManagedPublisher};

match publisher.publish_with_outcome(msg)? {
    PublishOutcome::Published => {}
    PublishOutcome::SuppressedInactive => {
        // Message was gated due to inactive lifecycle state.
    }
}
```

## Lifecycle user parity gaps (current)

* No set/replace callbacks API; `create`/`try_new` still default to no-op callbacks

## Other extensions (not implemented)

* Actions parity
* Executor extensions

## Build (ROS installed)

This crate is published on crates.io, but building requires ROS 2 and ROS message
crates available via a ROS workspace environment (Cargo patching is typical).

```bash
cargo build \
  --manifest-path crates/rosrustext_rosrs/Cargo.toml \
  --features ros2,bond
```

Docs build on docs.rs without ROS (feature `docsrs` + `rclrs/use_ros_shim`).
Runtime builds/tests require ROS 2 Jazzy with the environment sourced.

## Feature matrix

* `ros2`: enables ROS-facing tooling parity (parameter services/events and lifecycle services)
  and relies on `rclrs::vendor::rcl_interfaces`.
* `bond`: publishes `/bond` heartbeats for Nav2 lifecycle manager compatibility (requires `ros2`).
* `lifecycle_msgs`: exposes lifecycle message/service surfaces; no hidden deps.
* `docsrs`: docs.rs-only shim for ROS-free builds; not intended for users.

## Example commands (Jazzy)

```bash
ros2 lifecycle get /<node>
# Expect the current stable state (Unconfigured/Inactive/Active/Finalized)

ros2 lifecycle set /<node> configure
# success=true; a transition_event is published after completion

ros2 lifecycle set /<node> activate
# success=true; /bond heartbeats begin if `bond` is enabled
```

## Minimal usage

Recommended constructor: `LifecycleNode::create_with_callbacks`.

```rust
use rclrs::{Context, CreateBasicExecutor, SpinOptions};
use rosrustext_rosrs::lifecycle::{CallbackResult, LifecycleCallbacksWithNode, LifecycleNode};
use rosrustext_rosrs::State;

struct Callbacks;

impl LifecycleCallbacksWithNode for Callbacks {
    fn on_configure(&mut self, node: &LifecycleNode, _state: &State) -> CallbackResult {
        let _ = node.name();
        CallbackResult::Success
    }
    fn on_activate(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult { CallbackResult::Success }
    fn on_deactivate(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult { CallbackResult::Success }
    fn on_cleanup(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult { CallbackResult::Success }
    fn on_shutdown(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult { CallbackResult::Success }
    fn on_error(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult { CallbackResult::Success }
}

fn main() -> rosrustext_rosrs::Result<()> {
    let context = Context::default();
    let mut executor = context.create_basic_executor();

    let _lifecycle = LifecycleNode::create_with_callbacks(&executor, "lifecycle_demo", Box::new(Callbacks))?;
    executor.spin(SpinOptions::default());
    Ok(())
}
```

## Builder APIs (managed vs raw)

Managed (lifecycle-gated):

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

Raw (non-managed):

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
