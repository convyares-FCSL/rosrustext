# rosrustext_rosrs

rclrs extension layer that targets tool parity (ROS-facing behavior) and user
parity (developer ergonomics) for ROS 2 features, centered on the ROS-facing
adapter and application-facing APIs.

## Extension areas and status

| Area | Tool parity | User parity |
| ---- | ---------- | ---------- |
| Lifecycle | Implemented (services/events/bond/gating) | Partial (gaps documented) |
| Parameters | Not implemented | Not implemented |
| Actions | Not implemented | Not implemented |
| Executor | Not implemented | Not implemented |

## Lifecycle parity (implemented)

* Lifecycle services: `change_state`, `get_state`, `get_available_states`,
  `get_available_transitions`
* `transition_event` publisher (one per accepted attempt, after completion)
* Optional `get_transition_graph` service (feature `transition_graph`)
* Optional `/bond` heartbeats for Nav2 compatibility (feature `bond`)
* ManagedPublisher/ManagedTimer gating while inactive (local suppression)

See `docs/adapters/ros2rust/lifecycle/parity.md` for tool vs user parity details.

## Known differences vs rclcpp

* `change_state` returns `success=true` once a transition is accepted; callback
  outcome is reflected in the final state and `transition_event`.

## Lifecycle user parity gaps (current)

* No set/replace callbacks API; `create`/`try_new` still default to no-op callbacks
* Legacy `LifecycleCallbacks` lacks node/state context (use `LifecycleCallbacksWithNode`)
* A publish suppression signal from `ManagedPublisher::publish`

## Non-lifecycle extensions (not implemented)

* Parameters parity
* Actions parity
* Executor extensions

## Build (ROS installed, not on crates.io)

This crate requires ROS 2 to be installed and is not published on crates.io.

```bash
cargo build \
  --manifest-path crates/rosrustext_rosrs/Cargo.toml \
  --features ros2,bond,transition_graph
```

## Feature flags

* `ros2`: enable ROS-facing lifecycle services and `transition_event`
* `bond`: publish `/bond` heartbeats for Nav2 lifecycle manager compatibility
  (requires `ros2`)
* `transition_graph`: enable `GetTransitionGraph` via `rosrustext_interfaces`
  (requires `ros2`)

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
