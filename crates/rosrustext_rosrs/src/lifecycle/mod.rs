//! ROS 2 lifecycle (tooling-parity wrapper for `rclrs`).
//!
//! This module provides a ROS-facing lifecycle surface intended to behave like a
//! canonical ROS 2 managed node when observed via tools (e.g. `ros2 lifecycle`)
//! while keeping the transport-agnostic rules in `rosrustext_core`.
//!
//! The main entry point is [`LifecycleNode`], a thin wrapper around an
//! `Arc<rclrs::Node>` that:
//! - exposes the standard lifecycle services (feature `ros2`)
//! - publishes `lifecycle_msgs/msg/TransitionEvent`
//! - (optionally) publishes `/bond` heartbeats for Nav2 compatibility (feature `bond`)
//! - provides lifecycle-gated resource helpers ([`ManagedPublisher`], [`ManagedTimer`])
//!
//! # Purpose
//! - Match ROS 2 observable lifecycle behavior (services + events) with `rclrs`.
//! - Provide an explicit, hard-to-bypass “managed resources” surface for publishing
//!   and repeating timers while inactive.
//!
//! # Design overview (wrapper, not subclass)
//! `LifecycleNode` is a wrapper type and intentionally does **not**
//! `Deref<Target = rclrs::Node>`. In Rust, the practical equivalent of “subclassing”
//! an `rclrs::Node` would be exposing the inner node ergonomically; this module
//! avoids that so that lifecycle gating is the default.
//!
//! You can still access the underlying node via [`LifecycleNode::node_arc`], but
//! doing so is an explicit escape hatch and will bypass managed-resource gating.
//!
//! # Lifecycle semantics (configure/activate/…)
//! - Stable states: `Unconfigured`, `Inactive`, `Active`, `Finalized`.
//! - Transitions are initiated via the `change_state` service using ROS transition IDs.
//! - Busy/invalid transition requests are rejected deterministically (`success=false`)
//!   without mutating state.
//! - `get_state` reports the stable state only (intermediate transition states are
//!   not reported).
//! - After an accepted transition completes, one `transition_event` is published.
//!
//! Error handling:
//! - If a transition callback returns [`CallbackResult::Error`], the node enters
//!   `ErrorProcessing` and then calls `on_error` to decide recovery.
//!
//! # Managed resources behavior
//! Managed resources are gated by an internal [`ActivationGate`]:
//! - [`ManagedPublisher`] drops publishes while inactive. Use
//!   [`ManagedPublisher::publish_with_outcome`] when you need to distinguish
//!   “published” vs “suppressed”.
//! - [`LifecycleNode::create_timer_repeating_gated`] installs a repeating timer
//!   that *skips* its callback while inactive. The underlying timer still ticks.
//!
//! Not gated:
//! - services, subscriptions, and clients created via
//!   [`LifecycleNode::create_service`], [`LifecycleNode::create_subscription`],
//!   [`LifecycleNode::create_client`] are always active.
//!
//! # Threading model notes
//! - Lifecycle services run on the `rclrs` executor thread.
//! - Default behavior is synchronous completion: with
//!   `ROSRUSTEXT_RCLRS_CHANGE_STATE_DELAY_MS=0` (default), the transition callback
//!   runs inline in the `change_state` handler and the stable state is updated
//!   before the service response is returned.
//! - If `ROSRUSTEXT_RCLRS_CHANGE_STATE_DELAY_MS>0`, the transition callback is run
//!   on a spawned OS thread and completion is applied by a small internal polling
//!   timer (10ms) on the executor thread.
//!
//! # Tool parity notes
//! - `ros2 lifecycle` CLI interoperates via `get_state`, `change_state`,
//!   `get_available_states`, and `get_available_transitions`.
//! - Feature `bond` publishes `/bond` with Nav2-required QoS and publishes an
//!   *immediate* `active=true` message on the `inactive -> active` edge.
//! - Feature `transition_graph` exposes a non-standard `get_transition_graph`
//!   introspection service.
//!
//! # Known limitations / intentional differences
//! - Only publishers and repeating timers created via this module are lifecycle-gated.
//! - Subscriptions are not lifecycle-managed (they are always active).
//! - Environment variables under the `ROSRUSTEXT_RCLRS_*` prefix exist primarily
//!   for testing and can change transition timing/results when set.
//!
//! # Links
//! - [Lifecycle spec](https://github.com/convyares-FCSL/rosrustext_fcsl/blob/main/docs/spec/lifecycle.md)
//! - [Lifecycle parity notes](https://github.com/convyares-FCSL/rosrustext_fcsl/blob/main/parity.md)

/// Transport-agnostic activation gate used to implement managed-resource gating.
pub use rosrustext_core::lifecycle::ActivationGate;

/// Result returned from lifecycle transition callbacks.
pub use rosrustext_core::lifecycle::CallbackResult;

// Managed publisher (gated publish)
mod managed_publisher;
pub use managed_publisher::{ManagedPublisher, PublishOutcome};

// Managed timer (gated timer callback)
mod managed_timer;
pub use managed_timer::ManagedTimer;

// Bond agent (feature-gated)
#[cfg(feature = "bond")]
mod bond_agent;
#[cfg(feature = "bond")]
pub use bond_agent::BondAgent;

// Optional custom introspection service type (Jazzy compatibility)
#[cfg(feature = "transition_graph")]
pub use rosrustext_msgs::rosrustext_interfaces::srv::GetTransitionGraph;

// LifecycleNode (thin wrapper around Arc<rclrs::Node> + gate)
mod node;
pub use node::{LifecycleCallbacksWithNode, LifecycleNode};

// Internal helpers split out from node for readability
mod utils;
