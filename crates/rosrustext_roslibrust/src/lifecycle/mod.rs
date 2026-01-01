//! rosrustext_roslibrust::lifecycle
//!
//! Wrapper-side lifecycle API façade.
//!
//! Exposes application-facing lifecycle primitives while keeping core semantics in rosrustext_core.

/// Re-export core lifecycle types
pub use rosrustext_core::lifecycle::{ActivationGate, CallbackResult, LifecycleCallbacks};

// ROS lifecycle IDs/mapping (wrapper side).
mod ros;
pub use ros::{
    goal_state_for_transition, ros_ids, ros_state_id, ros_state_ids, ros_transition_id,
    shutdown_ros_id_for_state, state_from_ros_id, transition_from_ros_id,
};

// Lifecycle publisher module.
mod managed_publisher;
pub use managed_publisher::{ManagedPublisher, PublishLike};
#[cfg(feature = "roslibrust")]
pub use crate::transport::roslibrust::publisher::RosbridgePublisher;

// Lifecycle timer module.
mod managed_timer;
pub use managed_timer::ManagedInterval;

// DTOs module for lifecycle state transitions.
pub(crate) mod dtos;

// Lifecycle transition event stream (ROS-agnostic /transition_event equivalent).
mod events;
pub use events::TransitionEvent;

// Utility functions for lifecycle management.
mod util;
pub use util::run_if_active;

// Lifecycle node module.
mod node;
pub use node::LifecycleNode;
