//! rclrust_wrapper::lifecycle
//!
//! Wrapper-side lifecycle API façade.
//!
//! Exposes application-facing lifecycle primitives while keeping core semantics in rclrust_core.

/// Re-export core lifecycle types
pub use rclrust_core::lifecycle::{ActivationGate, CallbackResult, LifecycleCallbacks};

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
// pub(crate) use dtos::{change_state, get_available_transitions, get_state};

// Utility functions for lifecycle management.
mod util;
pub use util::run_if_active;

// Lifecycle node module.
mod node;
pub use node::LifecycleNode;
