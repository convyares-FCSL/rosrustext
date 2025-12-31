//! rclrust_wrapper
//!
//! ROS-facing adapter layer built on top of roslibrust.
//! Provides lifecycle + action surfaces compatible with ROS 2 tooling,
//! while keeping core semantics in `rclrust_core`.

// Public modules
pub mod error;

pub mod logging;
pub mod lifecycle;
pub mod transport;

// Re-export core types that wrapper users will commonly need
pub use rclrust_core::lifecycle::{CallbackResult, State, Transition};
pub use rclrust_core::error::{CoreError, Result};
