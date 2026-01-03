//! rosrustext_ros2_rust
//!
//! ROS-facing adapter layer built on top of `rclrs` (ros2_rust).
//! Provides lifecycle-compatible surfaces intended to match ROS 2 tooling,
//! while keeping core semantics in `rosrustext_core`.
//!
//! Design rules:
//! - Application owns executor + spinning.
//! - No hidden threads inside the adapter.
//! - Core lifecycle semantics live in `rosrustext_core`.
//! - ROS/rclrs wiring lives here.

pub mod error;
pub mod lifecycle;

// Adapter error type
pub use error::{Error, Result};

// Re-export core types that wrapper users will commonly need
pub use rosrustext_core::error::CoreError;
pub use rosrustext_core::lifecycle::{CallbackResult, State, Transition};
