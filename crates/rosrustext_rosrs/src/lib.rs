//! rosrustext_rosrs
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
#[cfg(feature = "ros2")]
pub mod lifecycle;
#[cfg(feature = "ros2")]
pub mod parameters;

// Adapter error type
pub use error::{Error, Result};

// Re-export core types that wrapper users will commonly need
pub use rosrustext_core::error::CoreError;
pub use rosrustext_core::lifecycle::{CallbackResult, State, Transition};

// Message compatibility re-exports
pub mod lifecycle_msgs {
    pub use rosrustext_msgs::lifecycle_msgs::*;
}
#[cfg(feature = "bond")]
pub mod bond {
    pub use rosrustext_msgs::bond::*;
}
#[cfg(feature = "transition_graph")]
pub mod rosrustext_interfaces {
    pub use rosrustext_msgs::rosrustext_interfaces::*;
}
