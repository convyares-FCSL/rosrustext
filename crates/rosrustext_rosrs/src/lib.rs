//! # rosrustext_rosrs
//!
//! This crate provides ROS 2 lifecycle + parameters parity for `rclrs`.
//!
//! - Start here: API + parity guide (in-repo): `docs/API.md`
//! - Specs: `docs/spec/lifecycle.md`, `docs/spec/parameters.md`
//! - Parity tables: `docs/parity/*`
//!
//! ## Feature matrix
//! See README for `ros2`, `bond`, `transition_graph`, and docs-only `docsrs`.
//!
//! (Keep this page short; it’s a signpost, not a novel.)

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
