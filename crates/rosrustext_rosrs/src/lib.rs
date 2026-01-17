//! # rosrustext_rosrs
//!
//! This crate provides ROS 2 lifecycle + parameters parity for `rclrs`.
//!
//! - Start here: API + parity guide (in-repo): `docs/API.md`
//! - Specs: `docs/spec/lifecycle.md`, `docs/spec/parameters.md`
//! - Parity tables: `docs/parity/*`
//!
//! ## Feature matrix
//! See README for `ros2`, `bond`, and docs-only `docsrs`.
//!
//! (Keep this page short; it’s a signpost, not a novel.)
//!
//! ## Builder quick start
//! Managed (lifecycle-gated):
//! ```rust,no_run
//! use rclrs::{Context, CreateBasicExecutor};
//! use rosrustext_rosrs::lifecycle::LifecycleNode;
//! use rosrustext_rosrs::lifecycle_msgs::msg::State;
//!
//! # fn main() -> rosrustext_rosrs::Result<()> {
//! let context = Context::default();
//! let executor = context.create_basic_executor();
//! let lifecycle = LifecycleNode::create(&executor, "demo")?;
//! let _pub = lifecycle.publisher::<State>("state").create()?;
//! # Ok(()) }
//! ```
//!
//! Raw (non-managed):
//! ```rust,no_run
//! use rclrs::{Context, CreateBasicExecutor};
//! use rosrustext_rosrs::NodeBuilderExt;
//! use rosrustext_rosrs::lifecycle_msgs::msg::State;
//!
//! # fn main() -> rosrustext_rosrs::Result<()> {
//! let context = Context::default();
//! let executor = context.create_basic_executor();
//! let node = executor.create_node("demo")?;
//! let _pub = node.publisher::<State>("state").create()?;
//! # Ok(()) }
//! ```

#[cfg(feature = "ros2")]
pub mod builders;
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

// Node builder extension APIs (raw rclrs resources)
#[cfg(feature = "ros2")]
pub use builders::NodeBuilderExt;
#[cfg(feature = "ros2")]
pub use builders::PublisherBuilder;
#[cfg(feature = "ros2")]
pub use builders::SubscriptionBuilder;
#[cfg(feature = "ros2")]
pub use builders::TimerBuilder;

// Message compatibility re-exports
pub mod lifecycle_msgs {
    pub use rosrustext_msgs::lifecycle_msgs::*;
}
#[cfg(feature = "bond")]
pub mod bond {
    pub use rosrustext_msgs::bond::*;
}
