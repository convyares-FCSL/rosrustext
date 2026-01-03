//! rosrustext_ros2_rust::lifecycle
//!
//! Adapter-side lifecycle API façade for rclrs.
//!
//! Slice-1 focuses on a thin gated node/publisher/timer surface.
//! Lifecycle services + transition_event + bond come in later slices.

// Re-export core lifecycle types (transport-agnostic)
pub use rosrustext_core::lifecycle::{ActivationGate, CallbackResult, LifecycleCallbacks};

// Managed publisher (gated publish)
mod managed_publisher;
pub use managed_publisher::ManagedPublisher;

// Managed timer (gated timer callback)
mod managed_timer;
pub use managed_timer::ManagedTimer;

// LifecycleNode (thin wrapper around Arc<rclrs::Node> + gate)
mod node;
pub use node::LifecycleNode;
