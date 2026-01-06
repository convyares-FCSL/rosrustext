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

// Bond agent (feature-gated)
#[cfg(feature = "bond")]
mod bond_agent;
#[cfg(feature = "bond")]
pub use bond_agent::BondAgent;

// Optional custom introspection service type (Jazzy compatibility)
#[cfg(feature = "transition_graph")]
pub use rosrustext_interfaces::srv::GetTransitionGraph;

// LifecycleNode (thin wrapper around Arc<rclrs::Node> + gate)
mod node;
pub use node::LifecycleNode;

// Internal helpers split out from node for readability
mod utils;
