//! Lifecycle event types (ROS-agnostic).
//!
//! This is the wrapper-side equivalent of ROS2 `/transition_event`.
//! Transport layers can subscribe and map to ROS messages (e.g. lifecycle_msgs/msg/TransitionEvent).

use rosrustext_core::lifecycle::State;

/// Emitted after a successful transition.
///
/// `transition_id` uses ROS transition id numbers (same ids you pass to ChangeState).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TransitionEvent {
    pub transition_id: u8,
    pub start_state: State,
    pub goal_state: State,
}
