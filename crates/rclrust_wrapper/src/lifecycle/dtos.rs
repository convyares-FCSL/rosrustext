//! Minimal lifecycle service DTOs.
//!
//! These are wrapper-internal request/response types.
//! The transport layer maps real ROS messages into these.

/// Minimal representation of lifecycle `ChangeState` request/response.
pub(crate) mod change_state {
    /// Request: ROS Transition ID (from lifecycle_msgs/msg/Transition constants).
    #[derive(Debug, Copy, Clone, Eq, PartialEq)]
    pub struct Request {
        pub transition_id: u8,
    }

    /// Response: success + human-readable message.
    #[derive(Debug, Clone, Eq, PartialEq)]
    pub struct Response {
        pub success: bool,
        pub message: String,
    }
}

/// Minimal representation of lifecycle `GetState` request/response.
pub(crate) mod get_state {
    /// Empty request (matches ROS GetState).
    #[derive(Debug, Copy, Clone, Eq, PartialEq)]
    pub struct Request;

    /// Response: current state id + label.
    #[derive(Debug, Clone, Eq, PartialEq)]
    pub struct Response {
        pub state_id: u8,
        pub label: String,
    }
}

/// Minimal representation of lifecycle `GetAvailableTransitions`.
pub(crate) mod get_available_transitions {
    /// Empty request.
    #[derive(Debug, Copy, Clone, Eq, PartialEq)]
    pub struct Request;

    /// Response: list of available transition ids + labels.
    #[derive(Debug, Clone, Eq, PartialEq)]
    pub struct Transition {
        pub id: u8,
        pub label: String,
    }

    #[derive(Debug, Clone, Eq, PartialEq)]
    pub struct Response {
        pub transitions: Vec<Transition>,
    }
}
