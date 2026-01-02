//! Minimal lifecycle service DTOs.
//!
//! These are wrapper-internal request/response types.
//! The transport layer maps real ROS messages into these.

/// Minimal representation of lifecycle `ChangeState` request/response.
pub(crate) mod change_state {
    /// Request: ROS Transition ID (from lifecycle_msgs/msg/Transition constants).
    #[allow(dead_code)]
    #[derive(Debug, Copy, Clone, Eq, PartialEq)]
    pub struct Request {
        pub transition_id: u8,
    }

    /// Response: success + human-readable message.
    #[allow(dead_code)]
    #[derive(Debug, Clone, Eq, PartialEq)]
    pub struct Response {
        pub success: bool,
        pub message: String,
    }
}

/// Minimal representation of lifecycle `GetState` request/response.
pub(crate) mod get_state {
    /// Empty request (matches ROS GetState).
    #[allow(dead_code)]
    #[derive(Debug, Copy, Clone, Eq, PartialEq)]
    pub struct Request;

    /// Response: current state id + label.
    #[allow(dead_code)]
    #[derive(Debug, Clone, Eq, PartialEq)]
    pub struct Response {
        pub state_id: u8,
        pub label: String,
    }
}

/// Minimal representation of lifecycle `GetAvailableTransitions`.
pub(crate) mod get_available_transitions {
    /// Empty request.
    #[allow(dead_code)]
    #[derive(Debug, Copy, Clone, Eq, PartialEq)]
    pub struct Request;

    /// Response: list of available transition ids + labels.
    #[allow(dead_code)]
    #[derive(Debug, Clone, Eq, PartialEq)]
    pub struct Transition {
        pub id: u8,
        pub label: String,
    }

    #[allow(dead_code)]
    #[derive(Debug, Clone, Eq, PartialEq)]
    pub struct Response {
        pub transitions: Vec<Transition>,
    }
}

/// Mirrors ROS2 lifecycle GetAvailableStates, but remains transport-agnostic.
pub mod get_available_states {
    #[allow(dead_code)]
    #[derive(Debug, Clone)]
    pub struct Request;

    #[allow(dead_code)]
    #[derive(Debug, Clone)]
    pub struct State {
        pub id: u8,
        pub label: String,
    }

    #[allow(dead_code)]
    #[derive(Debug, Clone)]
    pub struct Response {
        pub states: Vec<State>,
    }
}

/// Mirrors lifecycle transition graph introspection, transport-agnostic.
pub(crate) mod get_transition_graph {
    #[allow(dead_code)]
    #[derive(Debug, Copy, Clone, Eq, PartialEq)]
    pub struct Request;

    #[allow(dead_code)]
    #[derive(Debug, Clone, Eq, PartialEq)]
    pub struct State {
        pub id: u8,
        pub label: String,
    }

    #[allow(dead_code)]
    #[derive(Debug, Clone, Eq, PartialEq)]
    pub struct Transition {
        pub id: u8,
        pub label: String,
    }

    #[allow(dead_code)]
    #[derive(Debug, Clone, Eq, PartialEq)]
    pub struct TransitionDescription {
        pub transition: Transition,
        pub start_state: State,
        pub goal_state: State,
    }

    #[allow(dead_code)]
    #[derive(Debug, Clone, Eq, PartialEq)]
    pub struct Response {
        pub states: Vec<State>,
        pub transitions: Vec<TransitionDescription>,
    }
}
