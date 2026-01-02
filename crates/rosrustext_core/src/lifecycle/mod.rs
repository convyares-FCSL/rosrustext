//! rosrustext_core::lifecycle
//!
//! Pure (ROS-agnostic) lifecycle semantics aligned to the ROS 2 node lifecycle model.
//! This module intentionally contains **no** ROS transport code.
//!
//! Key ideas:
//! - Stable states + transition (intermediate) states
//! - Explicit transition pipeline: `begin()` -> callback -> `finish()`
//! - Error path enters `ErrorProcessing`, then `on_error()` decides recovery
//! - Wrapper layer is responsible for ROS services/topics and policy (e.g., publisher gating)

mod gate;
mod engine;
mod graph;
mod state;
mod transition;

pub use gate::ActivationGate;
pub use engine::{
    available_transitions, begin, drive, finish, finish_with_error_handling, CallbackResult,
    LifecycleCallbacks, goal_state_for_transition,
};
pub use graph::{transition_graph, TransitionEdge, TransitionGraph};
pub use state::{State, ALL_STATES};
pub use transition::Transition;
