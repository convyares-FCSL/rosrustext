use std::time::{SystemTime, UNIX_EPOCH};

use lifecycle_msgs::msg::{
    State as RosState, Transition as RosTransition, TransitionDescription, TransitionEvent,
};
use rosrustext_core::lifecycle::State;

pub(crate) fn change_state_delay_ms() -> u64 {
    std::env::var("ROSRUSTEXT_RCLRS_CHANGE_STATE_DELAY_MS")
        .ok()
        .and_then(|val| val.parse::<u64>().ok())
        .unwrap_or(0)
}

/// Apply primary transition to start State, returning goal State and label if valid.
pub(crate) fn apply_primary_transition(
    start: State,
    transition_id: u8,
) -> Option<(State, &'static str)> {
    match (start, transition_id) {
        (State::Unconfigured, RosTransition::TRANSITION_CONFIGURE) => {
            Some((State::Inactive, "configure"))
        }
        (State::Unconfigured, RosTransition::TRANSITION_UNCONFIGURED_SHUTDOWN) => {
            Some((State::Finalized, "shutdown"))
        }

        (State::Inactive, RosTransition::TRANSITION_ACTIVATE) => Some((State::Active, "activate")),
        (State::Inactive, RosTransition::TRANSITION_CLEANUP) => {
            Some((State::Unconfigured, "cleanup"))
        }
        (State::Inactive, RosTransition::TRANSITION_INACTIVE_SHUTDOWN) => {
            Some((State::Finalized, "shutdown"))
        }

        (State::Active, RosTransition::TRANSITION_DEACTIVATE) => {
            Some((State::Inactive, "deactivate"))
        }
        (State::Active, RosTransition::TRANSITION_ACTIVE_SHUTDOWN) => {
            Some((State::Finalized, "shutdown"))
        }

        (State::Finalized, _) => None,
        _ => None,
    }
}

/// Get available primary transitions from given start State.
pub(crate) fn available_primary_transitions(start: State) -> Vec<(u8, State, &'static str)> {
    match start {
        State::Unconfigured => vec![
            (
                RosTransition::TRANSITION_CONFIGURE,
                State::Inactive,
                "configure",
            ),
            (
                RosTransition::TRANSITION_UNCONFIGURED_SHUTDOWN,
                State::Finalized,
                "shutdown",
            ),
        ],
        State::Inactive => vec![
            (RosTransition::TRANSITION_ACTIVATE, State::Active, "activate"),
            (
                RosTransition::TRANSITION_CLEANUP,
                State::Unconfigured,
                "cleanup",
            ),
            (
                RosTransition::TRANSITION_INACTIVE_SHUTDOWN,
                State::Finalized,
                "shutdown",
            ),
        ],
        State::Active => vec![
            (
                RosTransition::TRANSITION_DEACTIVATE,
                State::Inactive,
                "deactivate",
            ),
            (
                RosTransition::TRANSITION_ACTIVE_SHUTDOWN,
                State::Finalized,
                "shutdown",
            ),
        ],
        State::Finalized => vec![],
        _ => vec![],
    }
}

/// Map primary State to ROS lifecycle_msgs/msg/State id.
pub(crate) fn ros_primary_state_id(s: State) -> u8 {
    match s {
        State::Unconfigured => 1,
        State::Inactive => 2,
        State::Active => 3,
        State::Finalized => 4,
        _ => 0,
    }
}

/// Create lifecycle_msgs/msg/State from primary State.
pub(crate) fn ros_state_msg(s: State) -> RosState {
    let mut msg = RosState::default();
    msg.id = ros_primary_state_id(s);
    msg.label = format!("{:?}", s);
    msg
}

/// Create lifecycle_msgs/msg/TransitionDescription from primary transition data.
pub(crate) fn transition_description(
    start: State,
    goal: State,
    id: u8,
    label: &str,
) -> TransitionDescription {
    let mut t = RosTransition::default();
    t.id = id;
    t.label = label.to_string();

    let mut td = TransitionDescription::default();
    td.transition = t;
    td.start_state = ros_state_msg(start);
    td.goal_state = ros_state_msg(goal);
    td
}

/// Get current time in nanoseconds since UNIX_EPOCH.
pub(crate) fn now_ns() -> u64 {
    // Jazzy TransitionEvent.timestamp is u64 nanoseconds (not builtin_interfaces/Time)
    (SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos()) as u64
}

/// Create lifecycle_msgs/msg/TransitionEvent from primary transition data.
pub(crate) fn make_transition_event(
    start: State,
    goal: State,
    id: u8,
    label: &str,
) -> TransitionEvent {
    let mut t = RosTransition::default();
    t.id = id;
    t.label = label.to_string();

    let mut ev = TransitionEvent::default();
    ev.timestamp = now_ns();
    ev.transition = t;
    ev.start_state = ros_state_msg(start);
    ev.goal_state = ros_state_msg(goal);
    ev
}
