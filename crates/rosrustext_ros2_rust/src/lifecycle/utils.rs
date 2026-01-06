use std::time::{SystemTime, UNIX_EPOCH};

use lifecycle_msgs::msg::{
    State as RosState, Transition as RosTransition, TransitionDescription, TransitionEvent,
};
use rosrustext_core::lifecycle::{CallbackResult, State, Transition};

pub(crate) fn change_state_delay_ms() -> u64 {
    std::env::var("ROSRUSTEXT_RCLRS_CHANGE_STATE_DELAY_MS")
        .ok()
        .and_then(|val| val.parse::<u64>().ok())
        .unwrap_or(0)
}

pub(crate) fn transition_from_ros_id(start: State, transition_id: u8) -> Option<Transition> {
    match transition_id {
        RosTransition::TRANSITION_CONFIGURE => Some(Transition::Configure),
        RosTransition::TRANSITION_CLEANUP => Some(Transition::Cleanup),
        RosTransition::TRANSITION_ACTIVATE => Some(Transition::Activate),
        RosTransition::TRANSITION_DEACTIVATE => Some(Transition::Deactivate),
        RosTransition::TRANSITION_UNCONFIGURED_SHUTDOWN if start == State::Unconfigured => {
            Some(Transition::Shutdown)
        }
        RosTransition::TRANSITION_INACTIVE_SHUTDOWN if start == State::Inactive => {
            Some(Transition::Shutdown)
        }
        RosTransition::TRANSITION_ACTIVE_SHUTDOWN if start == State::Active => {
            Some(Transition::Shutdown)
        }
        _ => None,
    }
}

fn transition_env_suffix(transition: Transition) -> &'static str {
    match transition {
        Transition::Configure => "CONFIGURE",
        Transition::Cleanup => "CLEANUP",
        Transition::Activate => "ACTIVATE",
        Transition::Deactivate => "DEACTIVATE",
        Transition::Shutdown => "SHUTDOWN",
    }
}

fn parse_callback_result(raw: &str) -> Option<CallbackResult> {
    match raw.trim().to_ascii_lowercase().as_str() {
        "success" | "ok" => Some(CallbackResult::Success),
        "failure" | "fail" => Some(CallbackResult::Failure),
        "error" | "err" => Some(CallbackResult::Error),
        _ => None,
    }
}

fn env_callback_result(prefix: &str, transition: Transition) -> Option<CallbackResult> {
    let suffix = transition_env_suffix(transition);
    let specific = format!("{prefix}_{suffix}");
    if let Ok(val) = std::env::var(&specific) {
        if let Some(parsed) = parse_callback_result(&val) {
            return Some(parsed);
        }
    }
    if let Ok(val) = std::env::var(prefix) {
        return parse_callback_result(&val);
    }
    None
}

pub(crate) fn transition_result_for(transition: Transition) -> CallbackResult {
    env_callback_result("ROSRUSTEXT_RCLRS_TRANSITION_RESULT", transition)
        .unwrap_or(CallbackResult::Success)
}

pub(crate) fn on_error_result_for(transition: Transition) -> CallbackResult {
    env_callback_result("ROSRUSTEXT_RCLRS_ON_ERROR_RESULT", transition)
        .unwrap_or(CallbackResult::Success)
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
