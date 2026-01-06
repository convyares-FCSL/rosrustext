use std::time::{SystemTime, UNIX_EPOCH};

use rosrustext_msgs::lifecycle_msgs::msg::{State as RosState, Transition as RosTransition, TransitionDescription, TransitionEvent};
use rosrustext_core::lifecycle::{goal_state_for_transition, CallbackResult, State, Transition};

pub(crate) fn change_state_delay_ms() -> u64 {
    std::env::var("ROSRUSTEXT_RCLRS_CHANGE_STATE_DELAY_MS").ok().and_then(|val| val.parse::<u64>().ok()).unwrap_or(0)
}

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub(crate) struct TransitionSpec {
    pub(crate) start: State,
    pub(crate) transition: Transition,
    pub(crate) transition_id: u8,
    pub(crate) label: &'static str,
}

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub(crate) struct TransitionEntry {
    pub(crate) spec: TransitionSpec,
    pub(crate) goal: State,
}

const TRANSITION_SPECS: &[TransitionSpec] = &[
    TransitionSpec {
        start: State::Unconfigured,
        transition: Transition::Configure,
        transition_id: RosTransition::TRANSITION_CONFIGURE,
        label: Transition::Configure.label(),
    },
    TransitionSpec {
        start: State::Unconfigured,
        transition: Transition::Shutdown,
        transition_id: RosTransition::TRANSITION_UNCONFIGURED_SHUTDOWN,
        label: Transition::Shutdown.label(),
    },
    TransitionSpec {
        start: State::Inactive,
        transition: Transition::Activate,
        transition_id: RosTransition::TRANSITION_ACTIVATE,
        label: Transition::Activate.label(),
    },
    TransitionSpec {
        start: State::Inactive,
        transition: Transition::Cleanup,
        transition_id: RosTransition::TRANSITION_CLEANUP,
        label: Transition::Cleanup.label(),
    },
    TransitionSpec {
        start: State::Inactive,
        transition: Transition::Shutdown,
        transition_id: RosTransition::TRANSITION_INACTIVE_SHUTDOWN,
        label: Transition::Shutdown.label(),
    },
    TransitionSpec {
        start: State::Active,
        transition: Transition::Deactivate,
        transition_id: RosTransition::TRANSITION_DEACTIVATE,
        label: Transition::Deactivate.label(),
    },
    TransitionSpec {
        start: State::Active,
        transition: Transition::Shutdown,
        transition_id: RosTransition::TRANSITION_ACTIVE_SHUTDOWN,
        label: Transition::Shutdown.label(),
    },
];

pub(crate) fn transition_spec_for_ros_id(start: State, transition_id: u8) -> Option<TransitionSpec> {
    TRANSITION_SPECS.iter().copied().find(|spec| spec.start == start && spec.transition_id == transition_id)
}

pub(crate) fn transition_entries_for_start(start: State) -> Vec<TransitionEntry> {
    TRANSITION_SPECS
        .iter()
        .copied()
        .filter(|spec| spec.start == start)
        .filter_map(|spec| {
            let goal = goal_state_for_transition(spec.start, spec.transition).ok()?;
            Some(TransitionEntry { spec, goal })
        })
        .collect()
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
    env_callback_result("ROSRUSTEXT_RCLRS_TRANSITION_RESULT", transition).unwrap_or(CallbackResult::Success)
}

pub(crate) fn on_error_result_for(transition: Transition) -> CallbackResult {
    env_callback_result("ROSRUSTEXT_RCLRS_ON_ERROR_RESULT", transition).unwrap_or(CallbackResult::Success)
}

/// Get available primary transitions from given start State.
/// Map primary State to ROS lifecycle_msgs/msg/State id.
pub(crate) fn ros_primary_state_id(s: State) -> u8 {
    match s {
        State::Unconfigured => 1,
        State::Inactive => 2,
        State::Active => 3,
        State::Finalized => 4,
        State::Configuring => 10,
        State::CleaningUp => 11,
        State::ShuttingDown => 12,
        State::Activating => 13,
        State::Deactivating => 14,
        State::ErrorProcessing => 15,
        _ => 0,
    }
}

/// Create lifecycle_msgs/msg/State from primary State.
pub(crate) fn ros_state_msg(s: State) -> RosState {
    RosState { id: ros_primary_state_id(s), label: format!("{:?}", s) }
}

/// Create lifecycle_msgs/msg/TransitionDescription from primary transition data.
pub(crate) fn transition_description(start: State, goal: State, id: u8, label: &str) -> TransitionDescription {
    let t = RosTransition { id, label: label.to_string() };

    TransitionDescription { transition: t, start_state: ros_state_msg(start), goal_state: ros_state_msg(goal) }
}

/// Get current time in nanoseconds since UNIX_EPOCH.
pub(crate) fn now_ns() -> u64 {
    // Jazzy TransitionEvent.timestamp is u64 nanoseconds (not builtin_interfaces/Time)
    (SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or_default().as_nanos()) as u64
}

/// Create lifecycle_msgs/msg/TransitionEvent from primary transition data.
pub(crate) fn make_transition_event(start: State, goal: State, id: u8, label: &str) -> TransitionEvent {
    let t = RosTransition { id, label: label.to_string() };

    TransitionEvent {
        timestamp: now_ns(),
        transition: t,
        start_state: ros_state_msg(start),
        goal_state: ros_state_msg(goal),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transition_table_matches_core_goals() {
        for start in [State::Unconfigured, State::Inactive, State::Active] {
            for entry in transition_entries_for_start(start) {
                let expected = goal_state_for_transition(entry.spec.start, entry.spec.transition)
                    .expect("goal_state_for_transition failed");
                assert_eq!(entry.goal, expected);
                assert_eq!(entry.spec.label, entry.spec.transition.label());
            }
        }
    }
}
