use rosrustext_core::error::Result;
use rosrustext_core::lifecycle::{begin, finish, CallbackResult, State, Transition, ALL_STATES};

/// ROS2 lifecycle transition IDs (subset needed for external control).
///
/// These mirror `lifecycle_msgs/msg/Transition` numeric constants.
pub mod ros_ids {
    pub const TRANSITION_CREATE: u8 = 0;
    pub const TRANSITION_CONFIGURE: u8 = 1;
    pub const TRANSITION_CLEANUP: u8 = 2;
    pub const TRANSITION_ACTIVATE: u8 = 3;
    pub const TRANSITION_DEACTIVATE: u8 = 4;
    pub const TRANSITION_UNCONFIGURED_SHUTDOWN: u8 = 5;
    pub const TRANSITION_INACTIVE_SHUTDOWN: u8 = 6;
    pub const TRANSITION_ACTIVE_SHUTDOWN: u8 = 7;
    pub const TRANSITION_DESTROY: u8 = 8;
}

/// ROS2 lifecycle state IDs (primary + transition).
///
/// These mirror `lifecycle_msgs/msg/State` numeric constants.
pub mod ros_state_ids {
    pub const PRIMARY_STATE_UNCONFIGURED: u8 = 1;
    pub const PRIMARY_STATE_INACTIVE: u8 = 2;
    pub const PRIMARY_STATE_ACTIVE: u8 = 3;
    pub const PRIMARY_STATE_FINALIZED: u8 = 4;
    pub const TRANSITION_STATE_CONFIGURING: u8 = 10;
    pub const TRANSITION_STATE_CLEANINGUP: u8 = 11;
    pub const TRANSITION_STATE_SHUTTINGDOWN: u8 = 12;
    pub const TRANSITION_STATE_ACTIVATING: u8 = 13;
    pub const TRANSITION_STATE_DEACTIVATING: u8 = 14;
    pub const TRANSITION_STATE_ERRORPROCESSING: u8 = 15;
}

const ROS_STATE_IDS: [u8; 10] = [
    ros_state_ids::PRIMARY_STATE_UNCONFIGURED,
    ros_state_ids::PRIMARY_STATE_INACTIVE,
    ros_state_ids::PRIMARY_STATE_ACTIVE,
    ros_state_ids::PRIMARY_STATE_FINALIZED,
    ros_state_ids::TRANSITION_STATE_CONFIGURING,
    ros_state_ids::TRANSITION_STATE_CLEANINGUP,
    ros_state_ids::TRANSITION_STATE_ACTIVATING,
    ros_state_ids::TRANSITION_STATE_DEACTIVATING,
    ros_state_ids::TRANSITION_STATE_SHUTTINGDOWN,
    ros_state_ids::TRANSITION_STATE_ERRORPROCESSING,
];

fn state_index(state: State) -> usize {
    ALL_STATES
        .iter()
        .position(|s| *s == state)
        .expect("ALL_STATES is missing a lifecycle state")
}

/// Map a ROS lifecycle transition ID into the core semantic Transition.
///
/// Notes:
/// - CREATE/DESTROY are process/construct/destruct concerns and are handled by the wrapper/app.
/// - Shutdown uses distinct ROS IDs depending on start state; core collapses them to `Shutdown`.
pub fn transition_from_ros_id(id: u8) -> Option<Transition> {
    match id {
        ros_ids::TRANSITION_CONFIGURE => Some(Transition::Configure),
        ros_ids::TRANSITION_CLEANUP => Some(Transition::Cleanup),
        ros_ids::TRANSITION_ACTIVATE => Some(Transition::Activate),
        ros_ids::TRANSITION_DEACTIVATE => Some(Transition::Deactivate),
        ros_ids::TRANSITION_UNCONFIGURED_SHUTDOWN
        | ros_ids::TRANSITION_INACTIVE_SHUTDOWN
        | ros_ids::TRANSITION_ACTIVE_SHUTDOWN => Some(Transition::Shutdown),
        _ => None,
    }
}

/// Get the correct ROS2 shutdown transition ID for a given *stable* state.
pub fn shutdown_ros_id_for_state(state: State) -> Option<u8> {
    match state {
        State::Unconfigured => Some(ros_ids::TRANSITION_UNCONFIGURED_SHUTDOWN),
        State::Inactive => Some(ros_ids::TRANSITION_INACTIVE_SHUTDOWN),
        State::Active => Some(ros_ids::TRANSITION_ACTIVE_SHUTDOWN),
        _ => None,
    }
}

/// Map a core lifecycle state to its ROS2 lifecycle state ID.
pub fn ros_state_id(state: State) -> u8 {
    ROS_STATE_IDS[state_index(state)]
}

/// Map a ROS2 lifecycle state ID to a core lifecycle state.
pub fn state_from_ros_id(id: u8) -> Option<State> {
    ROS_STATE_IDS
        .iter()
        .position(|value| *value == id)
        .and_then(|idx| ALL_STATES.get(idx).copied())
}

/// Get the correct ROS2 lifecycle transition ID for a transition from a given state.
pub fn ros_transition_id(start: State, transition: Transition) -> Option<u8> {
    match transition {
        Transition::Configure => Some(ros_ids::TRANSITION_CONFIGURE),
        Transition::Cleanup => Some(ros_ids::TRANSITION_CLEANUP),
        Transition::Activate => Some(ros_ids::TRANSITION_ACTIVATE),
        Transition::Deactivate => Some(ros_ids::TRANSITION_DEACTIVATE),
        Transition::Shutdown => shutdown_ros_id_for_state(start),
    }
}

/// Get the expected goal state when a transition succeeds.
pub fn goal_state_for_transition(start: State, transition: Transition) -> Result<State> {
    let intermediate = begin(start, transition)?;
    finish(intermediate, transition, CallbackResult::Success)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transition_from_ros_id_maps_shutdown_variants() {
        assert_eq!(
            transition_from_ros_id(ros_ids::TRANSITION_UNCONFIGURED_SHUTDOWN),
            Some(Transition::Shutdown)
        );
        assert_eq!(
            transition_from_ros_id(ros_ids::TRANSITION_INACTIVE_SHUTDOWN),
            Some(Transition::Shutdown)
        );
        assert_eq!(
            transition_from_ros_id(ros_ids::TRANSITION_ACTIVE_SHUTDOWN),
            Some(Transition::Shutdown)
        );
    }

    #[test]
    fn shutdown_ros_id_for_state_maps_primary_states() {
        assert_eq!(
            shutdown_ros_id_for_state(State::Active),
            Some(ros_ids::TRANSITION_ACTIVE_SHUTDOWN)
        );
        assert_eq!(
            shutdown_ros_id_for_state(State::Inactive),
            Some(ros_ids::TRANSITION_INACTIVE_SHUTDOWN)
        );
        assert_eq!(
            shutdown_ros_id_for_state(State::Unconfigured),
            Some(ros_ids::TRANSITION_UNCONFIGURED_SHUTDOWN)
        );
        assert_eq!(shutdown_ros_id_for_state(State::Finalized), None);
        assert_eq!(shutdown_ros_id_for_state(State::Configuring), None);
    }
}
