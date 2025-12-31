//! rclrust_core::lifecycle
//!
//! Pure (ROS-agnostic) lifecycle semantics aligned to the ROS 2 node lifecycle model.
//! This module intentionally contains **no** ROS transport code.
//!
//! Key ideas:
//! - Stable states + transition (intermediate) states
//! - Explicit transition pipeline: `begin()` -> callback -> `finish()`
//! - Error path enters `ErrorProcessing`, then `on_error()` decides recovery
//! - Wrapper layer is responsible for ROS services/topics and policy (e.g., publisher gating)

use crate::error::{CoreError, Result};
use std::sync::atomic::{AtomicBool, Ordering};

/// ROS2-style lifecycle primary + transition (intermediate) states.
///
/// Primary (stable) states:
/// - Unconfigured, Inactive, Active, Finalized
///
/// Transition (intermediate) states:
/// - Configuring, CleaningUp, Activating, Deactivating, ShuttingDown, ErrorProcessing
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum State {
    // Primary
    Unconfigured,
    Inactive,
    Active,
    Finalized,

    // Transition (intermediate)
    Configuring,
    CleaningUp,
    Activating,
    Deactivating,
    ShuttingDown,
    ErrorProcessing,
}

/// Internal, compact IDs used for error payloads.
///
/// These are **not** ROS message IDs. They are stable, lightweight identifiers for
/// debugging/telemetry inside `rclrust_core`.
impl State {
    pub const fn id(self) -> u8 {
        match self {
            // Primary
            State::Unconfigured => 0,
            State::Inactive => 1,
            State::Active => 2,
            State::Finalized => 3,

            // Transition
            State::Configuring => 10,
            State::CleaningUp => 11,
            State::Activating => 12,
            State::Deactivating => 13,
            State::ShuttingDown => 14,
            State::ErrorProcessing => 15,
        }
    }

    /// True for stable (externally targetable) states.
    pub const fn is_primary(self) -> bool {
        matches!(
            self,
            State::Unconfigured | State::Inactive | State::Active | State::Finalized
        )
    }

    /// True for intermediate states entered while callbacks are running.
    pub const fn is_transitioning(self) -> bool {
        !self.is_primary()
    }
}

/// ROS2-style lifecycle transitions (requests).
///
/// These are the **user-invoked** transitions. All "ON_*_SUCCESS/FAILURE/ERROR"
/// transitions are modeled via `finish(intermediate, via, CallbackResult)`.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum Transition {
    Configure,
    Activate,
    Deactivate,
    Cleanup,
    Shutdown,
}

/// Internal, compact IDs used for error payloads.
///
/// These are **not** the ROS `lifecycle_msgs/msg/Transition` IDs.
/// The wrapper layer performs ROS ID mapping.
impl Transition {
    pub const fn id(self) -> u8 {
        match self {
            Transition::Configure => 1,
            Transition::Cleanup => 2,
            Transition::Activate => 3,
            Transition::Deactivate => 4,
            Transition::Shutdown => 5,
        }
    }
}

/// Result of executing a lifecycle transition callback.
///
/// Mirrors the ROS lifecycle callback contract:
/// - Success: proceed to the expected next state
/// - Failure: rollback / remain in previous stable state (depends on transition)
/// - Error: enter `ErrorProcessing`
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum CallbackResult {
    Success,
    Failure,
    Error,
}

/// Simple activation gate for managed resources.
///
/// Intended use (wrapper layer):
/// - set `activate()` when state becomes Active
/// - set `deactivate()` when leaving Active
/// - publisher/timer wrappers check `is_active()` to allow or block work
#[derive(Debug)]
pub struct ActivationGate {
    active: AtomicBool,
}

impl ActivationGate {
    pub const fn new() -> Self {
        Self {
            active: AtomicBool::new(false),
        }
    }

    pub fn activate(&self) {
        self.active.store(true, Ordering::Release);
    }

    pub fn deactivate(&self) {
        self.active.store(false, Ordering::Release);
    }

    pub fn is_active(&self) -> bool {
        self.active.load(Ordering::Acquire)
    }
}

impl Default for ActivationGate {
    fn default() -> Self {
        Self::new()
    }
}

/// Callbacks invoked during lifecycle transitions.
///
/// Implementors own resources and decide what Success/Failure/Error means.
/// The wrapper will typically invoke `drive()` from service handlers.
pub trait LifecycleCallbacks {
    fn on_configure(&mut self) -> CallbackResult;
    fn on_activate(&mut self) -> CallbackResult;
    fn on_deactivate(&mut self) -> CallbackResult;
    fn on_cleanup(&mut self) -> CallbackResult;
    fn on_shutdown(&mut self) -> CallbackResult;

    /// Called when a transition callback reports `CallbackResult::Error`.
    ///
    /// ROS2 diagram semantics:
    /// - Success -> Unconfigured
    /// - Failure -> Finalized
    fn on_error(&mut self) -> CallbackResult;
}

/// Begin a lifecycle transition by moving from a **stable** state into the correct
/// **intermediate** state.
///
/// This enforces:
/// - which transitions are allowed from which states
/// - that you cannot start a new transition while already transitioning
pub fn begin(current: State, via: Transition) -> Result<State> {
    use State::*;
    use Transition::*;

    let next = match (current, via) {
        (Unconfigured, Configure) => Configuring,
        (Inactive, Activate) => Activating,
        (Active, Deactivate) => Deactivating,
        (Inactive, Cleanup) => CleaningUp,

        // Shutdown can start from any non-final state
        (s, Shutdown) if s != Finalized => ShuttingDown,

        _ => {
            return Err(CoreError::invalid_transition_lifecycle(
                current.id(),
                via.id(),
            ));
        }
    };

    Ok(next)
}

/// Finish a lifecycle transition by exiting an **intermediate** state to a **stable** state,
/// based on callback outcome.
///
/// This models ROS2 implicit ON_* transitions.
pub fn finish(intermediate: State, via: Transition, result: CallbackResult) -> Result<State> {
    use CallbackResult::*;
    use State::*;
    use Transition::*;

    let next = match (intermediate, via, result) {
        // Configure: Unconfigured -> Configuring -> (Inactive | Unconfigured | ErrorProcessing)
        (Configuring, Configure, Success) => Inactive,
        (Configuring, Configure, Failure) => Unconfigured,
        (Configuring, Configure, Error) => ErrorProcessing,

        // Activate: Inactive -> Activating -> (Active | Inactive | ErrorProcessing)
        (Activating, Activate, Success) => Active,
        (Activating, Activate, Failure) => Inactive,
        (Activating, Activate, Error) => ErrorProcessing,

        // Deactivate: Active -> Deactivating -> (Inactive | Active | ErrorProcessing)
        (Deactivating, Deactivate, Success) => Inactive,
        (Deactivating, Deactivate, Failure) => Active,
        (Deactivating, Deactivate, Error) => ErrorProcessing,

        // Cleanup: Inactive -> CleaningUp -> (Unconfigured | Inactive | ErrorProcessing)
        (CleaningUp, Cleanup, Success) => Unconfigured,
        (CleaningUp, Cleanup, Failure) => Inactive,
        (CleaningUp, Cleanup, Error) => ErrorProcessing,

        // Shutdown: * -> ShuttingDown -> Finalized
        // (ROS2 commonly treats shutdown as terminal regardless of callback outcome)
        (ShuttingDown, Shutdown, _) => Finalized,

        _ => {
            return Err(CoreError::invalid_transition_lifecycle(
                intermediate.id(),
                via.id(),
            ));
        }
    };

    Ok(next)
}

/// Drive a full lifecycle transition and expose intermediate state for introspection.
///
/// Returns:
/// - `(intermediate_state, final_stable_state)`
///
/// Notes:
/// - If a transition callback returns `Error`, `finish()` yields `ErrorProcessing`.
/// - In that case, `on_error()` is invoked and recovery is applied:
///   - Success -> Unconfigured
///   - Failure/Error -> Finalized
pub fn drive(
    current: State,
    via: Transition,
    callbacks: &mut dyn LifecycleCallbacks,
) -> Result<(State, State)> {
    let intermediate = begin(current, via)?;

    // Execute the transition callback
    let result = match via {
        Transition::Configure => callbacks.on_configure(),
        Transition::Activate => callbacks.on_activate(),
        Transition::Deactivate => callbacks.on_deactivate(),
        Transition::Cleanup => callbacks.on_cleanup(),
        Transition::Shutdown => callbacks.on_shutdown(),
    };

    // Apply implicit ON_* transition
    let final_state = finish(intermediate, via, result)?;

    // ErrorProcessing recovery (ROS2 diagram: success -> unconfigured, failure -> finalized)
    let final_state = if final_state == State::ErrorProcessing {
        match callbacks.on_error() {
            CallbackResult::Success => State::Unconfigured,
            CallbackResult::Failure => State::Finalized,
            CallbackResult::Error => State::Finalized, // defensive: treat as failure
        }
    } else {
        final_state
    };

    Ok((intermediate, final_state))
}

/// Get the list of available transitions from a given state.
///
/// This supports lifecycle manager introspection and wrapper-layer service responses.
///
/// Design:
/// - For transition states (busy), returns empty: external transitions are rejected while busy.
pub fn available_transitions(state: State) -> &'static [Transition] {
    use State::*;
    use Transition::*;

    match state {
        Unconfigured => &[Configure, Shutdown],
        Inactive => &[Activate, Cleanup, Shutdown],
        Active => &[Deactivate, Shutdown],
        Finalized => &[],
        // Transition states: no external transitions while busy
        Configuring | CleaningUp | Activating | Deactivating | ShuttingDown | ErrorProcessing => &[],
    }
}

/// ROS2 lifecycle transition IDs (subset needed for external control).
///
/// These mirror `lifecycle_msgs/msg/Transition` numeric constants.
/// Keeping these in core avoids scattering "magic numbers" in the wrapper.
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
        _ => None, // CREATE/DESTROY handled by wrapper/process
    }
}

/// Get the correct ROS2 shutdown transition ID for a given *stable* state.
///
/// This is used by the wrapper when responding to lifecycle manager requests or reporting transitions.
pub fn shutdown_ros_id_for_state(state: State) -> Option<u8> {
    match state {
        State::Unconfigured => Some(ros_ids::TRANSITION_UNCONFIGURED_SHUTDOWN),
        State::Inactive => Some(ros_ids::TRANSITION_INACTIVE_SHUTDOWN),
        State::Active => Some(ros_ids::TRANSITION_ACTIVE_SHUTDOWN),
        _ => None, // Finalized/transition states are not valid shutdown request origins
    }
}

//
// Tests
//

/// Unit tests for lifecycle state machine primitives.
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn invalid_transition_has_payload() {
        let e = begin(State::Active, Transition::Cleanup).unwrap_err();
        assert_eq!(e.kind, crate::error::ErrorKind::InvalidTransition);
        assert_eq!(e.domain, crate::error::Domain::Lifecycle);

        match e.payload {
            crate::error::Payload::LifecycleTransition {
                from_state,
                via_transition,
            } => {
                assert_eq!(from_state, State::Active.id());
                assert_eq!(via_transition, Transition::Cleanup.id());
            }
            _ => panic!("expected LifecycleTransition payload"),
        }
    }

    #[test]
    fn configure_success_path_uses_intermediate_state() {
        let mid = begin(State::Unconfigured, Transition::Configure).unwrap();
        assert_eq!(mid, State::Configuring);

        let end = finish(mid, Transition::Configure, CallbackResult::Success).unwrap();
        assert_eq!(end, State::Inactive);
    }

    #[test]
    fn configure_error_enters_error_processing() {
        let mid = begin(State::Unconfigured, Transition::Configure).unwrap();
        let end = finish(mid, Transition::Configure, CallbackResult::Error).unwrap();
        assert_eq!(end, State::ErrorProcessing);
    }

    #[test]
    fn available_transitions_test() {
        let transitions = available_transitions(State::Active);

        assert_eq!(transitions.len(), 2);
        assert!(transitions.contains(&Transition::Deactivate));
        assert!(!transitions.contains(&Transition::Activate));
    }

    #[test]
    fn activation_gate_test() {
        let gate = ActivationGate::new();

        assert!(!gate.is_active());

        gate.activate();
        assert!(gate.is_active());

        gate.deactivate();
        assert!(!gate.is_active());
    }

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

        // Not valid from these
        assert_eq!(shutdown_ros_id_for_state(State::Finalized), None);
        assert_eq!(shutdown_ros_id_for_state(State::Configuring), None);
    }
}

/// Unit tests for the lifecycle driver (callback execution + recovery policy).
#[cfg(test)]
mod drive_tests {
    use super::*;

    struct OkCallbacks;

    impl LifecycleCallbacks for OkCallbacks {
        fn on_configure(&mut self) -> CallbackResult {
            CallbackResult::Success
        }
        fn on_activate(&mut self) -> CallbackResult {
            CallbackResult::Success
        }
        fn on_deactivate(&mut self) -> CallbackResult {
            CallbackResult::Success
        }
        fn on_cleanup(&mut self) -> CallbackResult {
            CallbackResult::Success
        }
        fn on_shutdown(&mut self) -> CallbackResult {
            CallbackResult::Success
        }
        fn on_error(&mut self) -> CallbackResult {
            CallbackResult::Success
        }
    }

    #[test]
    fn drive_configure_then_activate_reaches_active() {
        let mut cb = OkCallbacks;

        let (_mid1, s1) = drive(State::Unconfigured, Transition::Configure, &mut cb).unwrap();
        assert_eq!(s1, State::Inactive);

        let (_mid2, s2) = drive(s1, Transition::Activate, &mut cb).unwrap();
        assert_eq!(s2, State::Active);
    }

    #[test]
    fn drive_configure_reports_intermediate_state() {
        let mut cb = OkCallbacks;

        let (mid, end) = drive(State::Unconfigured, Transition::Configure, &mut cb).unwrap();
        assert_eq!(mid, State::Configuring);
        assert_eq!(end, State::Inactive);
    }

    #[test]
    fn error_processing_failure_finalizes() {
        struct ErrThenFail;

        impl LifecycleCallbacks for ErrThenFail {
            fn on_configure(&mut self) -> CallbackResult {
                CallbackResult::Error // triggers ErrorProcessing
            }
            fn on_activate(&mut self) -> CallbackResult {
                CallbackResult::Success
            }
            fn on_deactivate(&mut self) -> CallbackResult {
                CallbackResult::Success
            }
            fn on_cleanup(&mut self) -> CallbackResult {
                CallbackResult::Success
            }
            fn on_shutdown(&mut self) -> CallbackResult {
                CallbackResult::Success
            }
            fn on_error(&mut self) -> CallbackResult {
                CallbackResult::Failure // ErrorProcessing -> Finalized
            }
        }

        let mut cb = ErrThenFail;
        let (_mid, end) = drive(State::Unconfigured, Transition::Configure, &mut cb).unwrap();
        assert_eq!(end, State::Finalized);
    }
}