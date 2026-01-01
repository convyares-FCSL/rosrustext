use crate::error::{CoreError, Result};

use super::{State, Transition};

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

/// Get the expected goal state when a transition succeeds.
pub fn goal_state_for_transition(start: State, transition: Transition) -> Result<State> {
    let intermediate = begin(start, transition)?;
    finish(intermediate, transition, CallbackResult::Success)
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
        Configuring | CleaningUp | Activating | Deactivating | ShuttingDown | ErrorProcessing => {
            &[]
        }
    }
}

//
// Tests
//

/// Unit tests for lifecycle state machine primitives.
#[cfg(test)]
mod tests {
    use super::*;
    use crate::error::{Domain, ErrorKind, Payload};

    #[derive(Clone, Copy)]
    struct TestCallbacks {
        on_configure: CallbackResult,
        on_error: CallbackResult,
    }

    impl Default for TestCallbacks {
        fn default() -> Self {
            Self {
                on_configure: CallbackResult::Success,
                on_error: CallbackResult::Success,
            }
        }
    }

    impl LifecycleCallbacks for TestCallbacks {
        fn on_configure(&mut self) -> CallbackResult {
            self.on_configure
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
            self.on_error
        }
    }

    #[test]
    fn invalid_transition_has_payload() {
        let e = begin(State::Active, Transition::Cleanup).unwrap_err();
        assert_eq!(e.kind, ErrorKind::InvalidTransition);
        assert_eq!(e.domain, Domain::Lifecycle);

        match e.payload {
            Payload::LifecycleTransition {
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
    fn available_transitions_test() {
        let transitions = available_transitions(State::Active);

        assert_eq!(transitions.len(), 2);
        assert!(transitions.contains(&Transition::Deactivate));
        assert!(!transitions.contains(&Transition::Activate));
    }

    #[test]
    fn drive_error_path_goes_through_error_processing() {
        let mut cb = TestCallbacks {
            on_configure: CallbackResult::Error,
            ..TestCallbacks::default()
        };
        let (mid, end) = drive(State::Unconfigured, Transition::Configure, &mut cb).unwrap();
        assert_eq!(mid, State::Configuring);
        assert_eq!(end, State::Unconfigured);
    }

    #[test]
    fn error_processing_failure_finalizes() {
        let mut cb = TestCallbacks {
            on_configure: CallbackResult::Error,
            on_error: CallbackResult::Failure,
        };
        let (_mid, end) = drive(State::Unconfigured, Transition::Configure, &mut cb).unwrap();
        assert_eq!(end, State::Finalized);
    }

    #[derive(Clone, Copy)]
    struct ScenarioCallbacks {
        configure: CallbackResult,
        activate: CallbackResult,
        deactivate: CallbackResult,
        cleanup: CallbackResult,
        shutdown: CallbackResult,
        on_error: CallbackResult,
    }

    impl ScenarioCallbacks {
        fn for_transition(
            transition: Transition,
            result: CallbackResult,
            on_error: CallbackResult,
        ) -> Self {
            let mut cb = Self {
                configure: CallbackResult::Success,
                activate: CallbackResult::Success,
                deactivate: CallbackResult::Success,
                cleanup: CallbackResult::Success,
                shutdown: CallbackResult::Success,
                on_error,
            };
            match transition {
                Transition::Configure => cb.configure = result,
                Transition::Activate => cb.activate = result,
                Transition::Deactivate => cb.deactivate = result,
                Transition::Cleanup => cb.cleanup = result,
                Transition::Shutdown => cb.shutdown = result,
            }
            cb
        }
    }

    impl LifecycleCallbacks for ScenarioCallbacks {
        fn on_configure(&mut self) -> CallbackResult {
            self.configure
        }
        fn on_activate(&mut self) -> CallbackResult {
            self.activate
        }
        fn on_deactivate(&mut self) -> CallbackResult {
            self.deactivate
        }
        fn on_cleanup(&mut self) -> CallbackResult {
            self.cleanup
        }
        fn on_shutdown(&mut self) -> CallbackResult {
            self.shutdown
        }
        fn on_error(&mut self) -> CallbackResult {
            self.on_error
        }
    }

    fn drive_once(
        start: State,
        transition: Transition,
        result: CallbackResult,
        on_error: CallbackResult,
    ) -> State {
        let mut cb = ScenarioCallbacks::for_transition(transition, result, on_error);
        let (_mid, end) = drive(start, transition, &mut cb).unwrap();
        end
    }

    #[test]
    fn failure_paths_return_to_previous_state() {
        let cases = [
            (State::Unconfigured, Transition::Configure, State::Unconfigured),
            (State::Inactive, Transition::Activate, State::Inactive),
            (State::Active, Transition::Deactivate, State::Active),
            (State::Inactive, Transition::Cleanup, State::Inactive),
            (State::Active, Transition::Shutdown, State::Finalized),
        ];

        for (start, transition, expected) in cases {
            let end = drive_once(start, transition, CallbackResult::Failure, CallbackResult::Success);
            assert_eq!(end, expected);
        }
    }

    #[test]
    fn error_paths_follow_error_processing_rules() {
        let cases = [
            (State::Unconfigured, Transition::Configure, State::Unconfigured),
            (State::Inactive, Transition::Activate, State::Unconfigured),
            (State::Active, Transition::Deactivate, State::Unconfigured),
            (State::Inactive, Transition::Cleanup, State::Unconfigured),
            (State::Active, Transition::Shutdown, State::Finalized),
        ];

        for (start, transition, expected) in cases {
            let end = drive_once(start, transition, CallbackResult::Error, CallbackResult::Success);
            assert_eq!(end, expected);
        }
    }
}
