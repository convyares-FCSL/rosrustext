use rosrustext_core::error::ErrorKind;
use rosrustext_core::lifecycle::{
    available_transitions, begin, finish, finish_with_error_handling, goal_state_for_transition,
    CallbackResult, LifecycleCallbacks, State, Transition,
};

#[derive(Clone, Copy)]
struct TestCallbacks {
    result: CallbackResult,
    on_error: CallbackResult,
}

impl LifecycleCallbacks for TestCallbacks {
    fn on_configure(&mut self) -> CallbackResult {
        self.result
    }
    fn on_activate(&mut self) -> CallbackResult {
        self.result
    }
    fn on_deactivate(&mut self) -> CallbackResult {
        self.result
    }
    fn on_cleanup(&mut self) -> CallbackResult {
        self.result
    }
    fn on_shutdown(&mut self) -> CallbackResult {
        self.result
    }
    fn on_error(&mut self) -> CallbackResult {
        self.on_error
    }
}

#[test]
fn success_moves_to_expected_goal_state() {
    let cases = [
        (State::Unconfigured, Transition::Configure, State::Inactive),
        (State::Inactive, Transition::Activate, State::Active),
        (State::Active, Transition::Deactivate, State::Inactive),
        (State::Inactive, Transition::Cleanup, State::Unconfigured),
        (State::Unconfigured, Transition::Shutdown, State::Finalized),
        (State::Inactive, Transition::Shutdown, State::Finalized),
        (State::Active, Transition::Shutdown, State::Finalized),
    ];

    for (start, transition, expected_goal) in cases {
        let mut callbacks = TestCallbacks {
            result: CallbackResult::Success,
            on_error: CallbackResult::Success,
        };
        let intermediate = begin(start, transition).expect("begin should succeed");
        let final_state =
            finish(intermediate, transition, callbacks.result).expect("finish should succeed");
        assert_eq!(final_state, expected_goal);

        let goal_from_api = goal_state_for_transition(start, transition).unwrap();
        assert_eq!(goal_from_api, expected_goal);

        let (_, driven_state) =
            rosrustext_core::lifecycle::drive(start, transition, &mut callbacks).unwrap();
        assert_eq!(driven_state, expected_goal);
    }
}

#[test]
fn failure_returns_to_origin_state() {
    let cases = [
        (
            State::Unconfigured,
            Transition::Configure,
            State::Unconfigured,
        ),
        (State::Inactive, Transition::Activate, State::Inactive),
        (State::Active, Transition::Deactivate, State::Active),
        (State::Inactive, Transition::Cleanup, State::Inactive),
        (State::Unconfigured, Transition::Shutdown, State::Finalized),
        (State::Inactive, Transition::Shutdown, State::Finalized),
        (State::Active, Transition::Shutdown, State::Finalized),
    ];

    for (start, transition, expected_goal) in cases {
        let intermediate = begin(start, transition).expect("begin should succeed");
        let final_state = finish(intermediate, transition, CallbackResult::Failure)
            .expect("finish should succeed");
        assert_eq!(final_state, expected_goal);
    }
}

#[test]
fn error_routes_through_error_processing_and_recovery() {
    let start = State::Unconfigured;
    let transition = Transition::Configure;
    let intermediate = begin(start, transition).unwrap();

    let error_state = finish(intermediate, transition, CallbackResult::Error).unwrap();
    assert_eq!(error_state, State::ErrorProcessing);

    let recovered = finish_with_error_handling(
        intermediate,
        transition,
        CallbackResult::Error,
        Some(CallbackResult::Success),
    )
    .unwrap();
    assert_eq!(recovered, State::Unconfigured);

    let fatal = finish_with_error_handling(
        intermediate,
        transition,
        CallbackResult::Error,
        Some(CallbackResult::Failure),
    )
    .unwrap();
    assert_eq!(fatal, State::Finalized);
}

#[test]
fn busy_state_rejection_is_deterministic() {
    let err = begin(State::Configuring, Transition::Activate).unwrap_err();
    assert_eq!(err.kind, ErrorKind::InvalidTransition);

    let transitional_states = [
        State::Configuring,
        State::CleaningUp,
        State::Activating,
        State::Deactivating,
        State::ShuttingDown,
        State::ErrorProcessing,
    ];

    for state in transitional_states {
        assert!(available_transitions(state).is_empty());
    }
}

#[test]
fn shutdown_from_primary_states_is_supported() {
    for state in [State::Unconfigured, State::Inactive, State::Active] {
        let intermediate = begin(state, Transition::Shutdown).unwrap();
        assert_eq!(intermediate, State::ShuttingDown);
        let final_state =
            finish(intermediate, Transition::Shutdown, CallbackResult::Success).unwrap();
        assert_eq!(final_state, State::Finalized);
    }
}
