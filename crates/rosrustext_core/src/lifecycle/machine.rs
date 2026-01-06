use crate::error::{CoreError, Result};
use super::{begin, finish_with_error_handling, goal_state_for_transition, CallbackResult, State, Transition};

/// Encapsulates the lifecycle state and ensures transitions follow the correct rules.
///
/// This struct owns the "source of truth" for the node's state.
/// It is transport-agnostic and does not know about ROS 2 service IDs or time.
#[derive(Debug)]
pub struct StateMachine {
    state: State,
    in_flight: Option<TransitionInFlight>,
}

/// Represents a transition that has started (`begin()`) but not yet completed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TransitionInFlight {
    pub start: State,
    pub transition: Transition,
    pub goal: State,
}

/// Input required to complete a transition.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CompleteInput {
    pub result: CallbackResult,
    /// Only required if `result` is `Error`.
    pub on_error_result: Option<CallbackResult>,
}

/// Outcome of a completed transition, carrying information needed for side effects.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CompleteOutcome {
    pub start_state: State,
    pub final_state: State,
    pub transition: Transition,
    pub gate_active: bool,
}

impl Default for StateMachine {
    fn default() -> Self {
        Self::new()
    }
}

impl StateMachine {
    pub fn new() -> Self {
        Self { state: State::Unconfigured, in_flight: None }
    }

    /// Get the current stable or intermediate state.
    pub fn current_state(&self) -> State {
        // If in-flight, technically we are in the intermediate state yielded by begin().
        // However, `self.state` acts as the "stable base" until completion updates it.
        // But wait, `begin()` returns the intermediate state.
        // Let's check `begin` implementation:
        // `begin(Unconfigured, Configure) -> Configuring`.
        // So `in_flight` implies we are effectively in the intermediate state.
        // But for simplicity, `self.state` holds the *source* state until completion?
        // Let's refine:
        // When `in_flight` is Some, we are "transitioning".
        // The *current* state of the system is the intermediate state.
        // Let's verify `begin` logic.
        
        if let Some(flight) = &self.in_flight {
             // Re-derive the intermediate state to be safe/pure?
             // Or store it. `begin` returns it.
             // Usually `begin` returns `intermediate`.
             // Ideally we should know it.
             // Let's use `begin` to re-derive purely.
             begin(flight.start, flight.transition).unwrap_or(flight.start)
        } else {
            self.state
        }
    }
    
    /// Get the "stable base" state. If transitioning, this is the state we started from.
    pub fn stable_state(&self) -> State {
        self.state
    }

    /// Attempt to begin a transition.
    ///
    /// Fails if:
    /// - Already in a transition (in_flight is Some).
    /// - Transition is invalid from current state.
    pub fn begin(&mut self, transition: Transition) -> Result<TransitionInFlight> {
        if self.in_flight.is_some() {
             // Already busy.
             // We can return a specific error or generic "InvalidTransition" implying state is busy.
             // Using existing error types.
             return Err(CoreError::invalid_transition_lifecycle(self.state.id(), transition.id()));
        }

        // Verify valid start->transition
        // This calculates the intermediate state but we mainly check validness via `goal_state_for_transition` logic implicitly?
        // Actually `begin` does the check.
        let _intermediate = begin(self.state, transition)?;
        
        // Calculate goal for metadata
        let goal = goal_state_for_transition(self.state, transition)?;

        let flight = TransitionInFlight {
            start: self.state,
            transition,
            goal,
        };

        self.in_flight = Some(flight);
        Ok(flight)
    }

    /// Complete the currently in-flight transition.
    ///
    /// Fails if:
    /// - No transition is in flight.
    pub fn complete(&mut self, input: CompleteInput) -> Result<CompleteOutcome> {
        let flight = self.in_flight.take().ok_or_else(|| {
             CoreError::warn()
                .domain(crate::error::Domain::Lifecycle)
                .kind(crate::error::ErrorKind::InvalidState)
                .msg("Called complete() but no transition is in flight")
                .build()
        })?;

        // Re-calculate intermediate state properly
        let intermediate = begin(flight.start, flight.transition)?;

        let final_state = finish_with_error_handling(
            intermediate,
            flight.transition,
            input.result,
            input.on_error_result,
        )?;

        self.state = final_state;
        
        let gate_active = self.state == State::Active;

        Ok(CompleteOutcome {
            start_state: flight.start,
            final_state,
            transition: flight.transition,
            gate_active,
        })
    }
}
