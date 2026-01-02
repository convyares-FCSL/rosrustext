use crate::error::Result;

use super::{available_transitions, goal_state_for_transition, State, Transition, ALL_STATES};

/// Lifecycle transition graph derived from core state/transition tables.
#[derive(Debug, Clone, Eq, PartialEq)]
pub struct TransitionGraph {
    pub states: Vec<State>,
    pub transitions: Vec<TransitionEdge>,
}

/// Directed lifecycle transition edge.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct TransitionEdge {
    pub start: State,
    pub transition: Transition,
    pub goal: State,
}

/// Build the canonical lifecycle transition graph.
pub fn transition_graph() -> Result<TransitionGraph> {
    let mut transitions = Vec::new();

    for state in ALL_STATES {
        for transition in available_transitions(state) {
            let goal = goal_state_for_transition(state, *transition)?;
            transitions.push(TransitionEdge {
                start: state,
                transition: *transition,
                goal,
            });
        }
    }

    Ok(TransitionGraph {
        states: ALL_STATES.to_vec(),
        transitions,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn graph_contains_all_states_and_expected_edges() {
        let graph = transition_graph().unwrap();

        assert_eq!(graph.states.len(), ALL_STATES.len());

        let expected = [
            (State::Unconfigured, Transition::Configure, State::Inactive),
            (State::Unconfigured, Transition::Shutdown, State::Finalized),
            (State::Inactive, Transition::Activate, State::Active),
            (State::Inactive, Transition::Cleanup, State::Unconfigured),
            (State::Inactive, Transition::Shutdown, State::Finalized),
            (State::Active, Transition::Deactivate, State::Inactive),
            (State::Active, Transition::Shutdown, State::Finalized),
        ];

        for (start, transition, goal) in expected {
            assert!(
                graph.transitions.iter().any(|edge| {
                    edge.start == start && edge.transition == transition && edge.goal == goal
                }),
                "missing edge {start:?} -> {transition:?} -> {goal:?}"
            );
        }

        assert_eq!(graph.transitions.len(), expected.len());
    }
}
