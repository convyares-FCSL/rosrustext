use std::collections::HashMap;

use rosrustext_core::error::{CoreError, Domain, ErrorKind, Payload, Result as CoreResult};
use rosrustext_core::lifecycle::{
    begin, goal_state_for_transition, State as CoreState, Transition,
};
use rosrustext_roslibrust::lifecycle::{ros_state_id, state_from_ros_id};

#[derive(Clone, Copy, Debug)]
pub struct EventInfo {
    pub goal_state_id: u8,
    pub stamp: u64,
}

#[derive(Clone, Copy, Debug)]
pub struct ChangePlan {
    pub transition_id: u8,
    pub start_state: CoreState,
    pub intermediate_state: CoreState,
    pub expected_goal_state_id: Option<u8>,
    pub last_stamp_before: u64,
}

#[derive(Clone, Copy, Debug)]
pub struct TimeoutOutcome {
    pub ok: bool,
    pub warned: bool,
}

pub struct ProxyLifecycle {
    state: CoreState,
    pending_goal: Option<CoreState>,
    event_map: HashMap<u8, EventInfo>,
}

impl ProxyLifecycle {
    pub fn new(initial: CoreState) -> Self {
        Self {
            state: initial,
            pending_goal: None,
            event_map: HashMap::new(),
        }
    }

    pub fn state(&self) -> CoreState {
        self.state
    }

    pub fn reported_state(&self) -> CoreState {
        self.pending_goal.unwrap_or(self.state)
    }

    pub fn pending_goal(&self) -> Option<CoreState> {
        self.pending_goal
    }

    pub fn begin_change(
        &mut self,
        transition: Transition,
        transition_id: u8,
    ) -> CoreResult<ChangePlan> {
        let start_state = match self.state {
            CoreState::Configuring
            | CoreState::CleaningUp
            | CoreState::Activating
            | CoreState::Deactivating
            | CoreState::ShuttingDown
            | CoreState::ErrorProcessing => self.pending_goal.unwrap_or(self.state),
            _ => self.state,
        };

        let intermediate_state = begin(start_state, transition)?;
        self.state = intermediate_state;

        let expected_goal_state = match goal_state_for_transition(start_state, transition) {
            Ok(state) => Some(state),
            Err(_) => {
                return Err(CoreError::warn()
                    .domain(Domain::Lifecycle)
                    .kind(ErrorKind::InvalidTransition)
                    .msg("change_state requested while local state is stale")
                    .payload(Payload::LifecycleTransition {
                        from_state: start_state.id(),
                        via_transition: transition.id(),
                    })
                    .build());
            }
        };

        let expected_goal_state_id = expected_goal_state.map(ros_state_id);
        if let Some(goal) = expected_goal_state {
            self.pending_goal = Some(goal);
        }

        let last_stamp_before = self
            .event_map
            .get(&transition_id)
            .map(|ev| ev.stamp)
            .unwrap_or(0);

        Ok(ChangePlan {
            transition_id,
            start_state,
            intermediate_state,
            expected_goal_state_id,
            last_stamp_before,
        })
    }

    pub fn record_event(
        &mut self,
        transition_id: u8,
        goal_state_id: u8,
        stamp: u64,
    ) -> CoreResult<CoreState> {
        self.event_map.insert(
            transition_id,
            EventInfo {
                goal_state_id,
                stamp,
            },
        );

        let next_state = state_from_ros_id(goal_state_id).ok_or_else(|| {
            CoreError::warn()
                .domain(Domain::Lifecycle)
                .kind(ErrorKind::InvalidState)
                .msg("unknown ROS lifecycle state id")
                .payload(Payload::Code(goal_state_id as u32))
                .build()
        })?;

        self.state = next_state;
        self.pending_goal = None;
        Ok(next_state)
    }

    pub fn confirm_or_timeout(&mut self, plan: ChangePlan) -> TimeoutOutcome {
        let snapshot = self.event_map.get(&plan.transition_id).copied();
        let ok = match snapshot {
            Some(ev) => {
                if ev.stamp == plan.last_stamp_before {
                    false
                } else {
                    match plan.expected_goal_state_id {
                        Some(expected) => ev.goal_state_id == expected,
                        None => true,
                    }
                }
            }
            None => false,
        };

        if !ok {
            if self.state == plan.intermediate_state {
                self.state = plan.start_state;
            }
            self.pending_goal = None;
        }

        TimeoutOutcome { ok, warned: !ok }
    }
}
