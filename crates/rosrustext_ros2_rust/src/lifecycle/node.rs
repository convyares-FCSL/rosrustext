use std::sync::{Arc, Mutex};
use std::time::Duration;

use rclrs::{Node, TimerOptions};

use crate::error::Result;
use rosrustext_core::lifecycle::{ActivationGate, State};

use lifecycle_msgs::msg::{Transition, TransitionDescription};
use lifecycle_msgs::srv::{ChangeState, GetAvailableStates, GetAvailableTransitions, GetState};

use super::{ManagedPublisher, ManagedTimer};

#[derive(Clone)]
pub struct LifecycleNode {
    node: Arc<Node>,
    gate: Arc<ActivationGate>,

    // Adapter-owned lifecycle state (Slice-3; replaced by core machine later)
    state: Arc<Mutex<State>>,

    // Internal RAII retention for lifecycle-owned entities (services, etc.)
    internals: Arc<Mutex<Vec<Box<dyn std::any::Any + Send + Sync>>>>,
}

impl LifecycleNode {
    pub fn new(node: Arc<Node>) -> Self {
        Self {
            node,
            gate: Arc::new(ActivationGate::new()),
            state: Arc::new(Mutex::new(State::Unconfigured)),
            internals: Arc::new(Mutex::new(Vec::new())),
        }
    }

    /// For later slices: allow core-driven gate injection.
    pub fn with_gate(node: Arc<Node>, gate: Arc<ActivationGate>) -> Self {
        Self {
            node,
            gate,
            state: Arc::new(Mutex::new(State::Unconfigured)),
            internals: Arc::new(Mutex::new(Vec::new())),
        }
    }

    pub fn node(&self) -> &Arc<Node> {
        &self.node
    }

    /// Slice-1 compatibility: manual toggling.
    pub fn set_active(&self, active: bool) {
        if active {
            self.gate.activate();
        } else {
            self.gate.deactivate();
        }
    }

    pub fn is_active(&self) -> bool {
        self.gate.is_active()
    }

    pub fn create_publisher<T>(&self, topic: &str) -> Result<ManagedPublisher<T>>
    where
        T: rclrs::MessageIDL,
    {
        let pub_ = self.node.create_publisher::<T>(topic)?;
        Ok(ManagedPublisher::new(pub_, Arc::clone(&self.gate)))
    }

    pub fn create_timer_repeating_gated<F>(
        &self,
        period: Duration,
        mut callback: F,
    ) -> Result<ManagedTimer>
    where
        F: FnMut() + Send + 'static,
    {
        let gate = Arc::clone(&self.gate);

        let timer = self
            .node
            .create_timer_repeating(TimerOptions::new(period), move || {
                if gate.is_active() {
                    callback();
                }
            })?;

        Ok(ManagedTimer::new(timer))
    }

    /// Enable lifecycle_msgs/srv/GetState at "/<node>/get_state".
    ///
    /// Lifecycle-owned: caller does NOT receive a handle.
    pub fn enable_get_state_service(&self) -> Result<()> {
        let service_name = format!("/{}/get_state", self.node.name());
        let state = Arc::clone(&self.state);

        let svc = self.node.create_service::<GetState, _>(
            &service_name,
            move |_req: lifecycle_msgs::srv::GetState_Request| {
                let current = *state.lock().expect("state mutex poisoned");

                let mut resp = lifecycle_msgs::srv::GetState_Response::default();
                resp.current_state = Self::ros_state_msg(current);
                resp
            },
        )?;

        self.keep_internal(svc);
        Ok(())
    }

    /// Enable lifecycle_msgs/srv/ChangeState at "/<node>/change_state".
    ///
    /// Slice-3 minimal: synchronous state update + gate update, respond immediately.
    pub fn enable_change_state_service(&self) -> Result<()> {
        let service_name = format!("/{}/change_state", self.node.name());

        let state = Arc::clone(&self.state);
        let gate = Arc::clone(&self.gate);

        let svc = self.node.create_service::<ChangeState, _>(
            &service_name,
            move |req: lifecycle_msgs::srv::ChangeState_Request| {
                let transition_id = req.transition.id;
                let mut state_guard = state.lock().expect("state mutex poisoned");

                let success = match transition_id {
                    Transition::TRANSITION_CONFIGURE => {
                        *state_guard = State::Inactive;
                        gate.deactivate();
                        true
                    }
                    Transition::TRANSITION_ACTIVATE => {
                        *state_guard = State::Active;
                        gate.activate();
                        true
                    }
                    Transition::TRANSITION_DEACTIVATE => {
                        *state_guard = State::Inactive;
                        gate.deactivate();
                        true
                    }
                    Transition::TRANSITION_CLEANUP => {
                        *state_guard = State::Unconfigured;
                        gate.deactivate();
                        true
                    }
                    Transition::TRANSITION_UNCONFIGURED_SHUTDOWN
                    | Transition::TRANSITION_INACTIVE_SHUTDOWN
                    | Transition::TRANSITION_ACTIVE_SHUTDOWN => {
                        *state_guard = State::Finalized;
                        gate.deactivate();
                        true
                    }
                    _ => false,
                };

                let mut resp = lifecycle_msgs::srv::ChangeState_Response::default();
                resp.success = success;
                resp
            },
        )?;

        self.keep_internal(svc);
        Ok(())
    }

    /// Enable lifecycle_msgs/srv/GetAvailableTransitions at "/<node>/get_available_transitions".
    pub fn enable_get_available_transitions_service(&self) -> Result<()> {
        let service_name = format!("/{}/get_available_transitions", self.node.name());
        let state = Arc::clone(&self.state);

        let svc = self.node.create_service::<GetAvailableTransitions, _>(
            &service_name,
            move |_req: lifecycle_msgs::srv::GetAvailableTransitions_Request| {
                let current = *state.lock().expect("state mutex poisoned");

                let transitions: Vec<TransitionDescription> = match current {
                    State::Unconfigured => vec![
                        Self::td(
                            State::Unconfigured,
                            State::Inactive,
                            Transition::TRANSITION_CONFIGURE,
                            "configure",
                        ),
                        Self::td(
                            State::Unconfigured,
                            State::Finalized,
                            Transition::TRANSITION_UNCONFIGURED_SHUTDOWN,
                            "shutdown",
                        ),
                    ],
                    State::Inactive => vec![
                        Self::td(
                            State::Inactive,
                            State::Active,
                            Transition::TRANSITION_ACTIVATE,
                            "activate",
                        ),
                        Self::td(
                            State::Inactive,
                            State::Unconfigured,
                            Transition::TRANSITION_CLEANUP,
                            "cleanup",
                        ),
                        Self::td(
                            State::Inactive,
                            State::Finalized,
                            Transition::TRANSITION_INACTIVE_SHUTDOWN,
                            "shutdown",
                        ),
                    ],
                    State::Active => vec![
                        Self::td(
                            State::Active,
                            State::Inactive,
                            Transition::TRANSITION_DEACTIVATE,
                            "deactivate",
                        ),
                        Self::td(
                            State::Active,
                            State::Finalized,
                            Transition::TRANSITION_ACTIVE_SHUTDOWN,
                            "shutdown",
                        ),
                    ],
                    State::Finalized => vec![],
                    _ => vec![],
                };

                let mut resp = lifecycle_msgs::srv::GetAvailableTransitions_Response::default();
                resp.available_transitions = transitions;
                resp
            },
        )?;

        self.keep_internal(svc);
        Ok(())
    }

    /// Enable lifecycle_msgs/srv/GetAvailableStates at "/<node>/get_available_states".
    pub fn enable_get_available_states_service(&self) -> Result<()> {
        let service_name = format!("/{}/get_available_states", self.node.name());

        let svc = self.node.create_service::<GetAvailableStates, _>(
            &service_name,
            move |_req: lifecycle_msgs::srv::GetAvailableStates_Request| {
                let states = vec![
                    Self::ros_state_msg(State::Unconfigured),
                    Self::ros_state_msg(State::Inactive),
                    Self::ros_state_msg(State::Active),
                    Self::ros_state_msg(State::Finalized),
                ];

                let mut resp = lifecycle_msgs::srv::GetAvailableStates_Response::default();
                resp.available_states = states;
                resp
            },
        )?;

        self.keep_internal(svc);
        Ok(())
    }

    fn keep_internal<T>(&self, handle: T)
    where
        T: Send + Sync + 'static,
    {
        self.internals
            .lock()
            .expect("LifecycleNode internals poisoned")
            .push(Box::new(handle));
    }

    fn ros_primary_state_id(s: State) -> u8 {
        // ROS2 primary state IDs (lifecycle_msgs / rclcpp)
        match s {
            State::Unconfigured => 1,
            State::Inactive => 2,
            State::Active => 3,
            State::Finalized => 4,
            _ => 0,
        }
    }

    fn ros_state_msg(s: State) -> lifecycle_msgs::msg::State {
        let mut msg = lifecycle_msgs::msg::State::default();
        msg.id = Self::ros_primary_state_id(s);
        msg.label = format!("{:?}", s);
        msg
    }

    fn td(start: State, goal: State, id: u8, label: &str) -> TransitionDescription {
        let mut t = lifecycle_msgs::msg::Transition::default();
        t.id = id;
        t.label = label.to_string();

        let mut td = TransitionDescription::default();
        td.transition = t;
        td.start_state = Self::ros_state_msg(start);
        td.goal_state = Self::ros_state_msg(goal);
        td
    }
}
