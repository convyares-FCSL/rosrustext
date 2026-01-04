// crates/rosrustext_ros2_rust/src/lifecycle/node.rs
//
// Notes:
// - Keeps your “public API / internal API” split.
// - Fixes imports (no duplicate TransitionEvent, etc).
// - Keeps lifecycle entities enabled by default via try_new/try_with_gate.
// - Keeps bond plumbing in place, but (important) this file assumes BondAgent exists and compiles.
//   Your current build errors are in bond_agent.rs (crate name + QoS API). This node.rs won’t fix those.

use crate::error::Result;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use lifecycle_msgs::msg::{
    State as RosState, Transition as RosTransition, TransitionDescription, TransitionEvent,
};
use lifecycle_msgs::srv::{ChangeState, GetAvailableStates, GetAvailableTransitions, GetState};

use rclrs::{Node, TimerOptions};
use rosrustext_core::lifecycle::{ActivationGate, State};

#[cfg(feature = "bond")]
use super::BondAgent;
use super::{ManagedPublisher, ManagedTimer};

#[derive(Clone)]
pub struct LifecycleNode {
    node: Arc<Node>,
    gate: Arc<ActivationGate>,

    // Adapter-owned lifecycle state (until core machine drives it)
    state: Arc<Mutex<State>>,

    // transition_event publisher (lifecycle-owned)
    transition_event_pub: Arc<Mutex<Option<Arc<rclrs::Publisher<TransitionEvent>>>>>, // retained via internals too

    // bond agent (if any) (lifecycle-owned)
    #[cfg(feature = "bond")]
    bond: Arc<Mutex<Option<Arc<BondAgent>>>>,

    // Internal RAII retention for lifecycle-owned entities (services, pubs, etc.)
    internals: Arc<Mutex<Vec<Box<dyn std::any::Any + Send + Sync>>>>,
}

// ===== Public API =====
impl LifecycleNode {
    /// Preferred constructor: creates the lifecycle node and enables all lifecycle-facing entities.
    pub fn try_new(node: Arc<Node>) -> Result<Self> {
        let ln = Self {
            node,
            gate: Arc::new(ActivationGate::new()),
            state: Arc::new(Mutex::new(State::Unconfigured)),
            transition_event_pub: Arc::new(Mutex::new(None)),
            #[cfg(feature = "bond")]
            bond: Arc::new(Mutex::new(None)),
            internals: Arc::new(Mutex::new(Vec::new())),
        };

        ln.enable_defaults()?;
        Ok(ln)
    }

    /// For later slices: allow core-driven gate injection.
    pub fn try_with_gate(node: Arc<Node>, gate: Arc<ActivationGate>) -> Result<Self> {
        let ln = Self {
            node,
            gate,
            state: Arc::new(Mutex::new(State::Unconfigured)),
            transition_event_pub: Arc::new(Mutex::new(None)),
            #[cfg(feature = "bond")]
            bond: Arc::new(Mutex::new(None)),
            internals: Arc::new(Mutex::new(Vec::new())),
        };

        ln.enable_defaults()?;
        Ok(ln)
    }

    /// Access underlying rclrs::Node.
    pub fn node(&self) -> &Arc<Node> {
        &self.node
    }

    /// Set activation / deactivation gate state.
    pub fn activate(&self) {
        self.gate.activate();
    }
    pub fn deactivate(&self) {
        self.gate.deactivate();
    }

    /// Get activation gate state.
    pub fn is_active(&self) -> bool {
        self.gate.is_active()
    }

    /// Create a publisher managed by the lifecycle node's activation gate.
    pub fn create_publisher<T>(&self, topic: &str) -> Result<ManagedPublisher<T>>
    where
        T: rclrs::MessageIDL,
    {
        let pub_ = self.node.create_publisher::<T>(topic)?;
        Ok(ManagedPublisher::new(pub_, Arc::clone(&self.gate)))
    }

    /// Create a repeating timer managed by the lifecycle node's activation gate.
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
}

// ===== Internal / test-visible functions =====
impl LifecycleNode {
    /// Internal: enable all lifecycle ROS entities that must exist “because node exists”.
    pub(crate) fn enable_defaults(&self) -> Result<()> {
        // Order doesn’t matter much, but publishing transition_event is useful early.
        self.enable_transition_event_publisher()?;
        self.enable_get_state_service()?;
        self.enable_change_state_service()?;
        self.enable_get_available_states_service()?;
        self.enable_get_available_transitions_service()?;
        self.enable_bond()?;
        Ok(())
    }

    /// Enable lifecycle_msgs/msg/TransitionEvent publisher at "/<node>/transition_event".
    /// Lifecycle-owned: caller does NOT receive a handle.
    pub(crate) fn enable_transition_event_publisher(&self) -> Result<()> {
        let topic = format!("/{}/transition_event", self.node.name());
        let pub_ = Arc::new(self.node.create_publisher::<TransitionEvent>(&topic)?);

        *self
            .transition_event_pub
            .lock()
            .expect("transition_event_pub mutex poisoned") = Some(Arc::clone(&pub_));

        self.keep_internal(pub_);
        Ok(())
    }

    /// Enable lifecycle_msgs/srv/GetState at "/<node>/get_state".
    pub(crate) fn enable_get_state_service(&self) -> Result<()> {
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
    /// Minimal synchronous state update + gate update, respond immediately.
    /// Emits exactly one TransitionEvent per successful transition (if publisher enabled).
    pub(crate) fn enable_change_state_service(&self) -> Result<()> {
        let service_name = format!("/{}/change_state", self.node.name());

        let state = Arc::clone(&self.state);
        let gate = Arc::clone(&self.gate);
        let tev_pub = Arc::clone(&self.transition_event_pub);
        #[cfg(feature = "bond")]
        let bond = Arc::clone(&self.bond);

        let svc = self.node.create_service::<ChangeState, _>(
            &service_name,
            move |req: lifecycle_msgs::srv::ChangeState_Request| {
                let transition_id = req.transition.id;

                let mut state_guard = state.lock().expect("state mutex poisoned");
                let start = *state_guard;

                let mut success = false;

                if let Some((goal, label)) = Self::apply_primary_transition(start, transition_id) {
                    success = true;
                    *state_guard = goal;

                    // gate policy for this slice: only Active => gate on
                    match goal {
                        State::Active => gate.activate(),
                        _ => gate.deactivate(),
                    }

                    // bond follows "Active means bonded"
                    #[cfg(feature = "bond")]
                    if let Some(agent) = bond.lock().expect("bond mutex poisoned").as_ref() {
                        agent.set_active(goal == State::Active);
                    }

                    // Emit transition_event if enabled (exactly once per success)
                    if let Some(pub_) = tev_pub
                        .lock()
                        .expect("transition_event_pub mutex poisoned")
                        .as_ref()
                    {
                        let evt = Self::make_transition_event(start, goal, transition_id, label);
                        let _ = pub_.publish(evt);
                    }
                }

                let mut resp = lifecycle_msgs::srv::ChangeState_Response::default();
                resp.success = success;
                resp
            },
        )?;

        self.keep_internal(svc);
        Ok(())
    }

    /// Enable lifecycle_msgs/srv/GetAvailableTransitions at "/<node>/get_available_transitions".
    pub(crate) fn enable_get_available_transitions_service(&self) -> Result<()> {
        let service_name = format!("/{}/get_available_transitions", self.node.name());
        let state = Arc::clone(&self.state);

        let svc = self.node.create_service::<GetAvailableTransitions, _>(
            &service_name,
            move |_req: lifecycle_msgs::srv::GetAvailableTransitions_Request| {
                let current = *state.lock().expect("state mutex poisoned");

                let transitions: Vec<TransitionDescription> =
                    Self::available_primary_transitions(current)
                        .into_iter()
                        .map(|(id, goal, label)| Self::td(current, goal, id, label))
                        .collect();

                let mut resp = lifecycle_msgs::srv::GetAvailableTransitions_Response::default();
                resp.available_transitions = transitions;
                resp
            },
        )?;

        self.keep_internal(svc);
        Ok(())
    }

    /// Enable lifecycle_msgs/srv/GetAvailableStates at "/<node>/get_available_states".
    pub(crate) fn enable_get_available_states_service(&self) -> Result<()> {
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

    /// Enable bond management for this lifecycle node.
    #[cfg(feature = "bond")]
    pub(crate) fn enable_bond(&self) -> Result<()> {
        // match roslibrust proxy values you documented
        let heartbeat_period = Duration::from_secs(1);
        let heartbeat_timeout = Duration::from_secs(4);

        let agent = Arc::new(BondAgent::new(
            Arc::clone(&self.node),
            heartbeat_period,
            heartbeat_timeout,
        )?);

        // start inactive until state reaches Active
        agent.set_active(false);

        *self.bond.lock().expect("bond mutex poisoned") = Some(Arc::clone(&agent));
        self.keep_internal(agent);
        Ok(())
    }

    #[cfg(not(feature = "bond"))]
    pub(crate) fn enable_bond(&self) -> Result<()> {
        Ok(())
    }

    /// Apply primary transition to start State, returning goal State and label if valid.
    pub(crate) fn apply_primary_transition(
        start: State,
        transition_id: u8,
    ) -> Option<(State, &'static str)> {
        match (start, transition_id) {
            (State::Unconfigured, RosTransition::TRANSITION_CONFIGURE) => {
                Some((State::Inactive, "configure"))
            }
            (State::Unconfigured, RosTransition::TRANSITION_UNCONFIGURED_SHUTDOWN) => {
                Some((State::Finalized, "shutdown"))
            }

            (State::Inactive, RosTransition::TRANSITION_ACTIVATE) => {
                Some((State::Active, "activate"))
            }
            (State::Inactive, RosTransition::TRANSITION_CLEANUP) => {
                Some((State::Unconfigured, "cleanup"))
            }
            (State::Inactive, RosTransition::TRANSITION_INACTIVE_SHUTDOWN) => {
                Some((State::Finalized, "shutdown"))
            }

            (State::Active, RosTransition::TRANSITION_DEACTIVATE) => {
                Some((State::Inactive, "deactivate"))
            }
            (State::Active, RosTransition::TRANSITION_ACTIVE_SHUTDOWN) => {
                Some((State::Finalized, "shutdown"))
            }

            (State::Finalized, _) => None,
            _ => None,
        }
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
                (
                    RosTransition::TRANSITION_ACTIVATE,
                    State::Active,
                    "activate",
                ),
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

    /// Retain lifecycle-owned entities.
    pub(crate) fn keep_internal<T>(&self, handle: T)
    where
        T: Send + Sync + 'static,
    {
        self.internals
            .lock()
            .expect("LifecycleNode internals poisoned")
            .push(Box::new(handle));
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
        msg.id = Self::ros_primary_state_id(s);
        msg.label = format!("{:?}", s);
        msg
    }

    /// Create lifecycle_msgs/msg/TransitionDescription from primary transition data.
    fn td(start: State, goal: State, id: u8, label: &str) -> TransitionDescription {
        let mut t = RosTransition::default();
        t.id = id;
        t.label = label.to_string();

        let mut td = TransitionDescription::default();
        td.transition = t;
        td.start_state = Self::ros_state_msg(start);
        td.goal_state = Self::ros_state_msg(goal);
        td
    }

    /// Get current time in nanoseconds since UNIX_EPOCH.
    pub(crate) fn now_ns() -> u64 {
        // Jazzy TransitionEvent.timestamp is u64 nanoseconds (not builtin_interfaces/Time)
        (std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
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
        ev.timestamp = Self::now_ns();
        ev.transition = t;
        ev.start_state = Self::ros_state_msg(start);
        ev.goal_state = Self::ros_state_msg(goal);
        ev
    }
}
