// crates/rosrustext_ros2_rust/src/lifecycle/node.rs
//
// Notes:
// - Keeps your “public API / internal API” split.
// - Fixes imports (no duplicate TransitionEvent, etc).
// - Keeps lifecycle entities enabled by default via try_new/try_with_gate.
// - Keeps bond plumbing in place, but (important) this file assumes BondAgent exists and compiles.
//   Your current build errors are in bond_agent.rs (crate name + QoS API). This node.rs won’t fix those.

use crate::error::Result;
use std::sync::{mpsc, Arc, Mutex};
use std::time::Duration;

use lifecycle_msgs::msg::{TransitionDescription, TransitionEvent};
use lifecycle_msgs::srv::{ChangeState, GetAvailableStates, GetAvailableTransitions, GetState};

use rclrs::{Node, TimerOptions};
use rosrustext_core::lifecycle::{
    begin, finish_with_error_handling, ActivationGate, CallbackResult, State, Transition,
};

#[cfg(feature = "transition_graph")]
use rosrustext_interfaces::srv::GetTransitionGraph;

#[cfg(feature = "bond")]
use super::BondAgent;
use super::{utils, ManagedPublisher, ManagedTimer};

#[derive(Clone)]
pub struct LifecycleNode {
    node: Arc<Node>,
    gate: Arc<ActivationGate>,

    // Adapter-owned lifecycle state (until core machine drives it)
    state: Arc<Mutex<LifecycleState>>,
    completion_tx: mpsc::Sender<TransitionOutcome>,
    completion_rx: Arc<Mutex<mpsc::Receiver<TransitionOutcome>>>,

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
        let (completion_tx, completion_rx) = mpsc::channel();
        let ln = Self {
            node,
            gate: Arc::new(ActivationGate::new()),
            state: Arc::new(Mutex::new(LifecycleState::new())),
            completion_tx,
            completion_rx: Arc::new(Mutex::new(completion_rx)),
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
        let (completion_tx, completion_rx) = mpsc::channel();
        let ln = Self {
            node,
            gate,
            state: Arc::new(Mutex::new(LifecycleState::new())),
            completion_tx,
            completion_rx: Arc::new(Mutex::new(completion_rx)),
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
    pub fn create_timer_repeating_gated<F>(&self, period: Duration, mut callback: F) -> Result<ManagedTimer>
    where
        F: FnMut() + Send + 'static,
    {
        let gate = Arc::clone(&self.gate);

        let timer = self.node.create_timer_repeating(TimerOptions::new(period), move || {
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
        self.enable_completion_pump()?;
        self.enable_get_available_states_service()?;
        self.enable_get_available_transitions_service()?;
        #[cfg(feature = "transition_graph")]
        self.enable_get_transition_graph_service()?;
        self.enable_bond()?;
        Ok(())
    }

    /// Enable lifecycle_msgs/msg/TransitionEvent publisher at "/<node>/transition_event".
    /// Lifecycle-owned: caller does NOT receive a handle.
    pub(crate) fn enable_transition_event_publisher(&self) -> Result<()> {
        let topic = format!("/{}/transition_event", self.node.name());
        let pub_ = Arc::new(self.node.create_publisher::<TransitionEvent>(&topic)?);

        *self.transition_event_pub.lock().expect("transition_event_pub mutex poisoned") = Some(Arc::clone(&pub_));

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
                let current = state.lock().expect("state mutex poisoned").state;

                let mut resp = lifecycle_msgs::srv::GetState_Response::default();
                resp.current_state = utils::ros_state_msg(current);
                resp
            },
        )?;

        self.keep_internal(svc);
        Ok(())
    }

    /// Enable lifecycle_msgs/srv/ChangeState at "/<node>/change_state".
    /// Accepts transitions immediately, marking them in-flight without completion.
    /// Transition completion is deferred (TODO: async execution + event emission).
    pub(crate) fn enable_change_state_service(&self) -> Result<()> {
        let service_name = format!("/{}/change_state", self.node.name());

        let state = Arc::clone(&self.state);
        let gate = Arc::clone(&self.gate);
        let tev_pub = Arc::clone(&self.transition_event_pub);
        #[cfg(feature = "bond")]
        let bond = Arc::clone(&self.bond);
        let completion_tx = self.completion_tx.clone();

        let svc = self.node.create_service::<ChangeState, _>(
            &service_name,
            move |req: lifecycle_msgs::srv::ChangeState_Request| {
                let transition_id = req.transition.id;

                let delay_ms = utils::change_state_delay_ms();
                let mut state_guard = state.lock().expect("state mutex poisoned");

                if state_guard.in_flight.is_some() {
                    let mut resp = lifecycle_msgs::srv::ChangeState_Response::default();
                    resp.success = false;
                    return resp;
                }

                let start = state_guard.state;
                let spec = match utils::transition_spec_for_ros_id(start, transition_id) {
                    Some(spec) => spec,
                    None => {
                        let mut resp = lifecycle_msgs::srv::ChangeState_Response::default();
                        resp.success = false;
                        return resp;
                    }
                };

                let intermediate = match begin(start, spec.transition) {
                    Ok(intermediate) => intermediate,
                    Err(_) => {
                        let mut resp = lifecycle_msgs::srv::ChangeState_Response::default();
                        resp.success = false;
                        return resp;
                    }
                };

                let in_flight = TransitionInFlight {
                    start,
                    intermediate,
                    transition: spec.transition,
                    transition_id,
                    label: spec.label,
                };
                state_guard.in_flight = Some(in_flight);
                drop(state_guard);

                if delay_ms == 0 {
                    let outcome = Self::compute_outcome(in_flight);
                    Self::apply_outcome(
                        &state,
                        &gate,
                        &tev_pub,
                        #[cfg(feature = "bond")]
                        &bond,
                        outcome,
                    );
                } else {
                    let completion_tx = completion_tx.clone();
                    std::thread::spawn(move || {
                        std::thread::sleep(Duration::from_millis(delay_ms));
                        let outcome = Self::compute_outcome(in_flight);
                        let _ = completion_tx.send(outcome);
                    });
                }

                let mut resp = lifecycle_msgs::srv::ChangeState_Response::default();
                resp.success = true;
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
                let guard = state.lock().expect("state mutex poisoned");
                let transitions: Vec<TransitionDescription> = if guard.in_flight.is_some() {
                    Vec::new()
                } else {
                    utils::transition_entries_for_start(guard.state)
                        .into_iter()
                        .map(|entry| {
                            utils::transition_description(
                                entry.spec.start,
                                entry.goal,
                                entry.spec.transition_id,
                                entry.spec.label,
                            )
                        })
                        .collect()
                };

                let mut resp = lifecycle_msgs::srv::GetAvailableTransitions_Response::default();
                resp.available_transitions = transitions;
                resp
            },
        )?;

        self.keep_internal(svc);
        Ok(())
    }

    /// Poll for transition outcomes and apply them in executor context.
    pub(crate) fn enable_completion_pump(&self) -> Result<()> {
        let state = Arc::clone(&self.state);
        let gate = Arc::clone(&self.gate);
        let tev_pub = Arc::clone(&self.transition_event_pub);
        let completion_rx = Arc::clone(&self.completion_rx);
        #[cfg(feature = "bond")]
        let bond = Arc::clone(&self.bond);

        let timer = self.node.create_timer_repeating(TimerOptions::new(Duration::from_millis(10)), move || {
            let outcomes: Vec<TransitionOutcome> = {
                let rx = completion_rx.lock().expect("completion_rx mutex poisoned");
                let mut pending = Vec::new();
                loop {
                    match rx.try_recv() {
                        Ok(outcome) => pending.push(outcome),
                        Err(mpsc::TryRecvError::Empty) => break,
                        Err(mpsc::TryRecvError::Disconnected) => break,
                    }
                }
                pending
            };

            for outcome in outcomes {
                Self::apply_outcome(
                    &state,
                    &gate,
                    &tev_pub,
                    #[cfg(feature = "bond")]
                    &bond,
                    outcome,
                );
            }
        })?;

        self.keep_internal(timer);
        Ok(())
    }

    /// Enable rosrustext_interfaces/srv/GetTransitionGraph at "/<node>/get_transition_graph".
    #[cfg(feature = "transition_graph")]
    pub(crate) fn enable_get_transition_graph_service(&self) -> Result<()> {
        let service_name = format!("/{}/get_transition_graph", self.node.name());

        let svc = self.node.create_service::<GetTransitionGraph, _>(
            &service_name,
            move |_req: rosrustext_interfaces::srv::GetTransitionGraph_Request| {
                let states = vec![
                    utils::ros_state_msg(State::Unconfigured),
                    utils::ros_state_msg(State::Inactive),
                    utils::ros_state_msg(State::Active),
                    utils::ros_state_msg(State::Finalized),
                ];

                let mut transitions = Vec::new();
                for start in [State::Unconfigured, State::Inactive, State::Active, State::Finalized] {
                    transitions.extend(
                        utils::transition_entries_for_start(start)
                            .into_iter()
                            .map(|entry| {
                                utils::transition_description(
                                    entry.spec.start,
                                    entry.goal,
                                    entry.spec.transition_id,
                                    entry.spec.label,
                                )
                            }),
                    );
                }

                let mut resp = rosrustext_interfaces::srv::GetTransitionGraph_Response::default();
                resp.states = states;
                resp.transitions = transitions;
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
                    utils::ros_state_msg(State::Unconfigured),
                    utils::ros_state_msg(State::Inactive),
                    utils::ros_state_msg(State::Active),
                    utils::ros_state_msg(State::Finalized),
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

        let agent = Arc::new(BondAgent::new(Arc::clone(&self.node), heartbeat_period, heartbeat_timeout)?);

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

    /// Retain lifecycle-owned entities.
    pub(crate) fn keep_internal<T>(&self, handle: T)
    where
        T: Send + Sync + 'static,
    {
        self.internals.lock().expect("LifecycleNode internals poisoned").push(Box::new(handle));
    }

    fn apply_outcome(
        state: &Arc<Mutex<LifecycleState>>, gate: &Arc<ActivationGate>,
        tev_pub: &Arc<Mutex<Option<Arc<rclrs::Publisher<TransitionEvent>>>>>,
        #[cfg(feature = "bond")] bond: &Arc<Mutex<Option<Arc<BondAgent>>>>, outcome: TransitionOutcome,
    ) {
        let mut state_guard = state.lock().expect("state mutex poisoned");
        let in_flight = match state_guard.in_flight {
            Some(in_flight) => in_flight,
            None => return,
        };
        if in_flight.transition_id != outcome.transition_id
            || in_flight.start != outcome.start
            || in_flight.transition != outcome.transition
        {
            return;
        }

        state_guard.state = outcome.goal;
        state_guard.in_flight = None;
        drop(state_guard);

        match outcome.goal {
            State::Active => gate.activate(),
            _ => gate.deactivate(),
        }

        #[cfg(feature = "bond")]
        if let Some(agent) = bond.lock().expect("bond mutex poisoned").as_ref() {
            agent.set_active(outcome.goal == State::Active);
        }

        if let Some(pub_) = tev_pub.lock().expect("transition_event_pub mutex poisoned").as_ref() {
            let evt = utils::make_transition_event(outcome.start, outcome.goal, outcome.transition_id, outcome.label);
            let _ = pub_.publish(evt);
        }
    }

    fn compute_outcome(in_flight: TransitionInFlight) -> TransitionOutcome {
        let result = utils::transition_result_for(in_flight.transition);
        let on_error =
            if result == CallbackResult::Error { Some(utils::on_error_result_for(in_flight.transition)) } else { None };
        let goal = finish_with_error_handling(in_flight.intermediate, in_flight.transition, result, on_error)
            .unwrap_or(in_flight.start);

        TransitionOutcome {
            start: in_flight.start,
            goal,
            transition: in_flight.transition,
            transition_id: in_flight.transition_id,
            label: in_flight.label,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct TransitionInFlight {
    start: State,
    intermediate: State,
    transition: Transition,
    transition_id: u8,
    label: &'static str,
}

#[derive(Debug, Clone, Copy)]
struct TransitionOutcome {
    start: State,
    goal: State,
    transition: Transition,
    transition_id: u8,
    label: &'static str,
}

#[derive(Debug)]
struct LifecycleState {
    state: State,
    in_flight: Option<TransitionInFlight>,
}

impl LifecycleState {
    fn new() -> Self {
        Self { state: State::Unconfigured, in_flight: None }
    }
}
