// crates/rosrustext_rosrs/src/lifecycle/node.rs
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

use rosrustext_msgs::lifecycle_msgs::msg::{TransitionDescription, TransitionEvent};
use rosrustext_msgs::lifecycle_msgs::srv::{ChangeState, GetAvailableStates, GetAvailableTransitions, GetState};

use rclrs::{
    Client, ClientOptions, Executor, IntoNodeOptions, IntoNodeServiceCallback, IntoNodeSubscriptionCallback, Node,
    Service, ServiceOptions, Subscription, SubscriptionOptions, TimerOptions,
};
use rosrustext_core::lifecycle::{
    begin, finish_with_error_handling, ActivationGate, CallbackResult, CompleteInput, CompleteOutcome,
    LifecycleCallbacks, State, StateMachine, Transition, TransitionInFlight,
};

#[cfg(feature = "transition_graph")]
use rosrustext_msgs::rosrustext_interfaces::srv::GetTransitionGraph;

#[cfg(feature = "bond")]
use super::BondAgent;
use super::{utils, ManagedPublisher, ManagedTimer};

/// Lifecycle-aware node wrapper.
///
/// This intentionally does **not** implement `Deref<Target = Node>` so lifecycle-aware
/// publishing and timers cannot be bypassed accidentally. Use `node_arc()` only
/// as an explicit escape hatch.
#[derive(Clone)]
pub struct LifecycleNode {
    node: Arc<Node>,
    gate: Arc<ActivationGate>,

    // Core state machine ownership
    machine: Arc<Mutex<StateMachine>>,
    
    // User callbacks
    callbacks: Arc<Mutex<Box<dyn LifecycleCallbacks + Send>>>,

    completion_tx: mpsc::Sender<AsyncOutcome>,
    completion_rx: Arc<Mutex<mpsc::Receiver<AsyncOutcome>>>,

    // transition_event publisher (lifecycle-owned)
    transition_event_pub: Arc<Mutex<Option<Arc<rclrs::Publisher<TransitionEvent>>>>>, // retained via internals too

    // bond agent (if any) (lifecycle-owned)
    #[cfg(feature = "bond")]
    bond: Arc<Mutex<Option<Arc<BondAgent>>>>,

    // Internal RAII retention for lifecycle-owned entities (services, pubs, etc.)
    internals: Arc<Mutex<Vec<Box<dyn std::any::Any + Send + Sync>>>>,
}

struct DefaultCallbacks;
impl LifecycleCallbacks for DefaultCallbacks {
    fn on_configure(&mut self) -> CallbackResult { CallbackResult::Success }
    fn on_activate(&mut self) -> CallbackResult { CallbackResult::Success }
    fn on_deactivate(&mut self) -> CallbackResult { CallbackResult::Success }
    fn on_cleanup(&mut self) -> CallbackResult { CallbackResult::Success }
    fn on_shutdown(&mut self) -> CallbackResult { CallbackResult::Success }
    fn on_error(&mut self) -> CallbackResult { CallbackResult::Success }
}

// ===== Public API =====
impl LifecycleNode {
    /// Primary constructor: create a node on the executor and enable lifecycle semantics.
    pub fn create<'a>(executor: &'a Executor, options: impl IntoNodeOptions<'a>) -> Result<Self> {
        let node = Arc::new(executor.create_node(options)?);
        Self::try_new(node)
    }

    /// Advanced constructor: wraps an existing node with lifecycle semantics.
    pub fn try_new(node: Arc<Node>) -> Result<Self> {
        let (completion_tx, completion_rx) = mpsc::channel();
        let ln = Self {
            node,
            gate: Arc::new(ActivationGate::new()),
            machine: Arc::new(Mutex::new(StateMachine::new())),
            callbacks: Arc::new(Mutex::new(Box::new(DefaultCallbacks))),
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
    
    /// Create with specific callbacks.
    pub fn new_with_callbacks(node: Arc<Node>, callbacks: Box<dyn LifecycleCallbacks + Send>) -> Result<Self> {
        let (completion_tx, completion_rx) = mpsc::channel();
        let ln = Self {
            node,
            gate: Arc::new(ActivationGate::new()),
            machine: Arc::new(Mutex::new(StateMachine::new())),
            callbacks: Arc::new(Mutex::new(callbacks)),
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

    /// Advanced constructor alias for `try_new`.
    pub fn from_node(node: Arc<Node>) -> Result<Self> {
        Self::try_new(node)
    }

    /// For later slices: allow core-driven gate injection.
    pub fn try_with_gate(node: Arc<Node>, gate: Arc<ActivationGate>) -> Result<Self> {
        let (completion_tx, completion_rx) = mpsc::channel();
        let ln = Self {
            node,
            gate,
            machine: Arc::new(Mutex::new(StateMachine::new())),
            callbacks: Arc::new(Mutex::new(Box::new(DefaultCallbacks))),
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

    /// Escape hatch: access the underlying rclrs::Node.
    pub fn node(&self) -> &Arc<Node> {
        &self.node
    }

    /// Escape hatch: clone the underlying rclrs::Node.
    pub fn node_arc(&self) -> Arc<Node> {
        Arc::clone(&self.node)
    }

    /// Node name after ROS remapping.
    pub fn name(&self) -> String {
        self.node.name()
    }

    /// Node namespace after ROS remapping.
    pub fn namespace(&self) -> String {
        self.node.namespace()
    }

    /// Create a non-lifecycle service on the underlying node.
    pub fn create_service<'a, T, Args>(
        &self, options: impl Into<ServiceOptions<'a>>, callback: impl IntoNodeServiceCallback<T, Args>,
    ) -> Result<Service<T>>
    where
        T: rclrs::ServiceIDL,
    {
        Ok(self.node.create_service::<T, Args>(options, callback)?)
    }

    /// Create a non-lifecycle subscription on the underlying node.
    pub fn create_subscription<'a, T, Args>(
        &self, options: impl Into<SubscriptionOptions<'a>>, callback: impl IntoNodeSubscriptionCallback<T, Args>,
    ) -> Result<Subscription<T>>
    where
        T: rclrs::MessageIDL,
    {
        Ok(self.node.create_subscription::<T, Args>(options, callback)?)
    }

    /// Create a client on the underlying node.
    pub fn create_client<'a, T>(&self, options: impl Into<ClientOptions<'a>>) -> Result<Client<T>>
    where
        T: rclrs::ServiceIDL,
    {
        Ok(self.node.create_client::<T>(options)?)
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

    /// Internal helper: Apply completion outcome to state machine and trigger side effects.
    fn handle_completion_outcome(&self, outcome: AsyncOutcome) {
         let mut machine_guard = self.machine.lock().expect("machine mutex poisoned");
         // We rely on machine valid state.
         let result = match machine_guard.complete(outcome.input) {
             Ok(res) => res,
             Err(_e) => {
                 // Log error?
                 // "Called complete() but no transition is in flight" -> ignore
                 return;
             }
         };
         drop(machine_guard);
         
         // Side effects
          if result.gate_active {
             self.gate.activate();
         } else {
             self.gate.deactivate();
         }

         #[cfg(feature = "bond")]
         if let Some(agent) = self.bond.lock().expect("bond mutex poisoned").as_ref() {
             agent.set_active(result.gate_active);
         }

         if let Some(pub_) = self.transition_event_pub.lock().expect("transition_event_pub mutex poisoned").as_ref() {
             let evt = utils::make_transition_event(result.start_state, result.final_state, outcome.transition_id, outcome.label);
             let _ = pub_.publish(evt);
         }
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
        let machine = Arc::clone(&self.machine);

        let svc = self.node.create_service::<GetState, _>(
            &service_name,
            move |_req: rosrustext_msgs::lifecycle_msgs::srv::GetState_Request| {
                let current = machine.lock().expect("machine mutex poisoned").current_state();

                rosrustext_msgs::lifecycle_msgs::srv::GetState_Response { current_state: utils::ros_state_msg(current) }
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

        let machine = Arc::clone(&self.machine);
        let callbacks = Arc::clone(&self.callbacks);
        let completion_tx = self.completion_tx.clone();
        let active_self = self.clone();

        let svc = self.node.create_service::<ChangeState, _>(
            &service_name,
            move |req: rosrustext_msgs::lifecycle_msgs::srv::ChangeState_Request| {
                let transition_id = req.transition.id;

                let delay_ms = utils::change_state_delay_ms();
                let mut machine_guard = machine.lock().expect("machine mutex poisoned");

                // Map ROS ID to core transition spec
                let start_state = machine_guard.stable_state();
                let spec = match utils::transition_spec_for_ros_id(start_state, transition_id) {
                    Some(spec) => spec,
                    None => return rosrustext_msgs::lifecycle_msgs::srv::ChangeState_Response { success: false },
                };

                let flight = match machine_guard.begin(spec.transition) {
                    Ok(flight) => flight,
                    Err(_) => return rosrustext_msgs::lifecycle_msgs::srv::ChangeState_Response { success: false },
                };

                
                // Store metadata for the async worker
                let label = spec.label;
                let transition_id = spec.transition_id;
                
                drop(machine_guard);

                // 2. Spawn worker (or run inline if delay=0)
                // Note: We need to run the callbacks.
                // We clone the callbacks Arc.
                let callbacks_clone = callbacks.clone();
                let completion_tx_clone = completion_tx.clone();
                
                // Closure to execute the user callback
                let do_callback = move || -> CompleteInput {
                    if delay_ms > 0 {
                        std::thread::sleep(Duration::from_millis(delay_ms));
                    }
                    
                    let mut cb_guard = callbacks_clone.lock().expect("callbacks mutex poisoned");
                    let result = match flight.transition {
                         Transition::Configure => cb_guard.on_configure(),
                         Transition::Activate => cb_guard.on_activate(),
                         Transition::Deactivate => cb_guard.on_deactivate(),
                         Transition::Cleanup => cb_guard.on_cleanup(),
                         Transition::Shutdown => cb_guard.on_shutdown(),
                    };
                    
                    let on_error_result = if result == CallbackResult::Error {
                        Some(cb_guard.on_error())
                    } else {
                        None
                    };
                    drop(cb_guard);
                    
                    CompleteInput { result, on_error_result }
                };

                if delay_ms == 0 {
                    // SYNCHRONOUS PATH: Run callback inline, complete immediately.
                    // This ensures that when the service returns, the state is updated.
                    // This blocks the executor thread for the duration of the callback.
                    let input = do_callback();
                    let active_self_clone = active_self.clone();
                    active_self_clone.handle_completion_outcome(AsyncOutcome { input, label, transition_id });
                } else {
                    // ASYNC PATH: Spawn thread, send result to pump.
                    std::thread::spawn(move || {
                        let input = do_callback();
                        let _ = completion_tx_clone.send(AsyncOutcome { input, label, transition_id });
                    });
                }

                rosrustext_msgs::lifecycle_msgs::srv::ChangeState_Response { success: true }
            },
        )?;

        self.keep_internal(svc);
        Ok(())
    }

    /// Enable lifecycle_msgs/srv/GetAvailableTransitions at "/<node>/get_available_transitions".
    pub(crate) fn enable_get_available_transitions_service(&self) -> Result<()> {
        let service_name = format!("/{}/get_available_transitions", self.node.name());
        let machine = Arc::clone(&self.machine);

        let svc = self.node.create_service::<GetAvailableTransitions, _>(
            &service_name,
            move |_req: rosrustext_msgs::lifecycle_msgs::srv::GetAvailableTransitions_Request| {
                let guard = machine.lock().expect("machine mutex poisoned");
                // If transitioning, no transitions available (ROS 2 behavior generally)
                // machine.current_state() handles in-flight logic?
                // Actually `begin` rejects if in-flight. 
                // We should expose `is_in_flight` or similar? 
                // Or just try?
                // `available_transitions` utility takes `State`.
                let current = guard.current_state();
                // If we are in a transition state (Configuring etc), available_transitions returns empty.
                // So this works naturally if `current_state()` returns the intermediate state.
                
                let transitions: Vec<TransitionDescription> = utils::transition_entries_for_start(current)
                        .into_iter()
                        .map(|entry| {
                            utils::transition_description(
                                entry.spec.start,
                                entry.goal,
                                entry.spec.transition_id,
                                entry.spec.label,
                            )
                        })
                        .collect();

                rosrustext_msgs::lifecycle_msgs::srv::GetAvailableTransitions_Response { available_transitions: transitions }
            },
        )?;

        self.keep_internal(svc);
        Ok(())
    }

    /// Poll for transition outcomes and apply them in executor context.
    pub(crate) fn enable_completion_pump(&self) -> Result<()> {
        let completion_rx = Arc::clone(&self.completion_rx);
        // We clone self to move into closure, but self is Clone (Arc wrapper).
        // Actually self is Clone, so we can just move self.clone() into closure?
        // Wait, self.handle_completion_outcome requires &self.
        // Yes, self is cheap clone.
        let ln = self.clone();

        let timer = self.node.create_timer_repeating(TimerOptions::new(Duration::from_millis(10)), move || {
            let outcomes: Vec<AsyncOutcome> = {
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
                ln.handle_completion_outcome(outcome);
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
            move |_req: rosrustext_msgs::rosrustext_interfaces::srv::GetTransitionGraph_Request| {
                let states = vec![
                    utils::ros_state_msg(State::Unconfigured),
                    utils::ros_state_msg(State::Inactive),
                    utils::ros_state_msg(State::Active),
                    utils::ros_state_msg(State::Finalized),
                ];

                let mut transitions = Vec::new();
                for start in [State::Unconfigured, State::Inactive, State::Active, State::Finalized] {
                    transitions.extend(utils::transition_entries_for_start(start).into_iter().map(|entry| {
                        utils::transition_description(
                            entry.spec.start,
                            entry.goal,
                            entry.spec.transition_id,
                            entry.spec.label,
                        )
                    }));
                }

                rosrustext_msgs::rosrustext_interfaces::srv::GetTransitionGraph_Response { states, transitions }
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
            move |_req: rosrustext_msgs::lifecycle_msgs::srv::GetAvailableStates_Request| {
                let states = vec![
                    utils::ros_state_msg(State::Unconfigured),
                    utils::ros_state_msg(State::Inactive),
                    utils::ros_state_msg(State::Active),
                    utils::ros_state_msg(State::Finalized),
                ];

                rosrustext_msgs::lifecycle_msgs::srv::GetAvailableStates_Response { available_states: states }
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
}

// Data passed from worker thread to completion pump
#[derive(Debug, Clone, Copy)]
struct AsyncOutcome {
    input: CompleteInput,
    label: &'static str,
    transition_id: u8,
}
