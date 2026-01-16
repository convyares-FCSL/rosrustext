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
use rosrustext_core::lifecycle::{ActivationGate, CallbackResult, CompleteInput, State, StateMachine, Transition};

#[cfg(feature = "transition_graph")]
use rosrustext_msgs::rosrustext_interfaces::srv::GetTransitionGraph;

#[cfg(feature = "bond")]
use super::BondAgent;
use super::{utils, ManagedPublisher, ManagedTimer};

/// Lifecycle transition callbacks that receive a [`LifecycleNode`] handle.
///
/// # Semantics
/// - One `on_*` method is called for each *accepted* `change_state` request.
/// - `state` is the stable lifecycle state *before* the transition begins.
/// - Returning [`CallbackResult::Success`] completes the transition to the expected
///   goal state.
/// - Returning [`CallbackResult::Failure`] completes the transition back to the
///   pre-transition stable state (ROS 2 semantics).
/// - Returning [`CallbackResult::Error`] enters `ErrorProcessing` and then calls
///   [`Self::on_error`] to determine recovery (`Success` → `Unconfigured`,
///   `Failure`/`Error` → `Finalized`).
///
/// Threading:
/// - With `ROSRUSTEXT_RCLRS_CHANGE_STATE_DELAY_MS=0` (default), callbacks run on
///   the executor thread inside the `change_state` service handler.
/// - With `ROSRUSTEXT_RCLRS_CHANGE_STATE_DELAY_MS>0`, callbacks run on a spawned
///   OS thread.
///
/// # Errors
/// Callback methods do not return `Result`. Map recoverable failures to
/// [`CallbackResult::Failure`] / [`CallbackResult::Error`]. Panicking inside a
/// callback is not handled by the adapter and will unwind through the service
/// handler / worker thread.
///
/// # Example
/// ```rust,no_run
/// use rosrustext_rosrs::lifecycle::{CallbackResult, LifecycleCallbacksWithNode, LifecycleNode};
/// use rosrustext_rosrs::State;
///
/// struct Callbacks;
///
/// impl LifecycleCallbacksWithNode for Callbacks {
///     fn on_configure(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult {
///         CallbackResult::Success
///     }
///     fn on_activate(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult {
///         CallbackResult::Success
///     }
///     fn on_deactivate(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult {
///         CallbackResult::Success
///     }
///     fn on_cleanup(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult {
///         CallbackResult::Success
///     }
///     fn on_shutdown(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult {
///         CallbackResult::Success
///     }
///     fn on_error(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult {
///         CallbackResult::Success
///     }
/// }
/// ```
///
/// # See also
/// - [Lifecycle spec](https://github.com/convyares-FCSL/rosrustext/blob/main/docs/spec/lifecycle.md)
/// - [`LifecycleNode`]
pub trait LifecycleCallbacksWithNode {
    fn on_configure(&mut self, node: &LifecycleNode, state: &State) -> CallbackResult;
    fn on_activate(&mut self, node: &LifecycleNode, state: &State) -> CallbackResult;
    fn on_deactivate(&mut self, node: &LifecycleNode, state: &State) -> CallbackResult;
    fn on_cleanup(&mut self, node: &LifecycleNode, state: &State) -> CallbackResult;
    fn on_shutdown(&mut self, node: &LifecycleNode, state: &State) -> CallbackResult;

    /// Called when a transition callback reports `CallbackResult::Error`.
    fn on_error(&mut self, node: &LifecycleNode, state: &State) -> CallbackResult;
}

fn run_transition_callback(
    callbacks: &mut dyn LifecycleCallbacksWithNode, transition: Transition, node: &LifecycleNode, state: State,
) -> CallbackResult {
    match transition {
        Transition::Configure => callbacks.on_configure(node, &state),
        Transition::Activate => callbacks.on_activate(node, &state),
        Transition::Deactivate => callbacks.on_deactivate(node, &state),
        Transition::Cleanup => callbacks.on_cleanup(node, &state),
        Transition::Shutdown => callbacks.on_shutdown(node, &state),
    }
}

/// ROS-facing lifecycle wrapper around an `rclrs::Node`.
///
/// # Semantics
/// - Owns a transport-agnostic lifecycle state machine (`rosrustext_core`).
/// - Exposes ROS lifecycle services on `/<node>/…` and publishes
///   `/<node>/transition_event`.
/// - Maintains an internal [`ActivationGate`] that becomes active when the stable
///   state is `Active`.
/// - Provides managed resources ([`ManagedPublisher`], [`ManagedTimer`]) that are
///   suppressed while inactive.
///
/// This intentionally does **not** implement `Deref<Target = Node>` so lifecycle-aware
/// publishing and timers cannot be bypassed accidentally. Use [`LifecycleNode::node_arc`]
/// only as an explicit escape hatch.
///
/// # Errors
/// Constructors return [`crate::Error`] when:
/// - `rclrs` cannot create the node, services, publishers, or timers, or
/// - lifecycle entities already exist on the node (name collisions).
///
/// # Example
/// ```rust,no_run
/// use rclrs::{Context, CreateBasicExecutor, SpinOptions};
/// use rosrustext_rosrs::lifecycle::{CallbackResult, LifecycleCallbacksWithNode, LifecycleNode};
/// use rosrustext_rosrs::State;
///
/// struct Callbacks;
/// impl LifecycleCallbacksWithNode for Callbacks {
///     fn on_configure(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult { CallbackResult::Success }
///     fn on_activate(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult { CallbackResult::Success }
///     fn on_deactivate(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult { CallbackResult::Success }
///     fn on_cleanup(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult { CallbackResult::Success }
///     fn on_shutdown(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult { CallbackResult::Success }
///     fn on_error(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult { CallbackResult::Success }
/// }
///
/// # fn main() -> rosrustext_rosrs::Result<()> {
/// let context = Context::default();
/// let mut executor = context.create_basic_executor();
///
/// let lifecycle = LifecycleNode::create_with_callbacks(&executor, "demo", Box::new(Callbacks))?;
/// let _pub_ = lifecycle.create_publisher::<rosrustext_rosrs::lifecycle_msgs::msg::State>("state")?;
///
/// executor.spin(SpinOptions::default());
/// # Ok(()) }
/// ```
///
/// # See also
/// - [Lifecycle spec](https://github.com/convyares-FCSL/rosrustext/blob/main/docs/spec/lifecycle.md)
/// - [Lifecycle parity notes](https://github.com/convyares-FCSL/rosrustext/blob/main/parity.md)
/// - [`ManagedPublisher`]
/// - [`ManagedTimer`]
#[derive(Clone)]
pub struct LifecycleNode {
    node: Arc<Node>,
    gate: Arc<ActivationGate>,

    // Core state machine ownership
    machine: Arc<Mutex<StateMachine>>,

    // User callbacks
    callbacks: Arc<Mutex<Box<dyn LifecycleCallbacksWithNode + Send>>>,

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
impl LifecycleCallbacksWithNode for DefaultCallbacks {
    fn on_configure(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult {
        CallbackResult::Success
    }
    fn on_activate(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult {
        CallbackResult::Success
    }
    fn on_deactivate(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult {
        CallbackResult::Success
    }
    fn on_cleanup(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult {
        CallbackResult::Success
    }
    fn on_shutdown(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult {
        CallbackResult::Success
    }
    fn on_error(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult {
        CallbackResult::Success
    }
}

// ===== Public API =====
impl LifecycleNode {
    /// Create a new `rclrs::Node` and enable lifecycle semantics on it.
    ///
    /// # Semantics
    /// - Creates the underlying node via `executor.create_node(options)`.
    /// - Enables lifecycle services, `transition_event`, and any feature-gated
    ///   entities (e.g. `/bond`).
    ///
    /// # Errors
    /// Returns an error if the node or any lifecycle ROS entity cannot be created.
    ///
    /// # Example
    /// ```rust,no_run
    /// use rclrs::{Context, CreateBasicExecutor};
    /// use rosrustext_rosrs::lifecycle::LifecycleNode;
    ///
    /// # fn main() -> rosrustext_rosrs::Result<()> {
    /// let context = Context::default();
    /// let executor = context.create_basic_executor();
    /// let _node = LifecycleNode::create(&executor, "demo")?;
    /// # Ok(()) }
    /// ```
    ///
    /// # See also
    /// - [`LifecycleNode::create_with_callbacks`]
    pub fn create<'a>(executor: &'a Executor, options: impl IntoNodeOptions<'a>) -> Result<Self> {
        let node = Arc::new(executor.create_node(options)?);
        Self::try_new(node)
    }

    /// Wrap an existing `rclrs::Node` with lifecycle semantics.
    ///
    /// # Semantics
    /// - Enables lifecycle services and publishers on `node`.
    /// - Uses default no-op callbacks (always [`CallbackResult::Success`]).
    ///
    /// # Errors
    /// Returns an error if lifecycle entities cannot be created (for example due
    /// to name collisions with existing services/publishers on the same node).
    ///
    /// # Example
    /// ```rust,no_run
    /// use std::sync::Arc;
    /// use rclrs::{Context, CreateBasicExecutor};
    /// use rosrustext_rosrs::lifecycle::LifecycleNode;
    ///
    /// # fn main() -> rosrustext_rosrs::Result<()> {
    /// let context = Context::default();
    /// let executor = context.create_basic_executor();
    /// let node = Arc::new(executor.create_node("demo")?);
    /// let _lifecycle = LifecycleNode::try_new(node)?;
    /// # Ok(()) }
    /// ```
    ///
    /// # See also
    /// - [`LifecycleNode::new_with_callbacks`]
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

    /// Wrap an existing `rclrs::Node` with explicit callbacks.
    ///
    /// # Semantics
    /// Behaves like [`LifecycleNode::try_new`] but uses the provided callbacks.
    ///
    /// # Errors
    /// Returns an error if lifecycle entities cannot be created on `node`.
    ///
    /// # Example
    /// ```rust,no_run
    /// use std::sync::Arc;
    /// use rclrs::{Context, CreateBasicExecutor};
    /// use rosrustext_rosrs::lifecycle::{CallbackResult, LifecycleCallbacksWithNode, LifecycleNode};
    /// use rosrustext_rosrs::State;
    ///
    /// struct Callbacks;
    /// impl LifecycleCallbacksWithNode for Callbacks {
    ///     fn on_configure(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult { CallbackResult::Success }
    ///     fn on_activate(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult { CallbackResult::Success }
    ///     fn on_deactivate(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult { CallbackResult::Success }
    ///     fn on_cleanup(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult { CallbackResult::Success }
    ///     fn on_shutdown(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult { CallbackResult::Success }
    ///     fn on_error(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult { CallbackResult::Success }
    /// }
    ///
    /// # fn main() -> rosrustext_rosrs::Result<()> {
    /// let context = Context::default();
    /// let executor = context.create_basic_executor();
    /// let node = Arc::new(executor.create_node("demo")?);
    /// let _lifecycle = LifecycleNode::new_with_callbacks(node, Box::new(Callbacks))?;
    /// # Ok(()) }
    /// ```
    ///
    /// # See also
    /// - [`LifecycleCallbacksWithNode`]
    pub fn new_with_callbacks(node: Arc<Node>, callbacks: Box<dyn LifecycleCallbacksWithNode + Send>) -> Result<Self> {
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

    /// Alias for [`LifecycleNode::try_new`].
    ///
    /// # Semantics
    /// See [`LifecycleNode::try_new`].
    ///
    /// # Errors
    /// See [`LifecycleNode::try_new`].
    ///
    /// # Example
    /// See [`LifecycleNode::try_new`].
    ///
    /// # See also
    /// - [`LifecycleNode::try_new`]
    pub fn from_node(node: Arc<Node>) -> Result<Self> {
        Self::try_new(node)
    }

    /// For later slices: allow core-driven gate injection.
    ///
    /// # Semantics
    /// Advanced constructor that reuses an externally owned [`ActivationGate`].
    ///
    /// This is primarily useful for tests or for building higher-level orchestration
    /// that wants to share a gate across multiple managed resources.
    ///
    /// # Errors
    /// Returns an error if lifecycle entities cannot be created on `node`.
    ///
    /// # Example
    /// ```rust,no_run
    /// use std::sync::{Arc};
    /// use rclrs::{Context, CreateBasicExecutor};
    /// use rosrustext_rosrs::lifecycle::{ActivationGate, LifecycleNode};
    ///
    /// # fn main() -> rosrustext_rosrs::Result<()> {
    /// let context = Context::default();
    /// let executor = context.create_basic_executor();
    /// let node = Arc::new(executor.create_node("demo")?);
    /// let gate = Arc::new(ActivationGate::new());
    /// let _lifecycle = LifecycleNode::try_with_gate(node, gate)?;
    /// # Ok(()) }
    /// ```
    ///
    /// # See also
    /// - [`ActivationGate`]
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

    /// Primary constructor with callbacks that receive the lifecycle handle.
    ///
    /// # Semantics
    /// Convenience wrapper around:
    /// 1) creating an `rclrs::Node` on the provided executor
    /// 2) enabling lifecycle ROS entities
    /// 3) installing the provided callbacks
    ///
    /// # Errors
    /// Returns an error if the node or any lifecycle entity cannot be created.
    ///
    /// # Example
    /// ```rust,no_run
    /// use rclrs::{Context, CreateBasicExecutor};
    /// use rosrustext_rosrs::lifecycle::{CallbackResult, LifecycleCallbacksWithNode, LifecycleNode};
    /// use rosrustext_rosrs::State;
    ///
    /// struct Callbacks;
    /// impl LifecycleCallbacksWithNode for Callbacks {
    ///     fn on_configure(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult { CallbackResult::Success }
    ///     fn on_activate(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult { CallbackResult::Success }
    ///     fn on_deactivate(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult { CallbackResult::Success }
    ///     fn on_cleanup(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult { CallbackResult::Success }
    ///     fn on_shutdown(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult { CallbackResult::Success }
    ///     fn on_error(&mut self, _node: &LifecycleNode, _state: &State) -> CallbackResult { CallbackResult::Success }
    /// }
    ///
    /// # fn main() -> rosrustext_rosrs::Result<()> {
    /// let context = Context::default();
    /// let executor = context.create_basic_executor();
    /// let _lifecycle = LifecycleNode::create_with_callbacks(&executor, "demo", Box::new(Callbacks))?;
    /// # Ok(()) }
    /// ```
    ///
    /// # See also
    /// - [`LifecycleNode::create`]
    /// - [`LifecycleNode::new_with_callbacks`]
    pub fn create_with_callbacks<'a>(
        executor: &'a Executor, options: impl IntoNodeOptions<'a>,
        callbacks: Box<dyn LifecycleCallbacksWithNode + Send>,
    ) -> Result<Self> {
        let node = Arc::new(executor.create_node(options)?);
        Self::new_with_callbacks(node, callbacks)
    }

    /// Escape hatch: access the underlying rclrs::Node.
    ///
    /// # Semantics
    /// Returns a shared handle to the underlying `rclrs::Node`.
    ///
    /// Using the returned node to create publishers/timers will **bypass**
    /// lifecycle gating. Prefer [`LifecycleNode::create_publisher`] and
    /// [`LifecycleNode::create_timer_repeating_gated`] when you want managed behavior.
    ///
    /// # Errors
    /// This method does not fail.
    ///
    /// # Example
    /// ```rust,ignore
    /// let raw: &std::sync::Arc<rclrs::Node> = lifecycle.node();
    /// ```
    ///
    /// # See also
    /// - [`LifecycleNode::node_arc`]
    pub fn node(&self) -> &Arc<Node> {
        &self.node
    }

    /// Escape hatch: clone the underlying rclrs::Node.
    ///
    /// # Semantics
    /// Like [`LifecycleNode::node`], but clones the `Arc`.
    ///
    /// # Errors
    /// This method does not fail.
    ///
    /// # Example
    /// ```rust,ignore
    /// let raw: std::sync::Arc<rclrs::Node> = lifecycle.node_arc();
    /// ```
    ///
    /// # See also
    /// - [`LifecycleNode::node`]
    pub fn node_arc(&self) -> Arc<Node> {
        Arc::clone(&self.node)
    }

    /// Node name after ROS remapping.
    ///
    /// # Semantics
    /// Returns the node name as resolved by ROS remapping rules.
    ///
    /// # Errors
    /// This method does not fail.
    ///
    /// # Example
    /// ```rust,ignore
    /// let name = lifecycle.name();
    /// ```
    ///
    /// # See also
    /// - [`LifecycleNode::namespace`]
    pub fn name(&self) -> String {
        self.node.name()
    }

    /// Node namespace after ROS remapping.
    ///
    /// # Semantics
    /// Returns the node namespace as resolved by ROS remapping rules.
    ///
    /// # Errors
    /// This method does not fail.
    ///
    /// # Example
    /// ```rust,ignore
    /// let ns = lifecycle.namespace();
    /// ```
    ///
    /// # See also
    /// - [`LifecycleNode::name`]
    pub fn namespace(&self) -> String {
        self.node.namespace()
    }

    /// Create a non-lifecycle service on the underlying node.
    ///
    /// # Semantics
    /// - This service is **not** lifecycle-gated; it remains available regardless
    ///   of the lifecycle state.
    /// - Intended for application services that are independent of lifecycle state.
    ///
    /// # Errors
    /// Returns an error if `rclrs` fails to create the service.
    ///
    /// # Example
    /// ```rust,ignore
    /// let _svc = lifecycle.create_service::<std_srvs::srv::Trigger, _>("/ping", |_req| {
    ///     std_srvs::srv::Trigger_Response { success: true, message: "ok".into() }
    /// })?;
    /// ```
    ///
    /// # See also
    /// - [`LifecycleNode::create_subscription`]
    pub fn create_service<'a, T, Args>(
        &self, options: impl Into<ServiceOptions<'a>>, callback: impl IntoNodeServiceCallback<T, Args>,
    ) -> Result<Service<T>>
    where
        T: rclrs::ServiceIDL,
    {
        Ok(self.node.create_service::<T, Args>(options, callback)?)
    }

    /// Create a non-lifecycle subscription on the underlying node.
    ///
    /// # Semantics
    /// Subscriptions are not lifecycle-gated. Use this for data-plane subscriptions
    /// that should continue to receive messages in all lifecycle states.
    ///
    /// # Errors
    /// Returns an error if `rclrs` fails to create the subscription.
    ///
    /// # Example
    /// ```rust,ignore
    /// let _sub = lifecycle.create_subscription::<std_msgs::msg::String, _>("chatter", |_msg| {})?;
    /// ```
    ///
    /// # See also
    /// - [`LifecycleNode::create_service`]
    pub fn create_subscription<'a, T, Args>(
        &self, options: impl Into<SubscriptionOptions<'a>>, callback: impl IntoNodeSubscriptionCallback<T, Args>,
    ) -> Result<Subscription<T>>
    where
        T: rclrs::MessageIDL,
    {
        Ok(self.node.create_subscription::<T, Args>(options, callback)?)
    }

    /// Create a client on the underlying node.
    ///
    /// # Semantics
    /// Clients are not lifecycle-gated.
    ///
    /// # Errors
    /// Returns an error if `rclrs` fails to create the client.
    ///
    /// # Example
    /// ```rust,ignore
    /// let _client = lifecycle.create_client::<std_srvs::srv::Trigger>("/ping")?;
    /// ```
    ///
    /// # See also
    /// - [`LifecycleNode::create_service`]
    pub fn create_client<'a, T>(&self, options: impl Into<ClientOptions<'a>>) -> Result<Client<T>>
    where
        T: rclrs::ServiceIDL,
    {
        Ok(self.node.create_client::<T>(options)?)
    }

    /// Set activation / deactivation gate state.
    ///
    /// # Semantics
    /// Manually toggles the internal [`ActivationGate`]. In typical usage the
    /// gate is driven by lifecycle transitions (`Active` ↔ not `Active`), so
    /// most applications should not need to call this directly.
    ///
    /// # Errors
    /// This method does not fail.
    ///
    /// # Example
    /// ```rust,ignore
    /// lifecycle.activate();
    /// ```
    ///
    /// # See also
    /// - [`LifecycleNode::deactivate`]
    /// - [`LifecycleNode::is_active`]
    pub fn activate(&self) {
        self.gate.activate();
    }
    /// Set activation / deactivation gate state.
    ///
    /// # Semantics
    /// See [`LifecycleNode::activate`].
    ///
    /// # Errors
    /// This method does not fail.
    ///
    /// # Example
    /// ```rust,ignore
    /// lifecycle.deactivate();
    /// ```
    ///
    /// # See also
    /// - [`LifecycleNode::activate`]
    /// - [`LifecycleNode::is_active`]
    pub fn deactivate(&self) {
        self.gate.deactivate();
    }

    /// Get activation gate state.
    ///
    /// # Semantics
    /// Returns `true` when the node is considered “active” for managed-resource
    /// gating purposes.
    ///
    /// # Errors
    /// This method does not fail.
    ///
    /// # Example
    /// ```rust,ignore
    /// if lifecycle.is_active() { /* … */ }
    /// ```
    ///
    /// # See also
    /// - [`LifecycleNode::activate`]
    /// - [`LifecycleNode::deactivate`]
    pub fn is_active(&self) -> bool {
        self.gate.is_active()
    }

    /// Create a publisher managed by the lifecycle node's activation gate.
    ///
    /// # Semantics
    /// Returns a [`ManagedPublisher`] that suppresses publishes while inactive.
    ///
    /// # Errors
    /// Returns an error if `rclrs` fails to create the underlying publisher.
    ///
    /// # Example
    /// ```rust,ignore
    /// let pub_ = lifecycle.create_publisher::<std_msgs::msg::String>("chatter")?;
    /// ```
    ///
    /// # See also
    /// - [`ManagedPublisher`]
    /// - [`ManagedPublisher::publish_with_outcome`]
    pub fn create_publisher<T>(&self, topic: &str) -> Result<ManagedPublisher<T>>
    where
        T: rclrs::MessageIDL,
    {
        let pub_ = self.node.create_publisher::<T>(topic)?;
        Ok(ManagedPublisher::new(pub_, Arc::clone(&self.gate)))
    }

    /// Create a repeating timer managed by the lifecycle node's activation gate.
    ///
    /// # Semantics
    /// - Creates a repeating `rclrs::Timer` that fires at `period`.
    /// - On each tick, the callback runs **only if** the node is active.
    /// - While inactive, ticks are silently skipped (the timer still fires).
    ///
    /// Timer replacement guidance:
    /// - Keep the returned [`ManagedTimer`] alive to keep the timer installed.
    /// - To change the period or callback, drop the existing handle and create
    ///   a new gated timer (e.g. store it in an `Option<ManagedTimer>` and replace).
    ///
    /// # Errors
    /// Returns an error if `rclrs` fails to create the timer.
    ///
    /// # Example
    /// ```rust,ignore
    /// use std::time::Duration;
    /// let _timer = lifecycle.create_timer_repeating_gated(Duration::from_millis(100), move || {
    ///     // Runs only while Active
    /// })?;
    /// ```
    ///
    /// # See also
    /// - [`ManagedTimer`]
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
            let evt = utils::make_transition_event(
                result.start_state,
                result.final_state,
                outcome.transition_id,
                outcome.label,
            );
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
                let current = machine.lock().expect("machine mutex poisoned").stable_state();

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
                let active_self_for_cb = active_self.clone();
                let start_state = flight.start;

                // Closure to execute the user callback
                let do_callback = move || -> CompleteInput {
                    if delay_ms > 0 {
                        std::thread::sleep(Duration::from_millis(delay_ms));
                    }

                    let mut cb_guard = callbacks_clone.lock().expect("callbacks mutex poisoned");
                    let mut result =
                        run_transition_callback(cb_guard.as_mut(), flight.transition, &active_self_for_cb, start_state);
                    if let Some(forced) = utils::transition_result_override_for(flight.transition) {
                        result = forced;
                    }

                    let on_error_result = if result == CallbackResult::Error {
                        let mut on_error = cb_guard.on_error(&active_self_for_cb, &start_state);
                        if let Some(forced) = utils::on_error_result_override_for(flight.transition) {
                            on_error = forced;
                        }
                        Some(on_error)
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

                rosrustext_msgs::lifecycle_msgs::srv::GetAvailableTransitions_Response {
                    available_transitions: transitions,
                }
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
