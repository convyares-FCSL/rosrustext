use std::sync::Arc;

use rosrustext_core::error::{CoreError, Domain, ErrorKind, Result};
use rosrustext_core::lifecycle::{ActivationGate, LifecycleCallbacks, State};

/// Internal API (wrapper facing).
#[cfg(any(test, feature = "roslibrust"))]
use super::dtos::{change_state, get_available_states, get_available_transitions, get_state};
#[cfg(any(test, feature = "roslibrust"))]
use rosrustext_core::lifecycle::{available_transitions, drive};
#[cfg(any(test, feature = "roslibrust"))]
use rosrustext_core::lifecycle::ALL_STATES;
#[cfg(any(test, feature = "roslibrust"))]
use crate::lifecycle::{shutdown_ros_id_for_state, transition_from_ros_id};
#[cfg(test)]
use crate::lifecycle::ros_ids;

use crate::lifecycle::TransitionEvent;


/// Wrapper-side lifecycle node adapter.
///
/// Responsibilities:
/// - Hold current lifecycle state
/// - Hold the user's callback implementation
/// - Provide an activation gate for managed resources (publishers/timers)
/// - Provide a transition-event stream (ROS `/transition_event` equivalent, ROS-agnostic)
/// - Expose lifecycle service handlers (via transport layer)
pub struct LifecycleNode {
    name: String,
    state: State,
    gate: Arc<ActivationGate>,

    // Used by drive() when applying transitions.
    #[cfg_attr(not(any(test, feature = "roslibrust")), allow(dead_code))]
    callbacks: Box<dyn LifecycleCallbacks + Send>,

    // Wrapper-side `/transition_event` equivalent.
    //
    // broadcast is used so:
    // - lifecycle transitions never block on a slow transport publisher
    // - lagging receivers drop old events rather than stalling the node
    transition_events: tokio::sync::broadcast::Sender<TransitionEvent>,
}

/// Public API (library user facing).
impl LifecycleNode {
    /// Create a new lifecycle node adapter.
    ///
    /// Starts in `Unconfigured` with gate inactive.
    pub fn new(name: impl Into<String>, callbacks: Box<dyn LifecycleCallbacks + Send>) -> Result<Self> {
        let name = name.into();
        if name.is_empty() {
            return Err(
                CoreError::error()
                    .domain(Domain::Lifecycle)
                    .kind(ErrorKind::InvalidArgument)
                    .msg("node name must not be empty")
                    .build(),
            );
        }

        let (transition_events, _rx) = tokio::sync::broadcast::channel(32);

        Ok(Self {
            name,
            state: State::Unconfigured,
            gate: Arc::new(ActivationGate::new()),
            callbacks,
            transition_events,
        })
    }

    /// Current lifecycle state.
    pub fn state(&self) -> State {
        self.state
    }

    /// Shared activation gate for wrapper-managed resources.
    pub fn activation_gate(&self) -> Arc<ActivationGate> {
        Arc::clone(&self.gate)
    }

    /// Node name (for logging/introspection).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Subscribe to lifecycle transition events (wrapper `/transition_event` equivalent).
    ///
    /// Transport layers can map these to `lifecycle_msgs/msg/TransitionEvent` and publish them.
    pub fn subscribe_transition_events(&self) -> tokio::sync::broadcast::Receiver<TransitionEvent> {
        self.transition_events.subscribe()
    }
}

/// Internal wrapper plumbing (called by transport/service servers).
#[cfg(any(test, feature = "roslibrust"))]
impl LifecycleNode {
    /// Internal: apply a ROS transition id to the core lifecycle and update gating.
    ///
    /// Returns (intermediate_state, final_state) as per core drive().
    pub(crate) fn request_transition_ros(&mut self, ros_transition_id: u8) -> Result<(State, State)> {
        let transition = transition_from_ros_id(ros_transition_id).ok_or_else(|| {
            CoreError::warn()
                .domain(Domain::Lifecycle)
                .kind(ErrorKind::InvalidArgument)
                .msg("unsupported ROS lifecycle transition id")
                .build()
        })?;

        let current = self.state;

        let (intermediate, final_state) = drive(current, transition, self.callbacks.as_mut())?;

        // Apply activation policy
        match (current, final_state) {
            (_, State::Active) => self.gate.activate(),     // entering Active => gate on
            (State::Active, _) => self.gate.deactivate(),   // leaving Active => gate off
            _ => {}
        }

        // Update state
        self.state = final_state;

        // Emit transition event after successful transition.
        // Ignore errors (no receivers / lagging receivers) to keep lifecycle non-blocking.
        let _ = self.transition_events.send(TransitionEvent {
            transition_id: ros_transition_id,
            start_state: current,
            goal_state: final_state,
        });

        Ok((intermediate, final_state))
    }

    /// Internal handler for lifecycle `ChangeState` service.
    pub(crate) fn handle_change_state(&mut self, req: change_state::Request) -> change_state::Response {
        match self.request_transition_ros(req.transition_id) {
            Ok((_mid, final_state)) => change_state::Response {
                success: true,
                message: format!("transition ok -> {final_state:?}"),
            },
            Err(e) => change_state::Response {
                success: false,
                message: format!("transition failed: {e}"),
            },
        }
    }

    /// Internal handler for lifecycle `GetState` service.
    pub(crate) fn handle_get_state(&self, _req: get_state::Request) -> get_state::Response {
        get_state::Response {
            state_id: self.state.id(),
            label: format!("{:?}", self.state),
        }
    }

    /// Internal handler for lifecycle `GetAvailableTransitions` service.
    pub(crate) fn handle_get_available_transitions(
        &self,
        _req: get_available_transitions::Request,
    ) -> get_available_transitions::Response {
        let transitions = available_transitions(self.state)
            .iter()
            .map(|t| get_available_transitions::Transition {
                id: t.id(),
                label: format!("{:?}", t),
            })
            .collect();

        get_available_transitions::Response { transitions }
    }

    /// Internal handler for lifecycle `GetAvailableStates` service.
    pub(crate) fn handle_get_available_states(
        &self,
        _req: get_available_states::Request,
    ) -> get_available_states::Response {
        let states = ALL_STATES
            .into_iter()
            .map(|s| get_available_states::State {
                id: s.id(),
                label: s.label().to_string(),
            })
            .collect();

        get_available_states::Response { states }
    }

    /// Internal helper: get the correct ROS shutdown ID for the current state.
    #[allow(dead_code)]
    pub(crate) fn shutdown_transition_ros_id(&self) -> Option<u8> {
        shutdown_ros_id_for_state(self.state)
    }
}

/// Unit tests for LifecycleNode.
#[cfg(test)]
mod tests {
    use super::*;
    use rosrustext_core::lifecycle::CallbackResult;

    struct OkCallbacks;

    impl LifecycleCallbacks for OkCallbacks {
        fn on_configure(&mut self) -> CallbackResult { CallbackResult::Success }
        fn on_activate(&mut self) -> CallbackResult { CallbackResult::Success }
        fn on_deactivate(&mut self) -> CallbackResult { CallbackResult::Success }
        fn on_cleanup(&mut self) -> CallbackResult { CallbackResult::Success }
        fn on_shutdown(&mut self) -> CallbackResult { CallbackResult::Success }
        fn on_error(&mut self) -> CallbackResult { CallbackResult::Success }
    }

    #[test]
    fn ros_transition_configure_then_activate_sets_gate() {
        let mut node = LifecycleNode::new("test_node", Box::new(OkCallbacks)).unwrap();

        let (_mid, s1) = node
            .request_transition_ros(ros_ids::TRANSITION_CONFIGURE)
            .unwrap();
        assert_eq!(s1, State::Inactive);
        assert!(!node.activation_gate().is_active());

        let (_mid, s2) = node
            .request_transition_ros(ros_ids::TRANSITION_ACTIVATE)
            .unwrap();
        assert_eq!(s2, State::Active);
        assert!(node.activation_gate().is_active());
    }

    #[test]
    fn change_state_handler_success() {
        let mut node = LifecycleNode::new("test_node", Box::new(OkCallbacks)).unwrap();

        let resp = node.handle_change_state(change_state::Request {
            transition_id: ros_ids::TRANSITION_CONFIGURE,
        });

        assert!(resp.success);
        assert!(resp.message.contains("Inactive"));
    }

    #[test]
    fn get_state_reports_current_state() {
        let node = LifecycleNode::new("test_node", Box::new(OkCallbacks)).unwrap();

        let resp = node.handle_get_state(get_state::Request);

        assert_eq!(resp.state_id, rosrustext_core::lifecycle::State::Unconfigured.id());
        assert!(resp.label.contains("Unconfigured"));
    }

    #[test]
    fn get_available_transitions_reports_active_state() {
        let mut node = LifecycleNode::new("test_node", Box::new(OkCallbacks)).unwrap();

        node.request_transition_ros(ros_ids::TRANSITION_CONFIGURE)
            .unwrap();
        node.request_transition_ros(ros_ids::TRANSITION_ACTIVATE)
            .unwrap();

        let resp = node.handle_get_available_transitions(get_available_transitions::Request);

        assert!(resp.transitions.iter().any(|t| t.label == "Deactivate"));
        assert!(resp.transitions.iter().any(|t| t.label == "Shutdown"));
    }

    #[test]
    fn get_available_states_reports_all_states() {
        let node = LifecycleNode::new("test_node", Box::new(OkCallbacks)).unwrap();

        let resp =
            node.handle_get_available_states(crate::lifecycle::dtos::get_available_states::Request);

        assert!(resp.states.iter().any(|s| s.label == "Unconfigured"));
        assert!(resp.states.iter().any(|s| s.label == "Inactive"));
        assert!(resp.states.iter().any(|s| s.label == "Active"));
        assert!(resp.states.iter().any(|s| s.label == "Finalized"));

        assert!(resp.states.iter().any(|s| s.label == "Configuring"));
        assert!(resp.states.iter().any(|s| s.label == "CleaningUp"));
        assert!(resp.states.iter().any(|s| s.label == "Activating"));
        assert!(resp.states.iter().any(|s| s.label == "Deactivating"));
        assert!(resp.states.iter().any(|s| s.label == "ShuttingDown"));
        assert!(resp.states.iter().any(|s| s.label == "ErrorProcessing"));
    }

    #[test]
    fn transition_event_emitted_on_successful_transition() {
        let mut node = LifecycleNode::new("test_node", Box::new(OkCallbacks)).unwrap();
        let mut rx = node.subscribe_transition_events();

        node.request_transition_ros(ros_ids::TRANSITION_CONFIGURE)
            .unwrap();

        let ev = rx.try_recv().expect("expected transition event");
        assert_eq!(
            ev.transition_id,
            ros_ids::TRANSITION_CONFIGURE
        );
        assert_eq!(ev.start_state, State::Unconfigured);
        assert_eq!(ev.goal_state, State::Inactive);
    }
}   
