//! roslibrust lifecycle service adapters
//!
//! This module wires transport callbacks to wrapper lifecycle logic.
//! It contains **no ROS message types** — only transport glue.

use std::sync::{Arc, Mutex};

use crate::error::log_core_error;
use crate::lifecycle::{dtos, LifecycleNode};
use rosrustext_core::lifecycle::finish_with_error_handling;
use rosrustext_core::lifecycle::CallbackResult;
use rosrustext_core::lifecycle::Transition;
use tracing::warn;

/// ROS-facing lifecycle service adapter.
///
/// Owns a shared `LifecycleNode` and exposes handlers that can be
/// registered with roslibrust.
pub struct LifecycleService {
    node: Arc<Mutex<LifecycleNode>>,
}

impl LifecycleService {
    /// Create a new lifecycle service adapter.
    pub fn new(node: Arc<Mutex<LifecycleNode>>) -> Self {
        Self { node }
    }

    /// DTO handler: ChangeState
    pub fn handle_change_state(
        &self,
        req: dtos::change_state::Request,
    ) -> dtos::change_state::Response {
        let work = {
            let mut node = self.node.lock().expect("lifecycle node poisoned");
            node.begin_transition_ros(req.transition_id)
        };

        match work {
            Ok(work) => {
                let node = Arc::clone(&self.node);
                spawn_transition_task(move || {
                    let mut work = work;
                    let result = run_transition_callback(work.transition, work.callbacks.as_mut());
                    let on_error = if result == CallbackResult::Error {
                        Some(work.callbacks.on_error())
                    } else {
                        None
                    };

                    let final_state = match finish_with_error_handling(
                        work.intermediate_state,
                        work.transition,
                        result,
                        on_error,
                    ) {
                        Ok(state) => state,
                        Err(err) => {
                            log_core_error(err);
                            if let Ok(mut guard) = node.lock() {
                                guard.abort_transition_ros(work);
                            }
                            return;
                        }
                    };

                    match node.lock() {
                        Ok(mut guard) => {
                            if let Err(err) = guard.complete_transition_ros(work, final_state) {
                                log_core_error(err);
                            }
                        }
                        Err(_) => warn!("lifecycle node poisoned"),
                    }
                });

                dtos::change_state::Response {
                    success: true,
                    message: "transition accepted".to_string(),
                }
            }
            Err(e) => dtos::change_state::Response {
                success: false,
                message: format!("transition failed: {e}"),
            },
        }
    }

    /// Adapter for ROS-facing handlers that only need a transition id.
    pub fn handle_change_state_transition_id(&self, transition_id: u8) -> (bool, String) {
        let resp = self.handle_change_state(dtos::change_state::Request { transition_id });
        (resp.success, resp.message)
    }

    /// DTO handler: GetState
    pub fn handle_get_state(&self, req: dtos::get_state::Request) -> dtos::get_state::Response {
        let node = self.node.lock().expect("lifecycle node poisoned");
        node.handle_get_state(req)
    }

    /// DTO handler: GetAvailableTransitions
    pub fn handle_get_available_transitions(
        &self,
        req: dtos::get_available_transitions::Request,
    ) -> dtos::get_available_transitions::Response {
        let node = self.node.lock().expect("lifecycle node poisoned");
        node.handle_get_available_transitions(req)
    }

    /// DTO handler: GetAvailableStates
    pub fn handle_get_available_states(
        &self,
        req: dtos::get_available_states::Request,
    ) -> dtos::get_available_states::Response {
        let node = self.node.lock().expect("lifecycle node poisoned");
        node.handle_get_available_states(req)
    }

    /// DTO handler: GetTransitionGraph
    pub fn handle_get_transition_graph(
        &self,
        req: dtos::get_transition_graph::Request,
    ) -> rosrustext_core::error::Result<dtos::get_transition_graph::Response> {
        let node = self.node.lock().expect("lifecycle node poisoned");
        node.handle_get_transition_graph(req)
    }

    /// Best-effort graceful shutdown:
    /// - picks the correct shutdown transition id for the node's current state
    /// - drives the lifecycle so gate/callbacks run
    pub fn shutdown_best_effort(&self) -> (bool, String) {
        let mut node = self.node.lock().expect("lifecycle node poisoned");
        let Some(transition_id) = node.shutdown_transition_ros_id() else {
            return (true, "no shutdown transition for current state".to_string());
        };
        match node.request_transition_ros(transition_id) {
            Ok((_mid, final_state)) => (
                true,
                format!("transition ok -> {final_state:?}"),
            ),
            Err(err) => (false, format!("transition failed: {err}")),
        }
    }
}

fn run_transition_callback(
    transition: Transition,
    callbacks: &mut dyn crate::lifecycle::LifecycleCallbacks,
) -> CallbackResult {
    match transition {
        Transition::Configure => callbacks.on_configure(),
        Transition::Activate => callbacks.on_activate(),
        Transition::Deactivate => callbacks.on_deactivate(),
        Transition::Cleanup => callbacks.on_cleanup(),
        Transition::Shutdown => callbacks.on_shutdown(),
    }
}

fn spawn_transition_task<F>(task: F)
where
    F: FnOnce() + Send + 'static,
{
    if let Ok(handle) = tokio::runtime::Handle::try_current() {
        handle.spawn_blocking(task);
    } else {
        std::thread::spawn(task);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::lifecycle::LifecycleCallbacks;
    use rosrustext_core::lifecycle::CallbackResult;

    struct OkCallbacks;
    impl LifecycleCallbacks for OkCallbacks {
        fn on_configure(&mut self) -> CallbackResult {
            CallbackResult::Success
        }
        fn on_activate(&mut self) -> CallbackResult {
            CallbackResult::Success
        }
        fn on_deactivate(&mut self) -> CallbackResult {
            CallbackResult::Success
        }
        fn on_cleanup(&mut self) -> CallbackResult {
            CallbackResult::Success
        }
        fn on_shutdown(&mut self) -> CallbackResult {
            CallbackResult::Success
        }
        fn on_error(&mut self) -> CallbackResult {
            CallbackResult::Success
        }
    }

    #[test]
    fn change_state_dto_maps_and_reports_success() {
        let node = Arc::new(Mutex::new(
            LifecycleNode::new("test_node", Box::new(OkCallbacks)).unwrap(),
        ));
        let svc = LifecycleService::new(node);

        let req = dtos::change_state::Request {
            transition_id: crate::lifecycle::ros_ids::TRANSITION_CONFIGURE,
        };

        let resp = svc.handle_change_state(req);
        assert!(resp.success);
        assert!(!resp.message.is_empty());
    }

    #[test]
    fn change_state_dto_rejects_unsupported_transition_id() {
        let node = Arc::new(Mutex::new(
            LifecycleNode::new("test_node", Box::new(OkCallbacks)).unwrap(),
        ));
        let svc = LifecycleService::new(node);

        let resp = svc.handle_change_state(dtos::change_state::Request {
            transition_id: u8::MAX,
        });
        assert!(!resp.success);
        assert!(!resp.message.is_empty());
    }

    #[test]
    fn get_available_states_dto_reports_all_states() {
        let node = Arc::new(Mutex::new(
            LifecycleNode::new("test_node", Box::new(OkCallbacks)).unwrap(),
        ));
        let svc = LifecycleService::new(node);

        let resp = svc.handle_get_available_states(dtos::get_available_states::Request);

        assert!(resp.states.iter().any(|s| s.label == "Unconfigured"));
        assert!(resp.states.iter().any(|s| s.label == "Active"));
        assert!(resp.states.iter().any(|s| s.label == "ErrorProcessing"));
        assert!(resp.states.len() >= 10);
    }
}
