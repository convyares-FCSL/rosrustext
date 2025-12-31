//! roslibrust lifecycle service adapters
//!
//! This module wires transport callbacks to wrapper lifecycle logic.
//! It contains **no ROS message types** — only transport glue.

use std::sync::{Arc, Mutex};

use crate::lifecycle::{dtos, LifecycleNode};

/// ROS-facing lifecycle service adapter.
///
/// Owns a shared `LifecycleNode` and exposes handlers that can be
/// registered with roslibrust.
pub struct LifecycleService {
    node: Arc<Mutex<LifecycleNode>>,
}

impl LifecycleService {
    pub fn new(node: Arc<Mutex<LifecycleNode>>) -> Self {
        Self { node }
    }

    /// DTO handler: ChangeState
    pub fn handle_change_state(&self, req: dtos::change_state::Request) -> dtos::change_state::Response {
        let mut node = self.node.lock().expect("lifecycle node poisoned");
        node.handle_change_state(req)
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

    /// Best-effort graceful shutdown:
    /// - picks the correct shutdown transition id for the node's current state
    /// - drives the lifecycle so gate/callbacks run
    pub fn shutdown_best_effort(&self) -> (bool, String) {
        let mut node = self.node.lock().expect("lifecycle node poisoned");
        let Some(transition_id) = node.shutdown_transition_ros_id() else {
            return (true, "no shutdown transition for current state".to_string());
        };
        let resp = node.handle_change_state(crate::lifecycle::dtos::change_state::Request {
            transition_id,
        });
        (resp.success, resp.message)
    }
    
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::lifecycle::LifecycleCallbacks;
    use rclrust_core::lifecycle::CallbackResult;

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
    fn change_state_dto_maps_and_reports_success() {
        let node = Arc::new(Mutex::new(LifecycleNode::new("test_node", Box::new(OkCallbacks)).unwrap()));
        let svc = LifecycleService::new(node);

        let req = dtos::change_state::Request {
            transition_id: rclrust_core::lifecycle::ros_ids::TRANSITION_CONFIGURE,
        };

        let resp = svc.handle_change_state(req);
        assert!(resp.success);
        assert!(!resp.message.is_empty());
    }

    #[test]
    fn change_state_dto_rejects_unsupported_transition_id() {
        let node = Arc::new(Mutex::new(LifecycleNode::new("test_node", Box::new(OkCallbacks)).unwrap()));
        let svc = LifecycleService::new(node);

        let resp = svc.handle_change_state(dtos::change_state::Request {
            transition_id: u8::MAX,
        });
        assert!(!resp.success);
        assert!(!resp.message.is_empty());
    }
}
