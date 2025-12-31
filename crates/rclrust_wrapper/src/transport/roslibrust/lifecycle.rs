//! roslibrust lifecycle service adapters
//!
//! This module wires ROS lifecycle services to the wrapper lifecycle logic.
//! It contains **no lifecycle semantics** — only transport glue.

use std::sync::{Arc, Mutex};

use crate::lifecycle::LifecycleNode;
use crate::lifecycle::dtos::{change_state, get_available_transitions, get_state};

/// ROS-facing lifecycle service adapter.
///
/// Owns a shared `LifecycleNode` and exposes service handlers that can be
/// registered with roslibrust.
pub struct LifecycleService {
    node: Arc<Mutex<LifecycleNode>>,
}

/// Public API (transport layer facing).
impl LifecycleService {
    /// Create a new lifecycle service adapter.
    ///
    /// The node is wrapped in Arc<Mutex<..>> because:
    /// - service callbacks may be invoked concurrently
    /// - lifecycle transitions must be serialized
    pub fn new(node: Arc<Mutex<LifecycleNode>>) -> Self {
        Self { node }
    }

    /// Handle a ROS `ChangeState` service request.
    ///
    /// This will be registered as the service callback in roslibrust.
    pub fn handle_change_state(&self, req: change_state::Request) -> change_state::Response {
        let mut node = self.node.lock().expect("lifecycle node poisoned");
        node.handle_change_state(req)
    }

    pub fn handle_get_state(&self, req: get_state::Request) -> get_state::Response {
        let node = self.node.lock().expect("lifecycle node poisoned");
        node.handle_get_state(req)
    }

    pub fn handle_get_available_transitions(
        &self,
        req: get_available_transitions::Request,
    ) -> get_available_transitions::Response {
        let node = self.node.lock().expect("lifecycle node poisoned");
        node.handle_get_available_transitions(req)
    }
}
