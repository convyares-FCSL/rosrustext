use std::sync::Arc;
use std::time::Duration;

use rclrs::{Node, TimerOptions};

use crate::error::Result;
use rosrustext_core::lifecycle::ActivationGate;

use super::{ManagedPublisher, ManagedTimer};

#[derive(Clone)]
pub struct LifecycleNode {
    node: Arc<Node>,
    gate: Arc<ActivationGate>,
}

impl LifecycleNode {
    pub fn new(node: Arc<Node>) -> Self {
        Self {
            node,
            gate: Arc::new(ActivationGate::new()),
        }
    }

    /// For later slices: allow core-driven gate injection from the lifecycle machine.
    pub fn with_gate(node: Arc<Node>, gate: Arc<ActivationGate>) -> Self {
        Self { node, gate }
    }

    pub fn node(&self) -> &Arc<Node> {
        &self.node
    }

    /// Slice-1 compatibility: manual toggling (later driven by core lifecycle transitions).
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

    pub fn create_timer_repeating_gated<F>(&self, period: Duration, mut callback: F) -> Result<ManagedTimer>
    where
        F: FnMut() + Send + 'static,
    {
        let gate = Arc::clone(&self.gate);
        let timer = self.node.create_timer_repeating(
            TimerOptions::new(period),
            move || {
                if gate.is_active() {
                    callback();
                }
            },
        )?;

        Ok(ManagedTimer::new(timer))
    }
}
