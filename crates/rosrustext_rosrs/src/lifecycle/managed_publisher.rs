use std::sync::Arc;

use crate::error::Result;
use rosrustext_core::lifecycle::ActivationGate;

/// A publisher wrapper that drops publishes while inactive.
pub struct ManagedPublisher<T>
where
    T: rclrs::MessageIDL,
{
    inner: rclrs::Publisher<T>,
    gate: Arc<ActivationGate>,
}

impl<T> ManagedPublisher<T>
where
    T: rclrs::MessageIDL,
{
    pub(crate) fn new(inner: rclrs::Publisher<T>, gate: Arc<ActivationGate>) -> Self {
        Self { inner, gate }
    }

    pub fn publish(&self, msg: T) -> Result<()> {
        if self.gate.is_active() {
            self.inner.publish(msg)?;
        }
        Ok(())
    }

    pub fn inner(&self) -> &rclrs::Publisher<T> {
        &self.inner
    }
}
