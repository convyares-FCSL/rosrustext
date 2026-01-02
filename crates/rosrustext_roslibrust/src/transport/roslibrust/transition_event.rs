use std::sync::{Arc, Mutex};

use crate::lifecycle::{LifecycleNode, TransitionEvent};
use rosrustext_core::error::{CoreError, Domain, ErrorKind, Payload, Result};

/// Minimal trait: downstream provides a concrete ROS message type.
/// This keeps rosrustext_* crates independent of `lifecycle_msgs`.
pub trait FromTransitionEvent: Sized + Send + Sync + 'static {
    fn from_event(ev: &TransitionEvent) -> Self;
}

#[cfg(feature = "roslibrust")]
pub async fn run_transition_event_publisher<M>(
    node: Arc<Mutex<LifecycleNode>>,
    publisher: Arc<roslibrust::rosbridge::Publisher<M>>,
) -> Result<()>
where
    M: roslibrust::RosMessageType + FromTransitionEvent,
{
    // Grab the receiver once, then drop the lock.
    let mut rx = {
        let guard = node.lock().map_err(|_| {
 CoreError::error()
                .domain(Domain::Lifecycle)
                .kind(ErrorKind::InvalidState) // or ErrorKind::Other if you prefer
                .msg("LifecycleNode mutex poisoned")
                .payload(Payload::Context {key: "where", value: "run_transition_event_publisher".into(),
                })
                .build()
        })?;
        guard.subscribe_transition_events()
    };

    loop {
        let ev = rx.recv().await.map_err(|e| {
            CoreError::warn()
                .domain(Domain::Lifecycle)
                .msgf(format_args!("transition event recv failed: {e}"))
                .build()
        })?;

        let msg = M::from_event(&ev);
        
        publisher.publish(&msg).await.map_err(|e| {
            CoreError::warn()
                .domain(Domain::Transport)
                .kind(ErrorKind::Transport)
                .msgf(format_args!("transition event publish failed: {e}"))
                .build()
        })?;
    }
}
