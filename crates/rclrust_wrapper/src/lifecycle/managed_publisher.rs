use std::sync::Arc;

use rclrust_core::lifecycle::ActivationGate;

/// Minimal async publish capability.
///
/// This is intentionally tiny so we can:
/// - unit test gating without ROS
/// - adapt roslibrust publishers in the transport layer
pub trait PublishLike<T>: Send + Sync + 'static {
    type Error: std::error::Error + Send + Sync + 'static;

    fn publish<'a>(&'a self, msg: &'a T)
        -> std::pin::Pin<Box<dyn std::future::Future<Output = Result<(), Self::Error>> + Send + 'a>>;
}

/// Lifecycle-gated publisher wrapper (ROS2 lifecycle parity behavior).
///
/// Parity goal with rclcpp_lifecycle:
/// - publish attempts are suppressed unless the node is Active
/// - wrapper owns no ROS transport; it wraps any `PublishLike` implementation
pub struct ManagedPublisher<T, P>
where
    P: PublishLike<T>,
{
    gate: Arc<ActivationGate>,
    inner: Arc<P>,
    _phantom: std::marker::PhantomData<T>,
}

impl<T, P> ManagedPublisher<T, P>
where
    P: PublishLike<T>,
{
    pub fn new(gate: Arc<ActivationGate>, inner: Arc<P>) -> Self {
        Self {
            gate,
            inner,
            _phantom: std::marker::PhantomData,
        }
    }

    /// Publish only when the lifecycle gate is active.
    ///
    /// Returns:
    /// - Ok(true): published
    /// - Ok(false): suppressed because inactive
    /// - Err(_): underlying publisher error
    pub async fn publish(&self, msg: &T) -> Result<bool, P::Error> {
        if !self.gate.is_active() {
            return Ok(false);
        }
        self.inner.publish(msg).await?;
        Ok(true)
    }

    /// Access to the underlying publisher (escape hatch).
    pub fn inner(&self) -> &Arc<P> {
        &self.inner
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering};

    #[derive(Debug)]
    struct DummyError;

    impl std::fmt::Display for DummyError {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            write!(f, "dummy error")
        }
    }
    impl std::error::Error for DummyError {}

    struct DummyPublisher {
        calls: AtomicUsize,
    }

    impl DummyPublisher {
        fn new() -> Self {
            Self {
                calls: AtomicUsize::new(0),
            }
        }

        fn calls(&self) -> usize {
            self.calls.load(Ordering::Relaxed)
        }
    }

    impl PublishLike<String> for DummyPublisher {
        type Error = DummyError;

        fn publish<'a>(
            &'a self,
            _msg: &'a String,
        ) -> std::pin::Pin<Box<dyn std::future::Future<Output = Result<(), Self::Error>> + Send + 'a>>
        {
            Box::pin(async move {
                self.calls.fetch_add(1, Ordering::Relaxed);
                Ok(())
            })
        }
    }

    #[tokio::test]
    async fn suppressed_when_inactive() {
        let gate = Arc::new(ActivationGate::new());
        let inner = Arc::new(DummyPublisher::new());
        let pub_ = ManagedPublisher::<String, _>::new(gate, inner.clone());

        let ok = pub_.publish(&"hello".to_string()).await.unwrap();
        assert!(!ok);
        assert_eq!(inner.calls(), 0);
    }

    #[tokio::test]
    async fn publishes_when_active() {
        let gate = Arc::new(ActivationGate::new());
        gate.activate();

        let inner = Arc::new(DummyPublisher::new());
        let pub_ = ManagedPublisher::<String, _>::new(gate, inner.clone());

        let ok = pub_.publish(&"hello".to_string()).await.unwrap();
        assert!(ok);
        assert_eq!(inner.calls(), 1);
    }
}
