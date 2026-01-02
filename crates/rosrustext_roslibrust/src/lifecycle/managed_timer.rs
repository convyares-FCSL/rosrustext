use std::sync::Arc;
use std::time::Duration;

use rosrustext_core::lifecycle::ActivationGate;

/// A lifecycle-gated periodic runner.
///
/// Parity goal with rclcpp_lifecycle timers:
/// - the timer can keep “ticking”, but user work only runs when Active
/// - no ROS transport dependency
///
/// Notes:
/// - This is async and uses tokio because your roslibrust nodes already run under tokio.
/// - Wrapper policy stays simple: inactive => skip the tick body.
pub struct ManagedInterval {
    gate: Arc<ActivationGate>,
    period: Duration,
}

impl ManagedInterval {
    pub fn new(gate: Arc<ActivationGate>, period: Duration) -> Self {
        Self { gate, period }
    }

    /// Run `tick()` forever, but only execute it when the lifecycle gate is active.
    pub async fn run<F, Fut>(&self, mut tick: F) -> !
    where
        F: FnMut() -> Fut + Send,
        Fut: std::future::Future<Output = ()> + Send,
    {
        let mut interval = tokio::time::interval(self.period);

        loop {
            interval.tick().await;

            if self.gate.is_active() {
                tick().await;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering};

    #[tokio::test]
    async fn interval_skips_when_inactive_and_runs_when_active() {
        let gate = Arc::new(ActivationGate::new());
        let interval = ManagedInterval::new(gate.clone(), Duration::from_millis(10));
        let hits = Arc::new(AtomicUsize::new(0));

        // Spawn the runner.
        let hits2 = hits.clone();
        let handle = tokio::spawn(async move {
            interval
                .run(|| {
                    let hits3 = hits2.clone();
                    async move {
                        hits3.fetch_add(1, Ordering::Relaxed);
                    }
                })
                .await
        });

        // Let it tick while inactive.
        tokio::time::sleep(Duration::from_millis(35)).await;
        assert_eq!(hits.load(Ordering::Relaxed), 0);

        // Activate and verify it runs.
        gate.activate();
        tokio::time::sleep(Duration::from_millis(35)).await;
        assert!(hits.load(Ordering::Relaxed) > 0);

        // Cleanup the task (it's infinite).
        handle.abort();
    }
}
