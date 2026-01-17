/// A handle that keeps a lifecycle-gated timer alive.
///
/// # Semantics
/// `ManagedTimer` is returned by [`crate::lifecycle::LifecycleNode::timer_repeating`]
/// via `.callback(...).create()`.
/// Keeping the handle alive keeps the underlying `rclrs::Timer` installed.
///
/// The gating semantics are implemented by the creator (`LifecycleNode`): while
/// the lifecycle is inactive, timer ticks occur but the user callback is skipped.
///
/// # Errors
/// `ManagedTimer` itself does not produce errors; timer creation errors are
/// returned by [`crate::lifecycle::LifecycleNode::timer_repeating`].
///
/// # Example
/// ```rust,ignore
/// let timer = lifecycle
///     .timer_repeating(std::time::Duration::from_secs(1))
///     .callback(|| {
///         // Runs only while Active
///     })
///     .create()?;
/// drop(timer); // stops the timer by dropping the handle
/// ```
///
/// # See also
/// - [`crate::lifecycle::LifecycleNode::timer_repeating`]
pub struct ManagedTimer {
    _inner: rclrs::Timer,
}

impl ManagedTimer {
    pub(crate) fn new(inner: rclrs::Timer) -> Self {
        Self { _inner: inner }
    }
}
