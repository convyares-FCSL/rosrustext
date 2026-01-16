/// A handle that keeps a lifecycle-gated timer alive.
///
/// # Semantics
/// `ManagedTimer` is returned by [`crate::lifecycle::LifecycleNode::create_timer_repeating_gated`].
/// Keeping the handle alive keeps the underlying `rclrs::Timer` installed.
///
/// The gating semantics are implemented by the creator (`LifecycleNode`): while
/// the lifecycle is inactive, timer ticks occur but the user callback is skipped.
///
/// # Errors
/// `ManagedTimer` itself does not produce errors; timer creation errors are
/// returned by [`crate::lifecycle::LifecycleNode::create_timer_repeating_gated`].
///
/// # Example
/// ```rust,ignore
/// let timer = lifecycle.create_timer_repeating_gated(std::time::Duration::from_secs(1), || {
///     // Runs only while Active
/// })?;
/// drop(timer); // stops the timer by dropping the handle
/// ```
///
/// # See also
/// - [`crate::lifecycle::LifecycleNode::create_timer_repeating_gated`]
pub struct ManagedTimer {
    _inner: rclrs::Timer,
}

impl ManagedTimer {
    pub(crate) fn new(inner: rclrs::Timer) -> Self {
        Self { _inner: inner }
    }
}
