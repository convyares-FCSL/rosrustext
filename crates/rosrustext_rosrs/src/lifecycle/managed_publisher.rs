use std::sync::Arc;

use crate::error::Result;
use rosrustext_core::lifecycle::ActivationGate;

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
/// Outcome of a managed publish attempt.
///
/// # Semantics
/// - [`PublishOutcome::Published`]: the activation gate was active and the message
///   was passed through to the underlying `rclrs::Publisher`.
/// - [`PublishOutcome::SuppressedInactive`]: the activation gate was inactive and
///   the message was dropped (no ROS publish occurs).
///
/// # Errors
/// This type does not represent transport errors. Any error from the underlying
/// `rclrs::Publisher::publish` is returned by
/// [`ManagedPublisher::publish_with_outcome`].
///
/// # Example
/// ```rust,ignore
/// match publisher.publish_with_outcome(msg)? {
///     PublishOutcome::Published => {}
///     PublishOutcome::SuppressedInactive => {
///         // Node is not Active; message was intentionally suppressed.
///     }
/// }
/// ```
///
/// # See also
/// - [`ManagedPublisher`]
pub enum PublishOutcome {
    Published,
    SuppressedInactive,
}

/// A lifecycle-gated publisher wrapper.
///
/// # Semantics
/// - While the lifecycle is `Active`, publishes are forwarded to the underlying
///   `rclrs::Publisher`.
/// - While not `Active`, publishes are *suppressed* (silently dropped).
///
/// This behavior matches ROS 2 “managed entity” expectations: data-plane output
/// is stopped while inactive.
///
/// # Errors
/// - [`ManagedPublisher::publish`] and [`ManagedPublisher::publish_with_outcome`]
///   return an error only when the gate is active and the underlying publish call
///   fails.
///
/// # Example
/// ```rust,ignore
/// let publisher = lifecycle.create_publisher::<rosrustext_rosrs::lifecycle_msgs::msg::State>("state")?;
/// publisher.publish(rosrustext_rosrs::lifecycle_msgs::msg::State::default())?;
/// ```
///
/// # See also
/// - [`crate::lifecycle::LifecycleNode::create_publisher`]
/// - [Lifecycle spec](https://github.com/convyares-FCSL/rosrustext_fcsl/blob/main/docs/spec/lifecycle.md)
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

    /// Publish a message, suppressing silently while inactive.
    ///
    /// # Semantics
    /// - If the lifecycle is active, publishes the message.
    /// - If the lifecycle is inactive, drops the message and returns `Ok(())`.
    ///
    /// If you need to know whether the message was suppressed, use
    /// [`ManagedPublisher::publish_with_outcome`].
    ///
    /// # Errors
    /// Returns an error only when the gate is active and the underlying publish
    /// call fails.
    ///
    /// # Example
    /// ```rust,ignore
    /// publisher.publish(msg)?;
    /// ```
    ///
    /// # See also
    /// - [`ManagedPublisher::publish_with_outcome`]
    pub fn publish(&self, msg: T) -> Result<()> {
        let _ = self.publish_with_outcome(msg)?;
        Ok(())
    }

    /// Publish a message and report whether it was gated.
    ///
    /// # Semantics
    /// - Returns [`PublishOutcome::Published`] if the message was published.
    /// - Returns [`PublishOutcome::SuppressedInactive`] if the lifecycle is inactive.
    ///
    /// # Errors
    /// Returns an error only when the gate is active and the underlying publish
    /// call fails.
    ///
    /// # Example
    /// ```rust,ignore
    /// if publisher.publish_with_outcome(msg)? == PublishOutcome::SuppressedInactive {
    ///     // Consider buffering or dropping at a higher level.
    /// }
    /// ```
    ///
    /// # See also
    /// - [`PublishOutcome`]
    pub fn publish_with_outcome(&self, msg: T) -> Result<PublishOutcome> {
        if self.gate.is_active() {
            self.inner.publish(msg)?;
            Ok(PublishOutcome::Published)
        } else {
            Ok(PublishOutcome::SuppressedInactive)
        }
    }

    /// Access the underlying `rclrs::Publisher`.
    ///
    /// # Semantics
    /// This is an escape hatch. Publishing via the returned publisher bypasses
    /// lifecycle gating.
    ///
    /// # Errors
    /// This method does not fail.
    ///
    /// # Example
    /// ```rust,ignore
    /// let raw = publisher.inner();
    /// ```
    ///
    /// # See also
    /// - [`crate::lifecycle::LifecycleNode::node_arc`]
    pub fn inner(&self) -> &rclrs::Publisher<T> {
        &self.inner
    }
}
