use std::borrow::Cow;
use std::marker::PhantomData;
use std::sync::Arc;

use rclrs::{PublisherOptions, QoSProfile, RclrsError};

use super::{ActivationGate, ManagedPublisher};

/// Builder for lifecycle-managed publishers.
///
/// This owns all configuration data; no borrowed references are retained.
pub struct ManagedPublisherBuilder<T>
where
    T: rclrs::MessageIDL,
{
    node: Arc<rclrs::Node>,
    gate: Arc<ActivationGate>,
    topic: String,
    qos: QoSProfile,
    _phantom: PhantomData<T>,
}

impl<T> ManagedPublisherBuilder<T>
where
    T: rclrs::MessageIDL,
{
    pub(crate) fn new<'a>(node: Arc<rclrs::Node>, gate: Arc<ActivationGate>, topic: impl Into<Cow<'a, str>>) -> Self {
        Self { node, gate, topic: topic.into().into_owned(), qos: QoSProfile::topics_default(), _phantom: PhantomData }
    }

    /// Replace the topic name.
    pub fn topic<'a>(mut self, topic: impl Into<Cow<'a, str>>) -> Self {
        self.topic = topic.into().into_owned();
        self
    }

    /// Replace the QoS profile entirely.
    pub fn qos(mut self, profile: QoSProfile) -> Self {
        self.qos = profile;
        self
    }

    /// Replace topic + QoS from rclrs options.
    pub fn with_options<'a>(mut self, options: PublisherOptions<'a>) -> Self {
        self.topic = options.topic.to_string();
        self.qos = options.qos;
        self
    }

    /// Set reliability to Reliable.
    pub fn reliable(mut self) -> Self {
        self.qos = self.qos.reliable();
        self
    }

    /// Set reliability to BestEffort.
    pub fn best_effort(mut self) -> Self {
        self.qos = self.qos.best_effort();
        self
    }

    /// Set durability to TransientLocal.
    pub fn transient_local(mut self) -> Self {
        self.qos = self.qos.transient_local();
        self
    }

    /// Set durability to Volatile.
    pub fn volatile(mut self) -> Self {
        self.qos = self.qos.volatile();
        self
    }

    /// Set history to KeepLast(depth).
    pub fn history_keep_last(mut self, depth: u32) -> Self {
        self.qos = self.qos.keep_last(depth);
        self
    }

    /// Set or clear avoid_ros_namespace_conventions.
    pub fn avoid_ros_namespace_conventions(mut self, avoid: bool) -> Self {
        self.qos.avoid_ros_namespace_conventions = avoid;
        self
    }

    /// Create the managed publisher.
    pub fn create(self) -> Result<ManagedPublisher<T>, RclrsError> {
        let mut options = PublisherOptions::new(&self.topic);
        options.qos = self.qos;
        let pub_ = self.node.create_publisher::<T>(options)?;
        Ok(ManagedPublisher::new(pub_, Arc::clone(&self.gate)))
    }
}
