use std::borrow::Cow;
use std::marker::PhantomData;

use rclrs::{AnySubscriptionCallback, IntoNodeSubscriptionCallback, QoSProfile, RclrsError, SubscriptionOptions};

use super::{HasCallback, NoCallback};

/// Builder for non-managed subscriptions on an `rclrs::Node`.
///
/// This owns all configuration data; no borrowed references are retained.
pub struct SubscriptionBuilder<T, State>
where
    T: rclrs::MessageIDL,
{
    node: rclrs::Node,
    topic: String,
    qos: QoSProfile,
    callback: Option<AnySubscriptionCallback<T, ()>>,
    _phantom: PhantomData<State>,
}

impl<T> SubscriptionBuilder<T, NoCallback>
where
    T: rclrs::MessageIDL,
{
    pub(crate) fn new<'a>(node: rclrs::Node, topic: impl Into<Cow<'a, str>>) -> Self {
        Self {
            node,
            topic: topic.into().into_owned(),
            qos: QoSProfile::topics_default(),
            callback: None,
            _phantom: PhantomData,
        }
    }
}

impl<T, State> SubscriptionBuilder<T, State>
where
    T: rclrs::MessageIDL,
{
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
    pub fn with_options<'a>(mut self, options: SubscriptionOptions<'a>) -> Self {
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

    /// Set the subscription callback (erased and owned).
    pub fn callback<Args>(self, cb: impl IntoNodeSubscriptionCallback<T, Args>) -> SubscriptionBuilder<T, HasCallback> {
        let erased = cb.into_node_subscription_callback();
        SubscriptionBuilder {
            node: self.node,
            topic: self.topic,
            qos: self.qos,
            callback: Some(erased),
            _phantom: PhantomData,
        }
    }
}

impl<T> SubscriptionBuilder<T, HasCallback>
where
    T: rclrs::MessageIDL,
{
    /// Create the subscription.
    pub fn create(self) -> Result<rclrs::Subscription<T>, RclrsError> {
        let Some(callback) = self.callback else {
            unreachable!("callback must be set in HasCallback state");
        };

        let mut options = SubscriptionOptions::new(&self.topic);
        options.qos = self.qos;

        let adapter = ErasedSubscriptionCallback::new(callback);
        self.node.create_subscription::<T, _>(options, adapter)
    }
}

struct ErasedSubscriptionCallback<T>
where
    T: rclrs::MessageIDL,
{
    callback: AnySubscriptionCallback<T, ()>,
}

impl<T> ErasedSubscriptionCallback<T>
where
    T: rclrs::MessageIDL,
{
    fn new(callback: AnySubscriptionCallback<T, ()>) -> Self {
        Self { callback }
    }
}

impl<T> IntoNodeSubscriptionCallback<T, ()> for ErasedSubscriptionCallback<T>
where
    T: rclrs::MessageIDL,
{
    fn into_node_subscription_callback(self) -> AnySubscriptionCallback<T, ()> {
        self.callback
    }
}
