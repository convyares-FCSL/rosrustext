//! Builder-style APIs for raw `rclrs::Node` resources.
//!
//! # Examples
//! Managed (lifecycle-gated):
//! ```rust,no_run
//! use rclrs::{Context, CreateBasicExecutor};
//! use rosrustext_rosrs::lifecycle::LifecycleNode;
//! use rosrustext_rosrs::lifecycle_msgs::msg::State;
//!
//! # fn main() -> rosrustext_rosrs::Result<()> {
//! let context = Context::default();
//! let executor = context.create_basic_executor();
//! let lifecycle = LifecycleNode::create(&executor, "demo")?;
//! let _pub = lifecycle.publisher::<State>("state").create()?;
//! # Ok(()) }
//! ```
//!
//! Raw (non-managed):
//! ```rust,no_run
//! use rclrs::{Context, CreateBasicExecutor};
//! use rosrustext_rosrs::NodeBuilderExt;
//! use rosrustext_rosrs::lifecycle_msgs::msg::State;
//!
//! # fn main() -> rosrustext_rosrs::Result<()> {
//! let context = Context::default();
//! let executor = context.create_basic_executor();
//! let node = executor.create_node("demo")?;
//! let _pub = node.publisher::<State>("state").create()?;
//! # Ok(()) }
//! ```

use std::borrow::Cow;
use std::time::Duration;

use super::{NoCallback, PublisherBuilder, SubscriptionBuilder, TimerBuilder};

/// Extension trait to build raw `rclrs::Node` resources using builder-style APIs.
pub trait NodeBuilderExt {
    fn publisher<'a, T>(&self, topic: impl Into<Cow<'a, str>>) -> PublisherBuilder<T>
    where
        T: rclrs::MessageIDL;

    fn subscription<'a, T>(&self, topic: impl Into<Cow<'a, str>>) -> SubscriptionBuilder<T, NoCallback>
    where
        T: rclrs::MessageIDL;

    fn timer_repeating(&self, period: Duration) -> TimerBuilder<NoCallback>;
}

impl NodeBuilderExt for rclrs::Node {
    fn publisher<'a, T>(&self, topic: impl Into<Cow<'a, str>>) -> PublisherBuilder<T>
    where
        T: rclrs::MessageIDL,
    {
        PublisherBuilder::new(self.clone(), topic)
    }

    fn subscription<'a, T>(&self, topic: impl Into<Cow<'a, str>>) -> SubscriptionBuilder<T, NoCallback>
    where
        T: rclrs::MessageIDL,
    {
        SubscriptionBuilder::new(self.clone(), topic)
    }

    fn timer_repeating(&self, period: Duration) -> TimerBuilder<NoCallback> {
        TimerBuilder::new(self.clone(), period)
    }
}
