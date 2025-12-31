use std::sync::Arc;

use crate::lifecycle::PublishLike;

/// Thin adapter so we can implement `PublishLike` without orphan-rule issues.
pub struct RosbridgePublisher<T: roslibrust::RosMessageType>(
    pub Arc<roslibrust::rosbridge::Publisher<T>>,
);

impl<T> PublishLike<T> for RosbridgePublisher<T>
where
    T: roslibrust::RosMessageType + Send + Sync + 'static,
{
    type Error = roslibrust::Error;

    fn publish<'a>(
        &'a self,
        msg: &'a T,
    ) -> std::pin::Pin<Box<dyn std::future::Future<Output = Result<(), Self::Error>> + Send + 'a>> {
        Box::pin(async move { self.0.publish(msg).await })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn _assert_publish_like<T>()
    where
        T: roslibrust::RosMessageType + Send + Sync + 'static,
        RosbridgePublisher<T>: crate::lifecycle::PublishLike<T>,
    {
    }

    #[test]
    fn compile_only_publishlike_impl_exists() {
        _assert_publish_like::<roslibrust::ShapeShifter>();
    }
}
