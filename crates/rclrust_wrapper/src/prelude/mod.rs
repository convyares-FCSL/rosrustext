pub mod prelude {
    // lifecycle façade types you want app code to use
    pub use crate::lifecycle::{
        ActivationGate, LifecycleNode, ManagedInterval, ManagedPublisher, PublishLike,
    };

    // re-export callbacks + result so apps never import rclrust_core
    pub use rclrust_core::lifecycle::{CallbackResult, LifecycleCallbacks};

    // optional: if you want State exposed too
    pub use rclrust_core::lifecycle::State;

    // optional roslibrust adapter type behind feature
    #[cfg(feature = "roslibrust")]
    pub use crate::transport::roslibrust::publisher::RosbridgePublisher;
}
