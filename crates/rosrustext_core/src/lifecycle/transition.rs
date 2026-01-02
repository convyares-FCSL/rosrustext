/// ROS2-style lifecycle transitions (requests).
///
/// These are the **user-invoked** transitions. All "ON_*_SUCCESS/FAILURE/ERROR"
/// transitions are modeled via `finish(intermediate, via, CallbackResult)`.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum Transition {
    Configure,
    Activate,
    Deactivate,
    Cleanup,
    Shutdown,
}

/// Internal, compact IDs used for error payloads.
///
/// These are **not** the ROS `lifecycle_msgs/msg/Transition` IDs.
/// The wrapper layer performs ROS ID mapping.
impl Transition {
    pub const fn id(self) -> u8 {
        match self {
            Transition::Configure => 1,
            Transition::Cleanup => 2,
            Transition::Activate => 3,
            Transition::Deactivate => 4,
            Transition::Shutdown => 5,
        }
    }

    /// Stable, human-readable label for ROS-facing adapters.
    pub const fn label(self) -> &'static str {
        match self {
            Transition::Configure => "configure",
            Transition::Cleanup => "cleanup",
            Transition::Activate => "activate",
            Transition::Deactivate => "deactivate",
            Transition::Shutdown => "shutdown",
        }
    }
}
