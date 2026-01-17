/// Marker type: builder has no callback set yet.
#[derive(Debug, Copy, Clone)]
pub struct NoCallback;

/// Marker type: builder has a callback set.
#[derive(Debug, Copy, Clone)]
pub struct HasCallback;

/// Owned clock selection for timer builders.
#[derive(Debug, Clone)]
pub enum ClockChoice {
    Steady,
    System,
    Node,
    Custom(rclrs::Clock),
}
