/// Timer wrapper whose existence keeps the timer alive.
pub struct ManagedTimer {
    _inner: rclrs::Timer,
}

impl ManagedTimer {
    pub(crate) fn new(inner: rclrs::Timer) -> Self {
        Self { _inner: inner }
    }
}
