use std::marker::PhantomData;
use std::time::Duration;

use rclrs::{AnyTimerCallback, Clock, IntoNodeTimerRepeatingCallback, RclrsError, TimerClock, TimerOptions};

use super::{HasCallback, NoCallback};

/// Builder for non-managed repeating timers on an `rclrs::Node`.
///
/// This owns all configuration data; no borrowed references are retained.
pub struct TimerBuilder<State> {
    node: rclrs::Node,
    period: Duration,
    clock: ClockChoice,
    callback: Option<AnyTimerCallback<rclrs::Node>>,
    _phantom: PhantomData<State>,
}

#[derive(Debug, Clone)]
enum ClockChoice {
    Steady,
    System,
    Node,
    Custom(Clock),
}

impl TimerBuilder<NoCallback> {
    pub(crate) fn new(node: rclrs::Node, period: Duration) -> Self {
        Self { node, period, clock: ClockChoice::Steady, callback: None, _phantom: PhantomData }
    }
}

impl<State> TimerBuilder<State> {
    /// Replace period + clock from rclrs options.
    pub fn with_options<'a>(mut self, options: TimerOptions<'a>) -> Self {
        self.period = options.period;
        self.clock = match options.clock {
            TimerClock::SteadyTime => ClockChoice::Steady,
            TimerClock::SystemTime => ClockChoice::System,
            TimerClock::NodeTime => ClockChoice::Node,
            TimerClock::Clock(clock) => ClockChoice::Custom(clock.clone()),
        };
        self
    }

    /// Use steady time (default).
    pub fn steady_time(mut self) -> Self {
        self.clock = ClockChoice::Steady;
        self
    }

    /// Use system time.
    pub fn system_time(mut self) -> Self {
        self.clock = ClockChoice::System;
        self
    }

    /// Use node time.
    pub fn node_time(mut self) -> Self {
        self.clock = ClockChoice::Node;
        self
    }

    /// Use a specific clock.
    pub fn clock(mut self, clock: &Clock) -> Self {
        self.clock = ClockChoice::Custom(clock.clone());
        self
    }

    /// Set the repeating callback (erased and owned).
    pub fn callback<Args>(self, cb: impl IntoNodeTimerRepeatingCallback<Args>) -> TimerBuilder<HasCallback> {
        let erased = cb.into_node_timer_repeating_callback();
        TimerBuilder {
            node: self.node,
            period: self.period,
            clock: self.clock,
            callback: Some(erased),
            _phantom: PhantomData,
        }
    }
}

impl TimerBuilder<HasCallback> {
    /// Create the repeating timer.
    pub fn create(self) -> Result<rclrs::Timer, RclrsError> {
        let TimerBuilder { node, period, clock, callback, _phantom: _ } = self;

        let Some(callback) = callback else {
            unreachable!("callback must be set in HasCallback state");
        };

        let mut options = TimerOptions::new(period);
        options.clock = match &clock {
            ClockChoice::Steady => TimerClock::SteadyTime,
            ClockChoice::System => TimerClock::SystemTime,
            ClockChoice::Node => TimerClock::NodeTime,
            ClockChoice::Custom(clock) => TimerClock::Clock(clock),
        };

        let adapter = ErasedTimerCallback::new(callback);
        node.create_timer_repeating(options, adapter)
    }
}

struct ErasedTimerCallback {
    callback: AnyTimerCallback<rclrs::Node>,
}

impl ErasedTimerCallback {
    fn new(callback: AnyTimerCallback<rclrs::Node>) -> Self {
        Self { callback }
    }
}

impl IntoNodeTimerRepeatingCallback<()> for ErasedTimerCallback {
    fn into_node_timer_repeating_callback(self) -> AnyTimerCallback<rclrs::Node> {
        self.callback
    }
}
