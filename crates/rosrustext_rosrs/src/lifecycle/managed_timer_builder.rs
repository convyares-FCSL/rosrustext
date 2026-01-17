use std::marker::PhantomData;
use std::sync::Arc;
use std::time::Duration;

use rclrs::{AnyTimerCallback, Clock, IntoNodeTimerRepeatingCallback, RclrsError, TimerClock, TimerOptions};

use super::{ActivationGate, ClockChoice, HasCallback, ManagedTimer, NoCallback};

/// Builder for lifecycle-managed repeating timers.
///
/// This owns all configuration data; no borrowed references are retained.
pub struct ManagedTimerBuilder<State> {
    node: Arc<rclrs::Node>,
    gate: Arc<ActivationGate>,
    period: Duration,
    clock: ClockChoice,
    callback: Option<AnyTimerCallback<rclrs::Node>>,
    _phantom: PhantomData<State>,
}

impl ManagedTimerBuilder<NoCallback> {
    pub(crate) fn new(node: Arc<rclrs::Node>, gate: Arc<ActivationGate>, period: Duration) -> Self {
        Self { node, gate, period, clock: ClockChoice::Steady, callback: None, _phantom: PhantomData }
    }
}

impl<State> ManagedTimerBuilder<State> {
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
    pub fn callback<Args>(self, cb: impl IntoNodeTimerRepeatingCallback<Args>) -> ManagedTimerBuilder<HasCallback> {
        let erased = cb.into_node_timer_repeating_callback();
        ManagedTimerBuilder {
            node: self.node,
            gate: self.gate,
            period: self.period,
            clock: self.clock,
            callback: Some(erased),
            _phantom: PhantomData,
        }
    }
}

impl ManagedTimerBuilder<HasCallback> {
    /// Create the managed repeating timer.
    pub fn create(self) -> Result<ManagedTimer, RclrsError> {
        let ManagedTimerBuilder { node, gate, period, clock, callback, _phantom: _ } = self;

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

        let gated = wrap_gated_callback(callback, gate);
        let adapter = ErasedTimerCallback::new(gated);

        let timer = node.create_timer_repeating(options, adapter)?;
        Ok(ManagedTimer::new(timer))
    }
}

fn wrap_gated_callback(
    callback: AnyTimerCallback<rclrs::Node>, gate: Arc<ActivationGate>,
) -> AnyTimerCallback<rclrs::Node> {
    match callback {
        AnyTimerCallback::Repeating(mut cb) => AnyTimerCallback::Repeating(Box::new(move |payload, timer| {
            if gate.is_active() {
                cb(payload, timer);
            }
        })),
        AnyTimerCallback::OneShot(_) | AnyTimerCallback::Inert => {
            debug_assert!(false, "ManagedTimerBuilder expects repeating callbacks from IntoNodeTimerRepeatingCallback");
            AnyTimerCallback::Repeating(Box::new(move |_, _| {
                if gate.is_active() {
                    // No-op: repeating builder only supports repeating callbacks.
                }
            }))
        }
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
