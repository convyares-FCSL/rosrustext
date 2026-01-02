/// ROS2-style lifecycle primary + transition (intermediate) states.
///
/// Primary (stable) states:
/// - Unconfigured, Inactive, Active, Finalized
///
/// Transition (intermediate) states:
/// - Configuring, CleaningUp, Activating, Deactivating, ShuttingDown, ErrorProcessing
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum State {
    // Primary
    Unconfigured,
    Inactive,
    Active,
    Finalized,

    // Transition (intermediate)
    Configuring,
    CleaningUp,
    Activating,
    Deactivating,
    ShuttingDown,
    ErrorProcessing,
}

/// Internal, compact IDs used for error payloads.
///
/// These are **not** ROS message IDs. They are stable, lightweight identifiers for
/// debugging/telemetry inside `rosrustext_core`.
impl State {
    pub const fn id(self) -> u8 {
        match self {
            // Primary
            State::Unconfigured => 0,
            State::Inactive => 1,
            State::Active => 2,
            State::Finalized => 3,

            // Transition
            State::Configuring => 10,
            State::CleaningUp => 11,
            State::Activating => 12,
            State::Deactivating => 13,
            State::ShuttingDown => 14,
            State::ErrorProcessing => 15,
        }
    }

    /// True for stable (externally targetable) states.
    pub const fn is_primary(self) -> bool {
        matches!(
            self,
            State::Unconfigured | State::Inactive | State::Active | State::Finalized
        )
    }

    /// True for intermediate states entered while callbacks are running.
    pub const fn is_transitioning(self) -> bool {
        !self.is_primary()
    }

    /// Stable, human-readable label for ROS-facing adapters.
    pub const fn label(self) -> &'static str {
        match self {
            State::Unconfigured => "Unconfigured",
            State::Inactive => "Inactive",
            State::Active => "Active",
            State::Finalized => "Finalized",
            State::Configuring => "Configuring",
            State::CleaningUp => "CleaningUp",
            State::Activating => "Activating",
            State::Deactivating => "Deactivating",
            State::ShuttingDown => "ShuttingDown",
            State::ErrorProcessing => "ErrorProcessing",
        }
    }
}

/// Canonical list of all lifecycle states (primary + transition).
pub const ALL_STATES: [State; 10] = [
    State::Unconfigured,
    State::Inactive,
    State::Active,
    State::Finalized,
    State::Configuring,
    State::CleaningUp,
    State::Activating,
    State::Deactivating,
    State::ShuttingDown,
    State::ErrorProcessing,
];
