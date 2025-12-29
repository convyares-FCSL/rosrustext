use std::borrow::Cow;
use thiserror::Error;

/// Convenient result alias for rclrust_core.
pub type Result<T> = std::result::Result<T, CoreError>;

/// Log/handling importance. Intended to map cleanly onto logging levels later.
#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd)]
pub enum Severity {
    Trace,
    Debug,
    Info,
    Warn,
    Error,
    Fatal,
}

/// Where an error came from (helps triage and routing).
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum Domain {
    Lifecycle,
    Action,
    Config,
    Logging,
    Transport,
    Other,
}

/// Stable error "kind" for matching/branching.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum ErrorKind {
    InvalidArgument,
    InvalidState,
    InvalidTransition,
    NotSupported,
    Io,
    ProtocolViolation,
    Timeout,
    Other,
}

/// Optional structured payload for rich context without forcing allocation.
///
/// This is intentionally small and copy-friendly.
/// If you later need “anything”, add a new enum variant rather than boxing.
#[derive(Debug, Clone, Eq, PartialEq)]
pub enum Payload {
    None,

    /// Generic key/value context (usually no heap alloc if using &str).
    Context {
        key: &'static str,
        value: Cow<'static, str>,
    },

    /// Lifecycle-specific context.
    LifecycleTransition {
        from_state: u8,
        via_transition: u8,
    },

    /// Arbitrary numeric detail (e.g., error codes from external libs).
    Code(u32),
}

impl Default for Payload {
    fn default() -> Self {
        Payload::None
    }
}

/// The one error type that crosses module boundaries in rclrust_core.
#[derive(Debug, Error, Clone, Eq, PartialEq)]
#[error("{kind:?} ({domain:?}, {severity:?})")]
pub struct CoreError {
    pub domain: Domain,
    pub kind: ErrorKind,
    pub severity: Severity,
    pub message: Cow<'static, str>,
    pub payload: Payload,
}

impl CoreError {
    /// Create a new error with minimal boilerplate.
    pub fn new(
        domain: Domain,
        kind: ErrorKind,
        severity: Severity,
        message: impl Into<Cow<'static, str>>,
    ) -> Self {
        Self {
            domain,
            kind,
            severity,
            message: message.into(),
            payload: Payload::None,
        }
    }

    /// Attach/replace payload (structured context).
    pub fn with_payload(mut self, payload: Payload) -> Self {
        self.payload = payload;
        self
    }

    /// Construct a lifecycle InvalidTransition error with structured context.
    /// Intended for state-machine enforcement, not user input validation.
    pub fn invalid_transition_lifecycle(from_state: u8, via_transition: u8) -> Self {
        Self::new(
            Domain::Lifecycle,
            ErrorKind::InvalidTransition,
            Severity::Warn,
            "invalid lifecycle transition",
        )
        .with_payload(Payload::LifecycleTransition {
            from_state,
            via_transition,
        })
    }
}

impl From<std::io::Error> for CoreError {
    fn from(_: std::io::Error) -> Self {
        CoreError::new(Domain::Other, ErrorKind::Io, Severity::Error, "io error")
    }
}
