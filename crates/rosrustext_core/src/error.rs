use std::borrow::Cow;
use std::fmt;
use thiserror::Error;

/// Convenient result alias for rosrustext_core.
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
    Transport,
    Io,
    ProtocolViolation,
    Timeout,
    Other,
}

/// Optional structured payload for rich context without forcing allocation.
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

/// The one error type that crosses module boundaries in rosrustext_core.
#[derive(Debug, Error, Clone, Eq, PartialEq)]
#[error("{severity:?}: {message}")]
pub struct CoreError {
    pub domain: Domain,
    pub kind: ErrorKind,
    pub severity: Severity,
    pub message: Cow<'static, str>,
    pub payload: Payload,
}

impl CoreError {
    /// Fully-specified constructor (rarely needed at call sites).
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

    // ---------------- Fluent entry points ----------------

    #[inline]
    pub fn trace() -> ErrB {
        ErrB::new(Severity::Trace)
    }
    #[inline]
    pub fn debug() -> ErrB {
        ErrB::new(Severity::Debug)
    }
    #[inline]
    pub fn info() -> ErrB {
        ErrB::new(Severity::Info)
    }
    #[inline]
    pub fn warn() -> ErrB {
        ErrB::new(Severity::Warn)
    }
    #[inline]
    pub fn error() -> ErrB {
        ErrB::new(Severity::Error)
    }
    #[inline]
    pub fn fatal() -> ErrB {
        ErrB::new(Severity::Fatal)
    }

    /// Construct a lifecycle InvalidTransition error with structured context.
    pub fn invalid_transition_lifecycle(from_state: u8, via_transition: u8) -> Self {
        CoreError::warn()
            .domain(Domain::Lifecycle)
            .kind(ErrorKind::InvalidTransition)
            .msg("invalid lifecycle transition")
            .payload(Payload::LifecycleTransition {
                from_state,
                via_transition,
            })
            .build()
    }
}

/// Fluent builder that behaves like iterator chains (takes self, returns Self).
/// Defaults:
/// - domain = Other
/// - kind = Other
/// - message = ""
/// - payload = None
#[derive(Debug, Clone)]
pub struct ErrB {
    domain: Domain,
    kind: ErrorKind,
    severity: Severity,
    message: Cow<'static, str>,
    payload: Payload,
}

impl ErrB {
    #[inline]
    fn new(severity: Severity) -> Self {
        Self {
            domain: Domain::Other,
            kind: ErrorKind::Other,
            severity,
            message: Cow::Borrowed(""),
            payload: Payload::None,
        }
    }

    // -------- Guided setters --------

    /// Set/override the domain (defaults to Domain::Other).
    #[inline]
    pub fn domain(mut self, d: Domain) -> Self {
        self.domain = d;
        self
    }

    /// Set/override the kind (defaults to ErrorKind::Other).
    #[inline]
    pub fn kind(mut self, k: ErrorKind) -> Self {
        self.kind = k;
        self
    }

    /// Set/override the message (defaults to "").
    #[inline]
    pub fn msg(mut self, m: impl Into<Cow<'static, str>>) -> Self {
        self.message = m.into();
        self
    }

    /// Formatting-friendly message setter.
    /// Note: still allocates once because we store as Cow<'static, str>.
    #[inline]
    pub fn msgf(mut self, args: fmt::Arguments<'_>) -> Self {
        self.message = Cow::Owned(args.to_string());
        self
    }

    /// Only one payload: this replaces any previous payload (default is None).
    #[inline]
    pub fn payload(mut self, p: Payload) -> Self {
        self.payload = p;
        self
    }

    // -------- Finish --------
    #[inline]
    pub fn build(self) -> CoreError {
        CoreError {
            domain: self.domain,
            kind: self.kind,
            severity: self.severity,
            message: self.message,
            payload: self.payload,
        }
    }
}

// Optional convenience: allow `.into()` from builder if you like.
impl From<ErrB> for CoreError {
    fn from(b: ErrB) -> Self {
        b.build()
    }
}

impl From<std::io::Error> for CoreError {
    fn from(e: std::io::Error) -> Self {
        CoreError::error()
            .domain(Domain::Other)
            .kind(ErrorKind::Io)
            .msg("io error")
            .payload(Payload::Context {
                key: "io",
                value: e.to_string().into(),
            })
            .build()
    }
}
