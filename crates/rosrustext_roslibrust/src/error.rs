use rosrustext_core::error::{CoreError, Severity};

pub fn log_core_error(err: CoreError) {
    match err.severity {
        Severity::Trace => tracing::trace!("{err}"),
        Severity::Debug => tracing::debug!("{err}"),
        Severity::Info => tracing::info!("{err}"),
        Severity::Warn => tracing::warn!("{err}"),
        Severity::Error | Severity::Fatal => tracing::error!("{err}"),
    }
}
