use rclrust_core::lifecycle::ActivationGate;

/// Execute a closure only when the lifecycle gate is active.
///
/// Intended for publisher/timer wrappers:
/// - return `true` if executed
/// - return `false` if suppressed
pub fn run_if_active<F>(gate: &ActivationGate, f: F) -> bool
where
    F: FnOnce(),
{
    if gate.is_active() {
        f();
        true
    } else {
        false
    }
}
