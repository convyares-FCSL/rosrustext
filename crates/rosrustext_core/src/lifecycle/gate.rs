use std::sync::atomic::{AtomicBool, Ordering};

/// Simple activation gate for managed resources.
///
/// Intended use (wrapper layer):
/// - set `activate()` when state becomes Active
/// - set `deactivate()` when leaving Active
/// - publisher/timer wrappers check `is_active()` to allow or block work
#[derive(Debug)]
pub struct ActivationGate {
    active: AtomicBool,
}

impl ActivationGate {
    pub const fn new() -> Self {
        Self {
            active: AtomicBool::new(false),
        }
    }

    pub fn activate(&self) {
        self.active.store(true, Ordering::Release);
    }

    pub fn deactivate(&self) {
        self.active.store(false, Ordering::Release);
    }

    pub fn is_active(&self) -> bool {
        self.active.load(Ordering::Acquire)
    }
}

impl Default for ActivationGate {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn activation_gate_test() {
        let gate = ActivationGate::new();

        assert!(!gate.is_active());

        gate.activate();
        assert!(gate.is_active());

        gate.deactivate();
        assert!(!gate.is_active());
    }
}
