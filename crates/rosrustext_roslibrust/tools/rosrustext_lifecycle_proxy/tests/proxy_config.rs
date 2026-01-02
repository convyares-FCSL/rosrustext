use std::env;
use std::sync::{Mutex, OnceLock};

use rosrustext_lifecycle_proxy::config::Config;

fn env_lock() -> std::sync::MutexGuard<'static, ()> {
    static LOCK: OnceLock<Mutex<()>> = OnceLock::new();
    LOCK.get_or_init(|| Mutex::new(())).lock().expect("lock")
}

#[test]
fn no_bond_flag_disables_bond() {
    let _guard = env_lock();
    env::remove_var("ROSRUSTEXT_BOND");

    let config = Config::from_args_iter(["bin", "--no-bond"]);
    assert!(!config.bond_enabled);
}

#[test]
fn bond_env_override_disables_bond() {
    let _guard = env_lock();
    env::set_var("ROSRUSTEXT_BOND", "0");

    let config = Config::from_args_iter(["bin"]);
    assert!(!config.bond_enabled);

    env::remove_var("ROSRUSTEXT_BOND");
}
