//! Platform primitives shared by the app: I2C bus and GPIO interrupts.

pub mod i2c;
pub mod interrupts;

use std::sync::{Mutex, MutexGuard};

/// Lock a mutex, recovering from poisoning instead of panicking.
///
/// This is the documented degraded mode for shared state: if a task poisoned a lock, the main
/// loop keeps running with the recovered contents rather than rebooting. See
/// `ARCHITECTURE.md` §17.
pub fn lock_recover<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    match m.lock() {
        Ok(guard) => guard,
        Err(poisoned) => {
            log::error!("mutex poisoned; continuing with recovered state");
            poisoned.into_inner()
        }
    }
}

