//! GPIO interrupt bookkeeping.
//!
//! The ISR callback only touches atomics (and, as a known debt, one `Mutex`); the main task
//! drains the per-pin counters when it processes a `GpioInput` device.

use lazy_static::lazy_static;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Mutex, MutexGuard};

/// Number of GPIOs with a per-pin counter. ESP32-C3 has 22 GPIOs; the fixed table is kept small
/// for compatibility with the original firmware. Pins outside this range only increment the
/// total counter (degraded, no panic). See `ARCHITECTURE.md` §19 item 20.
pub const MAX_TRACKED_PIN: usize = 10;

/// Total interrupts seen since boot (coalesced, used for logging only).
pub static INTERRUPT_COUNTER_GPIO: AtomicU32 = AtomicU32::new(0);

lazy_static! {
    pub static ref INTERRUPT_GPIO_HASHMAP: Mutex<[AtomicU32; MAX_TRACKED_PIN]> =
        Mutex::new(std::array::from_fn(|_| AtomicU32::new(0)));
}

/// Lock the counter table, recovering from poisoning instead of panicking in ISR context.
fn lock_counters() -> MutexGuard<'static, [AtomicU32; MAX_TRACKED_PIN]> {
    match INTERRUPT_GPIO_HASHMAP.lock() {
        Ok(guard) => guard,
        Err(poisoned) => poisoned.into_inner(),
    }
}

/// Record one interrupt for `pin`. Called from the GPIO ISR.
pub fn record_interrupt(pin: usize) {
    INTERRUPT_COUNTER_GPIO.fetch_add(1, Ordering::SeqCst);
    if let Some(counter) = lock_counters().get(pin) {
        counter.fetch_add(1, Ordering::SeqCst);
    }
}

/// Read and reset the coalesced counter for `pin`.
///
/// Returns `0` for pins outside the tracked range.
pub fn take_pin_count(pin: usize) -> u32 {
    let counters = lock_counters();
    match counters.get(pin) {
        Some(counter) => {
            let count = counter.load(Ordering::SeqCst);
            counter.store(0, Ordering::SeqCst);
            count
        }
        None => 0,
    }
}
