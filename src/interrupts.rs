//! GPIO interrupt bookkeeping.
//!
//! The ISR callback only touches atomics (and, as a known debt, one `Mutex`); the main task
//! drains the per-pin counters when it processes a `GpioInput` device.

use lazy_static::lazy_static;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Mutex;

/// Total interrupts seen since boot (coalesced, used for logging only).
pub static INTERRUPT_COUNTER_GPIO: AtomicU32 = AtomicU32::new(0);

// Per-pin interrupt counters, indexed by GPIO number.
//
// NOTE: this is a fixed 10-entry array. Interrupts on pins >= 10 index out of bounds; see
// `ARCHITECTURE.md` §19 item 20.
lazy_static! {
    pub static ref INTERRUPT_GPIO_HASHMAP: Mutex<[AtomicU32; 10]> = Mutex::new([
        AtomicU32::new(0),
        AtomicU32::new(0),
        AtomicU32::new(0),
        AtomicU32::new(0),
        AtomicU32::new(0),
        AtomicU32::new(0),
        AtomicU32::new(0),
        AtomicU32::new(0),
        AtomicU32::new(0),
        AtomicU32::new(0),
    ]);
}

/// Record one interrupt for `pin`. Called from the GPIO ISR.
pub fn record_interrupt(pin: usize) {
    INTERRUPT_COUNTER_GPIO.fetch_add(1, Ordering::SeqCst);
    INTERRUPT_GPIO_HASHMAP.lock().unwrap()[pin].fetch_add(1, Ordering::SeqCst);
}

/// Read and reset the coalesced counter for `pin`.
pub fn take_pin_count(pin: usize) -> u32 {
    let counters = INTERRUPT_GPIO_HASHMAP.lock().unwrap();
    let count = counters[pin].load(Ordering::SeqCst);
    counters[pin].store(0, Ordering::SeqCst);
    count
}
