//! Fast/Middle/Slow sampling scheduler.
//!
//! The counters are only reset by `reset_if_needed()` at the end of a main-loop iteration, which
//! matches the original inline scheduler. (An earlier scaffold reset the counter when it fired,
//! shifting the cadence by one loop.)

use crate::devices::model::SsnSensorPeriod;
use std::time::{Duration, Instant};

pub struct TimeManager {
    time_counter_middle: Instant,
    time_counter_seldom: Instant,
    middle_period_ms: u64,
    slow_period_ms: u64,
}

impl TimeManager {
    pub fn new(middle_period_ms: u64, slow_period_ms: u64) -> Self {
        Self {
            time_counter_middle: Instant::now(),
            time_counter_seldom: Instant::now(),
            middle_period_ms,
            slow_period_ms,
        }
    }

    /// Should a device with `period` be sampled in this iteration?
    pub fn should_measure(&mut self, period: &SsnSensorPeriod) -> bool {
        match period {
            SsnSensorPeriod::Slow => {
                if self.time_counter_seldom.elapsed() > Duration::from_millis(self.slow_period_ms) {
                    log::info!("fire seldom {:?}", self.time_counter_seldom);
                    true
                } else {
                    false
                }
            }
            SsnSensorPeriod::Middle => {
                if self.time_counter_middle.elapsed() > Duration::from_millis(self.middle_period_ms)
                {
                    log::info!("fire middle {:?}", self.time_counter_middle);
                    true
                } else {
                    false
                }
            }
            SsnSensorPeriod::Fast => true,
        }
    }

    /// Reset expired counters at the end of a loop iteration.
    pub fn reset_if_needed(&mut self, now: Instant) {
        if now.duration_since(self.time_counter_seldom) > Duration::from_millis(self.slow_period_ms)
        {
            self.time_counter_seldom = Instant::now();
        }
        if now.duration_since(self.time_counter_middle)
            > Duration::from_millis(self.middle_period_ms)
        {
            self.time_counter_middle = Instant::now();
        }
    }
}
