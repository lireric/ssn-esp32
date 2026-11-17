//! In-memory alarm store.
//!
//! Alarms match time-of-day only and emit a command when their active state changes. The store is
//! populated at boot with hardcoded test alarms; see `ARCHITECTURE.md` §13 and §19 item 7.

use chrono::{DateTime, FixedOffset, Utc};
use lazy_static::lazy_static;
use std::sync::Mutex;

#[derive(Clone)]
struct Alarm {
    trigger_time: DateTime<Utc>,
    active_duration_secs: u32,
    dev_id: String,
    active_level: u8,
    inactive_level: u8,
    previously_active: bool,
}

lazy_static! {
    static ref ALARMS: Mutex<Vec<Alarm>> = Mutex::new(Vec::new());
}

/// Add an alarm. The date is ignored when matching (time-of-day only).
pub fn add_alarm(
    trigger_time: DateTime<FixedOffset>,
    duration_secs: u32,
    dev_id: String,
    active_level: u8,
    inactive_level: u8,
) {
    let trigger_time_utc = trigger_time.with_timezone(&Utc);
    ALARMS.lock().unwrap_or_else(|poisoned| poisoned.into_inner()).push(Alarm {
        trigger_time: trigger_time_utc,
        active_duration_secs: duration_secs,
        dev_id,
        active_level,
        previously_active: false,
        inactive_level,
    });
    log::info!(
        "added alarm: {:?} [at UTC={:?}], duration = {}",
        trigger_time,
        trigger_time_utc,
        duration_secs
    );
}

/// Return `(dev_id, level)` only for alarms whose active state changed since the last call.
pub fn check_alarms() -> Vec<(String, u8)> {
    let now = Utc::now();
    let current_time = now.time();

    let mut alarms = ALARMS
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner());
    let mut actions: Vec<(String, u8)> = Vec::new();

    for alarm in alarms.iter_mut() {
        // TO DO: work with absolute dates or hourly/minutly etc variants
        let alarm_time = alarm.trigger_time.time();
        let end_time = alarm_time + chrono::Duration::seconds(alarm.active_duration_secs as i64);
        let is_active = if alarm_time <= end_time {
            current_time >= alarm_time && current_time <= end_time
        } else {
            // Handle midnight wrap-around
            current_time >= alarm_time || current_time <= end_time
        };

        // Only return actions when state changes
        if is_active != alarm.previously_active {
            let level = if is_active {
                alarm.active_level
            } else {
                alarm.inactive_level
            };
            actions.push((alarm.dev_id.clone(), level));
            alarm.previously_active = is_active;
        }
    }

    actions
}
