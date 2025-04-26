use embedded_svc::mqtt::client::Client;
use embedded_svc::mqtt::client::{
    Details::Complete, EventPayload::Error, EventPayload::Received, QoS,
};
use std::fmt::{Debug, Display};
use std::time::{Duration, Instant};

use crate::{update_sensor_data, SharedSensorData};

// use crate::mqtt_messages;

// Helper function to calculate percentage change
pub fn calculate_percentage_change(current: f32, previous: f32) -> f32 {
    if previous.abs() >= f32::EPSILON {
        ((current - previous) / previous) * 100.0
    } else {
        0.0
    }
}

// Function to handle MQTT sending logic for a sensor value
pub fn handle_sensor_value_update<T, U: Client>(
    client: &mut U,
    current_value: T,
    last_sent_value: &mut Option<T>,
    last_update: &mut Instant,
    change_pct: f32,
    delta_threshold: &f32,
    delta_over_threshold: Option<&f32>,
    now: Instant,
    max_refresh_period: Duration,
    topic: &str,
    value_name: &str,
) -> Result<(), anyhow::Error>
where
    T: std::fmt::Display + Copy + std::fmt::Debug + Into<f64>,
    U: embedded_svc::mqtt::client::Enqueue,
{
    let current_value_f32 = current_value.into() as f32;
    if change_pct >= *delta_threshold || last_update.elapsed() > max_refresh_period {
        let last_sent_value_tmp = last_sent_value.unwrap_or(current_value); // first time check

        if let Some(threshold) = delta_over_threshold {
            let threshold_pct =
                calculate_percentage_change(current_value_f32, last_sent_value_tmp.into() as f32);

            if threshold_pct.abs() >= *threshold {
                log::info!(
                    "Sensor - {} {:?} exceeds threshold {:.2}, skipping",
                    value_name,
                    current_value,
                    threshold
                );
                *last_sent_value = Some(current_value); // if such values is a new reality we'll send it for next iteration
                return Ok(());
            }
        }

        client
            .enqueue(
                topic,
                QoS::AtLeastOnce,
                false,
                current_value.to_string().as_bytes(),
            )
            .map_err(|e| anyhow::anyhow!("MQTT error: {:?}", e))?;
        *last_update = now;
        *last_sent_value = Some(current_value);
    } else {
        log::info!("Sensor {} skip sending  to mqtt", value_name);
    }
    Ok(())
}
