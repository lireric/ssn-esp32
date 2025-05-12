use embedded_svc::mqtt::client::Client;
use embedded_svc::mqtt::client::{
    Details::Complete, EventPayload::Error, EventPayload::Received, QoS,
};
use esp_idf_hal::adc::ADC1;
use esp_idf_hal::gpio::ADCPin;
use std::fmt::{Debug, Display};
use std::time::{Duration, Instant};

use crate::{update_sensor_data, SharedSensorData, SsnChannelInfo, SsnDevice, SsnDevices};

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

// impl SsnDevice {
//     pub fn get_channels(&self) -> Vec<SsnChannelInfo> {
//         match &self.device {
//             SsnDevices::Bmp180 { .. } => vec![
//                 SsnChannelInfo {
//                     index: 0,
//                     name: "temperature",
//                     description: "Temperature in °C",
//                 },
//                 SsnChannelInfo {
//                     index: 1,
//                     name: "pressure",
//                     description: "Pressure in hPa",
//                 },
//                 SsnChannelInfo {
//                     index: 2,
//                     name: "altitude",
//                     description: "Altitude in meters",
//                 },
//             ],
//             SsnDevices::Ens160 { .. } => vec![
//                 SsnChannelInfo {
//                     index: 0,
//                     name: "co2",
//                     description: "CO2 equivalent in ppm",
//                 },
//                 SsnChannelInfo {
//                     index: 1,
//                     name: "tvoc",
//                     description: "TVOC in ppb",
//                 },
//                 SsnChannelInfo {
//                     index: 2,
//                     name: "aqi",
//                     description: "Air Quality Index",
//                 },
//             ],
//             SsnDevices::Adc { .. } => vec![
//                 SsnChannelInfo {
//                     index: 0,
//                     name: "voltage",
//                     description: "Voltage in mV",
//                 },
//                 SsnChannelInfo {
//                     index: 1,
//                     name: "raw",
//                     description: "Raw ADC value",
//                 },
//             ],
//             SsnDevices::Aht21 { .. } => vec![
//                 SsnChannelInfo {
//                     index: 0,
//                     name: "temperature",
//                     description: "Temperature in °C",
//                 },
//                 SsnChannelInfo {
//                     index: 1,
//                     name: "humidity",
//                     description: "Humidity in %",
//                 },
//             ],
//             SsnDevices::StoreInt { .. } => vec![SsnChannelInfo {
//                 index: 0,
//                 name: "value",
//                 description: "Stored integer value",
//             }],
//             SsnDevices::GpioInput {
//                 data,
//                 delta,
//                 interrupt_type,
//                 pull_type,
//                 delta_over,
//                 last_update,
//                 last_sent_value,
//                 sensor,
//             } => todo!(),
//             SsnDevices::GpioOutput {
//                 data,
//                 delta,
//                 delta_over,
//                 last_update,
//                 last_sent_value,
//                 sensor,
//             } => todo!(),
//         }
//     }
//     // Return real channel by it name or 0
//     pub fn get_channel_by_name(&self, name: &str) -> usize {
//         self.get_channels()
//             .into_iter()
//             .find(|c| c.name == name)
//             .map(|c| c.index)
//             .unwrap_or(0)
//     }

//     pub fn get_value_by_channel(&self, channel: usize) -> Option<f32> {
//         let channel = self
//             .get_channels()
//             .into_iter()
//             .find(|c| c.index == channel)?
//             .name;

//         match &self.device {
//             SsnDevices::Bmp180 {
//                 temperature,
//                 pressure,
//                 altitude,
//                 ..
//             } => match channel {
//                 "temperature" => Some(*temperature),
//                 "pressure" => Some(*pressure as f32),
//                 "altitude" => Some(*altitude),
//                 _ => None,
//             },
//             SsnDevices::Ens160 {
//                 co2eq_ppm,
//                 tvoc_ppb,
//                 aqi,
//                 ..
//             } => match channel {
//                 "co2" => Some(*co2eq_ppm as f32),
//                 "tvoc" => Some(*tvoc_ppb as f32),
//                 "aqi" => Some(*aqi as f32),
//                 _ => None,
//             },
//             SsnDevices::Adc {
//                 voltage_mv,
//                 raw_value,
//                 ..
//             } => match channel {
//                 "voltage" => Some(*voltage_mv),
//                 "raw" => Some(*raw_value as f32),
//                 _ => None,
//             },
//             SsnDevices::Aht21 {
//                 temperature,
//                 humidity,
//                 ..
//             } => match channel {
//                 "temperature" => Some(*temperature),
//                 "humidity" => Some(*humidity),
//                 _ => None,
//             },
//             SsnDevices::StoreInt { data, .. } => match channel {
//                 "value" => Some(0.0), // Some(data.get(channel)), TO DO work with hash...
//                 _ => None,
//             },
//             SsnDevices::GpioInput {
//                 data,
//                 delta,
//                 interrupt_type,
//                 pull_type,
//                 delta_over,
//                 last_update,
//                 last_sent_value,
//                 sensor,
//             } => todo!(),
//             SsnDevices::GpioOutput {
//                 data,
//                 delta,
//                 delta_over,
//                 last_update,
//                 last_sent_value,
//                 sensor,
//             } => todo!(),
//         }
//     }
// }
