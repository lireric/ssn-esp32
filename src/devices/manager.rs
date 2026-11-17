//! Device sampling and publishing.
//!
//! This is the single device-processing path, moved out of `main.rs`. It preserves the original
//! inline behavior: one pass over the registered devices, each sampled when the scheduler fires,
//! with a 500 ms pause after every measured device. Read failures are logged and skipped instead
//! of panicking (`ARCHITECTURE.md` §17).

use esp_idf_hal::delay::FreeRtos;
use esp_idf_svc::mqtt::client::EspMqttClient;
use std::time::{Duration, Instant};

use crate::app::publish::{calculate_percentage_change, handle_sensor_value_update};
use crate::app::state::{update_sensor_data, SharedSensorData};
use crate::devices::model::{SharedSsnDevice, SsnDevices};
use crate::devices::scheduler::TimeManager;
use crate::platform::interrupts;
use crate::protocol::topics;

/// If a sensor value has not changed in this period, publish it anyway (seconds).
pub const MAX_PERIOD_REFRESH_MQTT: u64 = 60;

/// Everything the device pass needs from the main loop.
pub struct ProcessingContext<'a> {
    pub client: &'a mut EspMqttClient<'static>,
    pub account: &'a str,
    pub object: &'a str,
    pub now: Instant,
    pub command_dev_id: &'a str,
    pub command_channel: u8,
    pub command_value: f32,
    pub pin_num: Option<u32>,
}

pub struct DeviceManager<'a> {
    pub devices: Vec<SharedSsnDevice<'a>>,
    pub shared_data: SharedSensorData,
}

impl<'a> DeviceManager<'a> {
    pub fn new(devices: Vec<SharedSsnDevice<'a>>, shared_data: SharedSensorData) -> Self {
        Self {
            devices,
            shared_data,
        }
    }

    /// Sample every device whose period fires, and publish changed values.
    pub fn process_all_devices(&mut self, ctx: &mut ProcessingContext, time: &mut TimeManager) {
        let shared_data = self.shared_data.clone();

        let account = ctx.account;
        let object = ctx.object;
        let now = ctx.now;
        let command_dev_id = ctx.command_dev_id;
        let command_channel = ctx.command_channel;
        let command_value = ctx.command_value;
        let pin_num = ctx.pin_num;
        let client = &mut *ctx.client;

        for device in &self.devices {
            // Recover from a poisoned device lock instead of panicking in the sampling path.
            let mut cur_device = crate::platform::lock_recover(device);
            let dev = cur_device.dev_id.to_string();

            // check measure or not in this circle:
            if time.should_measure(&cur_device.period) {
                match &mut cur_device.device {
                    SsnDevices::Adc {
                        sensor,
                        raw_value,
                        voltage_mv,
                        raw_value_delta,
                        delta_over,
                        raw_value_last_update,
                        raw_value_last_sent_value,
                        voltage_value_last_sent_value,
                        ..
                    } => {
                        // Degraded mode: on a read error keep the last values and retry next cycle.
                        let (new_raw, new_voltage) = match sensor.read() {
                            Ok(values) => values,
                            Err(e) => {
                                log::error!("adc read failed on device {}: {:?}", dev, e);
                                continue;
                            }
                        };

                        // Calculate percentage changes
                        let raw_change_pct =
                            calculate_percentage_change(new_raw as f32, *raw_value as f32);
                        let voltage_change_pct =
                            calculate_percentage_change(new_voltage, *voltage_mv);

                        log::info!(
                            "adc: Raw={:.2}, Voltage={}, delta_Raw(%)={:.4}, delta_Voltage(%)={:.5}",
                            raw_value,
                            voltage_mv,
                            raw_change_pct,
                            voltage_change_pct
                        );

                        *raw_value = new_raw;
                        *voltage_mv = new_voltage;

                        // Handle voltage update
                        let _ = handle_sensor_value_update(
                            client,
                            *voltage_mv,
                            voltage_value_last_sent_value,
                            raw_value_last_update,
                            voltage_change_pct,
                            raw_value_delta,
                            delta_over.as_ref(),
                            now,
                            Duration::from_secs(MAX_PERIOD_REFRESH_MQTT),
                            &topics::publish_sensor_topic(account, object, &dev, "0"),
                            "adc-voltage",
                        );
                        // Handle raw update
                        let _ = handle_sensor_value_update(
                            client,
                            *raw_value,
                            raw_value_last_sent_value,
                            raw_value_last_update,
                            raw_change_pct,
                            raw_value_delta,
                            delta_over.as_ref(),
                            now,
                            Duration::from_secs(MAX_PERIOD_REFRESH_MQTT),
                            &topics::publish_sensor_topic(account, object, &dev, "1"),
                            "adc-raw",
                        );
                        let _ = update_sensor_data(
                            &shared_data,
                            dev,
                            vec![*voltage_mv, *raw_value as f32],
                        );
                    }
                    SsnDevices::Bmp180 {
                        sensor,
                        temperature,
                        pressure,
                        altitude,
                        temperature_delta,
                        temperature_delta_over,
                        temperature_last_update,
                        temperature_last_sent_value,
                        pressure_last_update,
                        pressure_last_sent_value,
                        pressure_delta,
                        pressure_delta_over,
                        ..
                    } => {
                        log::info!(
                            "Old: temperature={:.2}, pressure={}, altitude={}",
                            temperature,
                            pressure,
                            altitude
                        );
                        FreeRtos::delay_ms(500);
                        // Degraded mode: on a read error keep the last values and retry next cycle.
                        let (new_temp, new_pressure, new_alt) = match sensor.get_data() {
                            Ok(values) => values,
                            Err(e) => {
                                log::error!("bmp180 read failed on device {}: {:?}", dev, e);
                                continue;
                            }
                        };

                        // Calculate percentage changes
                        let temp_change_pct =
                            calculate_percentage_change(new_temp, *temperature);
                        let pressure_change_pct = calculate_percentage_change(
                            new_pressure as f32,
                            *pressure as f32,
                        );

                        log::info!(
                            "bmp180 New: temperature={:.2}, pressure={}, altitude={}, delta_temperature(%)={:.4}, delta_pressure(%)={:.5}",
                            new_temp,
                            new_pressure,
                            new_alt,
                            temp_change_pct,
                            pressure_change_pct
                        );

                        // Update stored values
                        *temperature = new_temp;
                        *pressure = new_pressure;
                        *altitude = new_alt;

                        // Handle temperature update
                        let _ = handle_sensor_value_update(
                            client,
                            *temperature,
                            temperature_last_sent_value,
                            temperature_last_update,
                            temp_change_pct,
                            temperature_delta,
                            temperature_delta_over.as_ref(),
                            now,
                            Duration::from_secs(MAX_PERIOD_REFRESH_MQTT),
                            &topics::publish_sensor_topic(account, object, &dev, "0"),
                            "bmp180-temperature",
                        );

                        // Handle pressure update
                        let _ = handle_sensor_value_update(
                            client,
                            *pressure,
                            pressure_last_sent_value,
                            pressure_last_update,
                            pressure_change_pct,
                            pressure_delta,
                            pressure_delta_over.as_ref(),
                            now,
                            Duration::from_secs(MAX_PERIOD_REFRESH_MQTT),
                            &topics::publish_sensor_topic(account, object, &dev, "1"),
                            "bmp180-pressure",
                        );
                        let _ = update_sensor_data(
                            &shared_data,
                            dev,
                            vec![*temperature, *pressure as f32],
                        );
                    }
                    SsnDevices::StoreInt {
                        data,
                        delta,
                        delta_over,
                        last_update,
                        last_sent_value,
                        ..
                    } => {
                        // check set value for StoreInt and store to Hash:
                        if dev.eq(command_dev_id) {
                            log::info!("set value StoreInt {}", command_value);
                            data.insert(command_channel.to_string(), command_value as i32);
                        };
                        // iterate for all values:
                        data.iter_mut().for_each(|d| {
                            let old_value = *d.1;
                            let new_value = old_value;

                            // Calculate percentage changes
                            let change_pct = calculate_percentage_change(
                                new_value as f32,
                                old_value as f32,
                            );

                            // Handle update
                            let mut cur_last_sent_value = match last_sent_value.get(d.0) {
                                Some(v) => v.map(|x| x as f32),
                                None => None,
                            };
                            let mut cur_last_update = match last_update.get(d.0) {
                                Some(v) => *v,
                                None => Instant::now(),
                            };

                            let _ = handle_sensor_value_update(
                                client,
                                old_value as f32,
                                &mut cur_last_sent_value,
                                &mut cur_last_update,
                                change_pct,
                                delta,
                                delta_over.as_ref(),
                                now,
                                Duration::from_secs(MAX_PERIOD_REFRESH_MQTT),
                                &topics::publish_sensor_topic(account, object, &dev, d.0),
                                "storeInt",
                            );

                            // Persist the per-key publish bookkeeping so the first-value and
                            // 60 s refresh rules behave like the other channels.
                            last_sent_value
                                .insert(d.0.clone(), cur_last_sent_value.map(|v| v as i32));
                            last_update.insert(d.0.clone(), cur_last_update);
                        });
                    }
                    SsnDevices::Ens160 {
                        sensor,
                        co2eq_ppm,
                        tvoc_ppb,
                        aqi,
                        co2eq_delta,
                        co2eq_delta_over,
                        tvoc_delta,
                        tvoc_delta_over,
                        aqi_delta,
                        aqi_delta_over,
                        co2eq_last_update,
                        tvoc_last_update,
                        aqi_last_update,
                        co2eq_last_sent_value,
                        tvoc_last_sent_value,
                        aqi_last_sent_value,
                        ..
                    } => {
                        FreeRtos::delay_ms(500);
                        // Degraded mode: ENS160 keeps publishing the invalid sentinel tuple when a
                        // read fails; the error is logged. (Documented in ARCHITECTURE.md §17.)
                        let (new_co2eq, new_tvoc, new_aqi) = match sensor.get_data() {
                            Ok(values) => values,
                            Err(e) => {
                                log::error!("ens160 read failed on device {}: {:?}", dev, e);
                                (-1, -1, -1)
                            }
                        };

                        // Calculate percentage changes
                        let co2eq_change_pct =
                            calculate_percentage_change(new_co2eq as f32, *co2eq_ppm as f32);
                        let tvoc_change_pct =
                            calculate_percentage_change(new_tvoc as f32, *tvoc_ppb as f32);
                        let aqi_change_pct =
                            calculate_percentage_change(new_aqi as f32, *aqi as f32);

                        log::info!(
                            "ENS160 - CO₂eq: {} ppm (Δ {:.2}%), TVOC: {} ppb (Δ {:.2}%), AQI: {} (Δ {:.2}%)",
                            new_co2eq,
                            co2eq_change_pct,
                            new_tvoc,
                            tvoc_change_pct,
                            new_aqi,
                            aqi_change_pct
                        );

                        // Update stored values
                        *co2eq_ppm = new_co2eq;
                        *tvoc_ppb = new_tvoc;
                        *aqi = new_aqi;

                        // Handle co2eq update
                        let _ = handle_sensor_value_update(
                            client,
                            *co2eq_ppm,
                            co2eq_last_sent_value,
                            co2eq_last_update,
                            co2eq_change_pct,
                            co2eq_delta,
                            co2eq_delta_over.as_ref(),
                            now,
                            Duration::from_secs(MAX_PERIOD_REFRESH_MQTT),
                            &topics::publish_sensor_topic(account, object, &dev, "0"),
                            "Ens160-co2eq",
                        );
                        // Handle tvoc update
                        let _ = handle_sensor_value_update(
                            client,
                            *tvoc_ppb,
                            tvoc_last_sent_value,
                            tvoc_last_update,
                            tvoc_change_pct,
                            tvoc_delta,
                            tvoc_delta_over.as_ref(),
                            now,
                            Duration::from_secs(MAX_PERIOD_REFRESH_MQTT),
                            &topics::publish_sensor_topic(account, object, &dev, "1"),
                            "Ens160-tvoc",
                        );
                        // Handle aqi update
                        let _ = handle_sensor_value_update(
                            client,
                            *aqi,
                            aqi_last_sent_value,
                            aqi_last_update,
                            aqi_change_pct,
                            aqi_delta,
                            aqi_delta_over.as_ref(),
                            now,
                            Duration::from_secs(MAX_PERIOD_REFRESH_MQTT),
                            &topics::publish_sensor_topic(account, object, &dev, "2"),
                            "Ens160-aqi",
                        );
                        let _ = update_sensor_data(
                            &shared_data,
                            dev,
                            vec![*co2eq_ppm as f32, *tvoc_ppb as f32, *aqi as f32],
                        );
                    }
                    SsnDevices::Aht21 {
                        temperature,
                        temperature_delta,
                        temperature_delta_over,
                        temperature_last_update,
                        temperature_last_sent_value,
                        humidity,
                        humidity_delta,
                        humidity_delta_over,
                        humidity_last_update,
                        humidity_last_sent_value,
                        sensor,
                    } => {
                        log::info!(
                            "aht21 Old: temperature={:.2}°C, humidity={:.2}%",
                            temperature,
                            humidity
                        );

                        // Read sensor data with error handling
                        // Degraded mode: on a read error keep the last values and retry next cycle.
                        let (new_temp, new_humidity) = match sensor.get_data() {
                            Ok(values) => values,
                            Err(e) => {
                                log::error!("aht21 read failed on device {}: {:?}", dev, e);
                                continue;
                            }
                        };

                        // Calculate percentage changes
                        let temperature_change_pct =
                            calculate_percentage_change(new_temp, *temperature);
                        let humidity_change_pct =
                            calculate_percentage_change(new_humidity, *humidity);

                        log::info!(
                            "aht21 New: temperature={:.2}°C (Δ {:.2}%), humidity={:.2}% (Δ {:.2}%)",
                            new_temp,
                            temperature_change_pct,
                            new_humidity,
                            humidity_change_pct
                        );

                        // Update values
                        *temperature = new_temp;
                        *humidity = new_humidity;

                        let _ = handle_sensor_value_update(
                            client,
                            *temperature,
                            temperature_last_sent_value,
                            temperature_last_update,
                            temperature_change_pct,
                            temperature_delta,
                            temperature_delta_over.as_ref(),
                            now,
                            Duration::from_secs(MAX_PERIOD_REFRESH_MQTT),
                            &topics::publish_sensor_topic(account, object, &dev, "0"),
                            "aht21-temperature",
                        );
                        let _ = handle_sensor_value_update(
                            client,
                            *humidity,
                            humidity_last_sent_value,
                            humidity_last_update,
                            humidity_change_pct,
                            humidity_delta,
                            humidity_delta_over.as_ref(),
                            now,
                            Duration::from_secs(MAX_PERIOD_REFRESH_MQTT),
                            &topics::publish_sensor_topic(account, object, &dev, "1"),
                            "aht21-humidity",
                        );
                        let _ = update_sensor_data(
                            &shared_data,
                            dev,
                            vec![*temperature, *humidity],
                        );
                    }
                    SsnDevices::GpioInput {
                        data,
                        delta,
                        delta_over,
                        last_update,
                        last_sent_value,
                        sensor,
                        counter,
                        last_sent_counter,
                        ..
                    } => {
                        if dev.eq(command_dev_id) {
                            log::info!("set value {}", command_value);
                            *data = command_value as u8;
                        };
                        let current_pin = match sensor.get_pin() {
                            Ok(pin) => pin as u32,
                            Err(e) => {
                                log::error!("gpio get_pin failed on device {}: {:?}", dev, e);
                                continue;
                            }
                        };
                        let old_counter = *counter;

                        if pin_num == Some(current_pin) {
                            *counter +=
                                interrupts::take_pin_count(current_pin as usize);
                            println!(
                                "Interrupt pin = {:?} current_pin: {}",
                                pin_num, current_pin
                            );
                            if let Err(e) = sensor.enable_interrupt() {
                                log::warn!("gpio re-enable interrupt failed on {}: {:?}", dev, e);
                            }
                        };
                        let new_data = match sensor.get_value() {
                            Ok(value) => value as u8,
                            Err(e) => {
                                log::error!("gpio read failed on device {}: {:?}", dev, e);
                                continue;
                            }
                        };
                        let data_change_pct =
                            calculate_percentage_change(new_data as f32, *data as u8 as f32);
                        let counter_change_pct = calculate_percentage_change(
                            *counter as f32,
                            old_counter as f32,
                        );

                        let _ = handle_sensor_value_update(
                            client,
                            new_data,
                            last_sent_value,
                            last_update,
                            data_change_pct,
                            delta,
                            delta_over.as_ref(),
                            now,
                            Duration::from_secs(MAX_PERIOD_REFRESH_MQTT),
                            &topics::publish_sensor_topic(account, object, &dev, "0"),
                            "gpio-in",
                        );
                        *data = new_data;
                        let mut last_sent_counter_f64 = last_sent_counter.map(|v| v as f64);
                        let _ = handle_sensor_value_update(
                            client,
                            *counter as f64,
                            &mut last_sent_counter_f64,
                            last_update,
                            counter_change_pct,
                            delta,
                            delta_over.as_ref(),
                            now,
                            Duration::from_secs(MAX_PERIOD_REFRESH_MQTT),
                            &topics::publish_sensor_topic(account, object, &dev, "1"),
                            "gpio-in-counter",
                        );
                        *last_sent_counter = last_sent_counter_f64.map(|v| v as u32);
                        let _ = update_sensor_data(
                            &shared_data,
                            dev,
                            vec![new_data as f32, *counter as f32],
                        );
                    }
                    SsnDevices::GpioOutput { data, sensor, .. } => {
                        if dev.eq(command_dev_id) {
                            log::info!("set value StoreInt {}", command_value);
                            let result = if command_value != 0.0 {
                                *data = true;
                                sensor.set_high()
                            } else {
                                *data = false;
                                sensor.set_low()
                            };
                            if let Err(e) = result {
                                log::error!("gpio write failed on device {}: {:?}", dev, e);
                            }
                        };
                    }
                    // KNOWN DEBT: the BMP280/BME280 processing path is not implemented. Degrade by
                    // logging and skipping the device instead of panicking (`ARCHITECTURE.md` §17).
                    SsnDevices::Bmp { .. } => {
                        log::error!(
                            "device {} uses the unimplemented Bmp variant; skipping",
                            dev
                        );
                    }
                }
                // little pause between measurments
                FreeRtos::delay_ms(500); // Wait 500ms
            }
        }
    }
}
