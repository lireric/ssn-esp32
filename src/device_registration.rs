//! Hardcoded device registration.
//!
//! Registration is intentionally still hardcoded here (not driven by `Config.devices`); this is
//! the block that used to live inline in `main_logic()`. See `ARCHITECTURE.md` §19 items 4 and 16.

use anyhow::Result;
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::{InputPin, InterruptType, OutputPin, Pin, PinDriver, Pull};
use esp_idf_hal::task::notification::Notification;
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::bmp180::BMP180Sensor;
use crate::ens160::ENS160Sensor;
use crate::shared_state::SharedI2c;
use crate::ssndevices::{SharedSsnDevice, SsnDevice, SsnDevices, SsnSensorPeriod};

/// Sensors start with "never measured" timestamps so the first cycle always publishes.
fn initial_instant() -> Instant {
    Instant::now() - Duration::from_secs(60 * 60 * 24 * 365 * 30)
}

/// Register the GPIO outputs (3/4/5) and the GPIO input (6).
pub fn register_gpio_devices<'a, Tout3, Tout4, Tout5, Tin6>(
    devices: &mut Vec<SharedSsnDevice<'a>>,
    pin_out3: Tout3,
    pin_out4: Tout4,
    pin_out5: Tout5,
    pin_in6: Tin6,
    notification: Arc<Notification>,
) -> Result<()>
where
    Tout3: Pin + InputPin + OutputPin,
    Tout4: Pin + InputPin + OutputPin,
    Tout5: Pin + InputPin + OutputPin,
    Tin6: Pin + InputPin + OutputPin,
{
    register_gpio_output(devices, "gpio-out3", pin_out3)?;
    register_gpio_output(devices, "gpio-out4", pin_out4)?;
    register_gpio_output(devices, "gpio-out5", pin_out5)?;
    register_gpio_input(devices, "gpio-in6", pin_in6, notification)?;
    Ok(())
}

/// Register the I2C sensors that answered during initialization.
pub fn register_i2c_sensors<'a>(
    devices: &mut Vec<SharedSsnDevice<'a>>,
    shared_i2c: &SharedI2c,
) {
    match ENS160Sensor::new(shared_i2c.clone()) {
        Ok(sensor) => {
            SsnDevice::add_shared_device(
                devices,
                SsnDevice {
                    dev_id: "ens160-test".to_string(),
                    period: SsnSensorPeriod::Slow,
                    is_active: true,
                    is_paused: false,
                    device: SsnDevices::Ens160 {
                        co2eq_ppm: -1,
                        co2eq_delta: 5.0,
                        co2eq_last_update: initial_instant(),
                        co2eq_last_sent_value: None,
                        tvoc_ppb: -1,
                        tvoc_delta: 2.0,
                        tvoc_last_update: initial_instant(),
                        tvoc_last_sent_value: None,
                        aqi: -1,
                        aqi_delta: 1.0,
                        aqi_last_update: initial_instant(),
                        aqi_last_sent_value: None,
                        sensor,
                        co2eq_delta_over: Some(20.0),
                        tvoc_delta_over: Some(20.0),
                        aqi_delta_over: Some(30.0),
                    },
                },
            );
        }
        Err(e) => {
            log::error!("Failed to initialize ENS160 sensor: {:?}", e);
        }
    };

    match BMP180Sensor::new(shared_i2c.clone(), FreeRtos) {
        Ok(sensor) => {
            SsnDevice::add_shared_device(
                devices,
                SsnDevice {
                    dev_id: "bmp180".to_string(),
                    period: SsnSensorPeriod::Slow,
                    is_active: true,
                    is_paused: false,
                    device: SsnDevices::Bmp180 {
                        temperature: (0.0),
                        temperature_delta: 0.5, // 0.5%
                        temperature_last_sent_value: None,
                        temperature_last_update: initial_instant(),
                        pressure: (0),
                        pressure_delta: 0.5, // 0.5%
                        pressure_last_sent_value: None,
                        pressure_last_update: initial_instant(),
                        altitude: (0.0),
                        sensor,
                        temperature_delta_over: Some(10.0),
                        pressure_delta_over: Some(5.0),
                    },
                },
            );
        }
        Err(e) => {
            log::error!("Failed to initialize BMP180 sensor: {:?}", e);
        }
    };
}

fn register_gpio_output<'a, T>(
    devices: &mut Vec<SharedSsnDevice<'a>>,
    dev_id: &str,
    pin: T,
) -> Result<()>
where
    T: Pin + InputPin + OutputPin,
{
    SsnDevice::add_shared_device(
        devices,
        SsnDevice {
            dev_id: dev_id.to_string(),
            period: SsnSensorPeriod::Fast,
            is_active: true,
            is_paused: false,
            device: SsnDevices::GpioOutput {
                data: false,
                delta: 0.0,
                delta_over: None,
                last_update: initial_instant(),
                last_sent_value: None,
                sensor: Box::new(PinDriver::input_output(pin)?),
            },
        },
    );
    Ok(())
}

fn register_gpio_input<'a, T>(
    devices: &mut Vec<SharedSsnDevice<'a>>,
    dev_id: &str,
    pin: T,
    notification: Arc<Notification>,
) -> Result<()>
where
    T: Pin + InputPin + OutputPin,
{
    SsnDevice::add_shared_device(
        devices,
        SsnDevice {
            dev_id: dev_id.to_string(),
            period: SsnSensorPeriod::Fast,
            is_active: true,
            is_paused: false,
            device: SsnDevices::GpioInput {
                data: 0,
                delta: 1.0,
                delta_over: None,
                last_update: initial_instant(),
                last_sent_value: None,
                interrupt_type: InterruptType::AnyEdge,
                pull_type: Pull::Up,
                sensor: Box::new(PinDriver::input(pin)?),
                notificator: notification,
                counter: 0,
                last_sent_counter: None,
            },
        },
    );
    Ok(())
}

// ---------------------------------------------------------------------------------------------
// Disabled registrations, kept as a reference for re-enabling these devices.
//
// ADC (`adc-test`) — needs `peripherals.adc1` and a free ADC-capable pin (GPIO4 is also an
// output today):
//
//     SsnDevice::add_shared_device(&mut shared_devices, SsnDevice {
//         dev_id: "adc-test".to_string(),
//         period: SsnSensorPeriod::Fast,
//         device: SsnDevices::Adc {
//             raw_value: 0,
//             voltage_mv: 0.0,
//             raw_value_delta: 1.0,
//             raw_value_last_sent_value: Some(0),
//             raw_value_last_update: initial_instant(),
//             sensor: Box::new(AdcReader::new(peripherals.pins.gpio4, adc1)?),
//             delta_over: Some(50.0),
//             voltage_value_last_sent_value: Some(0.0),
//         },
//     });
//
// AHT21 (`aht21-test`, period Middle, `temperature_delta = 0.5`, `humidity_delta = 0.5`,
// `*_delta_over = Some(30.0)`) — instantiate with `AHT21Sensor::new(shared_i2c.clone())`.
//
// GPIO input on GPIO7 (`gpio-in7`) — same shape as `gpio-in6` with `peripherals.pins.gpio7`.
//
// StoreInt (`store_int-test`, period Fast, `delta = 1.0`) — starts with empty maps.
// ---------------------------------------------------------------------------------------------
