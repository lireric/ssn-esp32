// #![cfg(feature = "smart-leds-trait")]
use anyhow::{Context, Result};
use core::str;
use embedded_svc::mqtt::client::{
    Details::Complete, EventPayload::Error, EventPayload::Received, QoS,
};
use embedded_svc::{http::Method, io::Write};
use esp_idf_hal::delay::FreeRtos; // For delay functionality
use esp_idf_hal::gpio::{ADCPin, Gpio0, Gpio1, Gpio2, Gpio3, Gpio4, Gpio5, Output, PinDriver}; // For GPIO control
use esp_idf_hal::prelude::*; // For peripheral access
use esp_idf_svc::hal::adc::oneshot::AdcDriver;
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    hal::{
        adc::ADC1,
        delay,
        i2c::{I2cConfig, I2cDriver},
        io::EspIOError,
        prelude::*,
    },
    http::server::{Configuration, EspHttpServer},
    mqtt::client::{Details, EspMqttClient, MqttClientConfiguration},
    nvs::{EspDefaultNvsPartition, EspNvs, EspNvsPartition, NvsDefault},
};
use esp_idf_sys::{self as _}; // Import the ESP-IDF bindings
use esp_idf_sys::{esp_err_t, esp_restart, ESP_ERR_TIMEOUT, ESP_OK};
use std::{
    sync::{Arc, Mutex},
    thread::sleep,
    time::{Duration, Instant},
};

// my modules:
mod commons;
use commons::ValuesDev;

mod mqtt_messages;
mod wifi;
use bincode::{config, Decode, Encode};
use esp_idf_hal::i2c::I2cError;
use serde::{Deserialize, Serialize};
mod settings;
use log::{info, warn};

use ws2812_esp32_rmt_driver::driver::color::{LedPixelColor, LedPixelColorGrb24};
use ws2812_esp32_rmt_driver::driver::Ws2812Esp32RmtDriver;

mod bmp180;
use bmp180::BMP180Sensor;

mod adc_reader;
use adc_reader::AdcReader;
use esp_idf_hal::gpio::AnyIOPin;

// if sensor value is not changed in this period mqtt message will by sended anyway
const MAX_PERIOD_REFRESH_MQTT: u64 = 20;

enum SsnSensorPeriod {
    Fast,
    Slow,
}

enum SsnDevices<P: ADCPin<Adc = ADC1>> {
    Bmp180 {
        temperature: f32,
        temperature_delta: f32, // delta between previous and new measurments for trigger sending mqtt message%
        temperature_last_update: Instant,
        temperature_last_sent_value: f32,
        pressure: i32,
        pressure_delta: f32,
        pressure_last_update: Instant,
        pressure_last_sent_value: i32,
        altitude: f32,
        period: SsnSensorPeriod,
        sensor: BMP180Sensor<esp_idf_hal::i2c::I2cDriver<'static>, esp_idf_hal::delay::FreeRtos>,
    },
    Adc {
        raw_value: u16,
        raw_value_delta: f32,
        raw_value_last_update: Instant,
        raw_value_last_sent_value: u16,
        voltage_mv: f32,
        period: SsnSensorPeriod,
        sensor: AdcReader<P>,
    },
    StoreInt {
        period: SsnSensorPeriod,
        data: i32,
        delta: f32,
        last_update: Instant,
        last_sent_value: i32,
    },
}

// #[derive(Default)]
struct SsnDevice<P: ADCPin<Adc = ADC1>> {
    dev_id: String,
    is_active: bool,
    is_paused: bool,
    device: SsnDevices<P>,
}

// impl <P: ADCPin<Adc = ADC1>> Default for SsnDevice <P> {
//     fn default() -> Self {
//         Self {
//             dev_id: "0".to_string(),
//             is_active: false,
//             is_paused: false,
//             device: SsnDevices::StoreInt { data: 0 }
//         }
//     }
// }

// fn main() -> Result<()> {
// Main logic that might return an error
fn main_logic() -> Result<()> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let mut ssn_devices = vec![];

    // Initialize NVS
    let nvs_default_partition: EspNvsPartition<NvsDefault> =
        EspDefaultNvsPartition::take().unwrap();
    let nvs_namespace = "ssn_ns";
    log::info!("Getting settings from NVS. nvs_namespace={}", nvs_namespace);

    let mut nvs = match EspNvs::new(nvs_default_partition, nvs_namespace, true) {
        Ok(nvs) => {
            log::info!("Got namespace {} from default partition", nvs_namespace);
            nvs
        }
        Err(e) => panic!("Could't get namespace {}", e),
    };

    let (app_config, app_config_size) = settings::get_config(nvs);
    let object = app_config.object.clone();
    let account = app_config.account.clone();

    log::info!(
        "Retrieved config: {}, size={}",
        app_config.clone(),
        app_config_size
    );

    let peripherals = esp_idf_hal::peripherals::Peripherals::take().unwrap();

    // Configure GPIO2 as an output pin for the LED
    // let mut led = PinDriver::output(peripherals.pins.gpio10).unwrap();
    let mut led = PinDriver::output(peripherals.pins.gpio6).unwrap();

    let adc1 = peripherals.adc1;

    ssn_devices.push(SsnDevice {
        dev_id: "adc-test".to_string(),
        is_active: true,
        is_paused: false,
        device: SsnDevices::Adc {
            raw_value: 0,
            voltage_mv: 0.0,
            raw_value_delta: 1.0, // 1%
            raw_value_last_sent_value: 0,
            raw_value_last_update: Instant::now() - Duration::from_secs(60 * 60 * 24 * 365 * 30),
            period: SsnSensorPeriod::Fast,
            sensor: (AdcReader::new(peripherals.pins.gpio4, adc1)
                .context("Failed to create ADC reader")?),
        },
    });

    // let gpio_adc = peripherals.pins.gpio4;
    // let mut adc_reader = AdcReader::new(gpio_adc, adc1).context("Failed to create ADC reader")?;
    // let mut adc_reader =
    // AdcReader::new(peripherals.pins.gpio3, adc1).context("Failed to create ADC reader")?;

    // let pins = peripherals.pins;
    let scl = peripherals.pins.gpio0;
    let sda = peripherals.pins.gpio1;

    // LED 2812
    let led_pin = peripherals.pins.gpio10;
    let channel = peripherals.rmt.channel0;

    // let mut ws2812 = LedPixelEsp32Rmt::<RGBW8, LedPixelColorGrbw32>::new(channel, led_pin).unwrap();
    // Create a WS2812 driver instance
    let mut ws2812 = Ws2812Esp32RmtDriver::new(channel, led_pin).unwrap(); // GPIO 10, RMT channel 0

    // Define some colors (RGB format)
    let red = LedPixelColorGrb24::new_with_rgb(30, 0, 0);
    let green = LedPixelColorGrb24::new_with_rgb(0, 30, 0);
    let blue = LedPixelColorGrb24::new_with_rgb(0, 0, 30);

    // // Initialize Wi-Fi
    let sys_loop = EspSystemEventLoop::take()?;

    // Connect to the Wi-Fi network
    let _wifi = wifi::wifi(
        app_config.wifi_ssid.as_str(),
        app_config.wifi_psk.as_str(),
        peripherals.modem,
        sys_loop,
    )?;

    log::info!("Our SSID is:");
    log::info!("{}", app_config.wifi_ssid);

    // let sda = peripherals.pins.gpio13;
    // let scl = peripherals.pins.gpio14;
    let i2c = peripherals.i2c0;
    let config = I2cConfig::new().baudrate(100.kHz().into());
    let i2c = I2cDriver::new(
        i2c,
        sda,
        scl,
        &esp_idf_svc::hal::i2c::config::Config::default(),
    )?;

    // let sensor = bmp180::init(i2c);
    // let mut sensor = bmp180::BMP180Sensor::new(i2c, FreeRtos)?; //.context("Failed to initialize BMP180 sensor")?;
    ssn_devices.push(SsnDevice {
        dev_id: "bmp180-test".to_string(),
        is_active: true,
        is_paused: false,
        device: SsnDevices::Bmp180 {
            temperature: (0.0),
            temperature_delta: 0.5, // 0.5%
            temperature_last_sent_value: 0.0,
            temperature_last_update: Instant::now() - Duration::from_secs(60 * 60 * 24 * 365 * 30),
            pressure: (0),
            pressure_delta: 0.5, // 0.5%
            pressure_last_sent_value: 0,
            pressure_last_update: Instant::now() - Duration::from_secs(60 * 60 * 24 * 365 * 30),
            altitude: (0.0),
            period: SsnSensorPeriod::Slow,
            sensor: (bmp180::BMP180Sensor::new(i2c, FreeRtos)?),
        },
    });

    ssn_devices.push(SsnDevice {
        dev_id: "store_int-test".to_string(),
        is_active: true,
        is_paused: false,
        device: SsnDevices::StoreInt {
            data: (10),
            period: SsnSensorPeriod::Fast,
            delta: 0.5,
            last_sent_value: 0,
            last_update: Instant::now() - Duration::from_secs(60 * 60 * 24 * 365 * 30),
        },
    });

    // MQTT Client configuration:
    log::info!("MQTT Client configuration");
    let broker_url = if !app_config.mqtt_user.is_empty() {
        format!(
            "mqtt://{}:{}@{}",
            app_config.mqtt_user, app_config.mqtt_pass, app_config.mqtt_host
        )
    } else {
        format!("mqtt://{}", app_config.mqtt_host)
    };

    let mqtt_config = MqttClientConfiguration::default();

    log::info!("Hello, world!");
    let mqtt_client_id = app_config.mqtt_client_id.clone();

    // Set the HTTP server
    let mut server = EspHttpServer::new(&Configuration::default())?;
    // http://<sta ip>/ handler
    server.fn_handler(
        "/",
        Method::Get,
        |request| -> core::result::Result<(), EspIOError> {
            let html = index_html();
            let mut response = request.into_ok_response()?;
            response.write_all(html.as_bytes())?;
            Ok(())
        },
    )?;
    // http://<sta ip>/settings handler
    server.fn_handler(
        "/settings",
        Method::Get,
        move |request| -> core::result::Result<(), EspIOError> {
            let html = format!("<h2>SSN ESP32 settings</h2> {}", app_config.clone());
            let mut response = request.into_ok_response()?;
            response.write_all(html.as_bytes())?;
            Ok(())
        },
    )?;

    log::info!("Http server awaiting connection");

    // 1. Create a client with default configuration and empty handler
    let mut client = EspMqttClient::new_cb(&broker_url, &mqtt_config, move |_message_event| {
        // ... your handler code here - leave this empty for now
        // we'll add functionality later in this chapter
    })?;

    // 2. publish an empty hello message
    let payload: &[u8] = &[];
    client.enqueue(
        &mqtt_messages::hello_topic(mqtt_client_id.as_str()),
        QoS::AtLeastOnce,
        true,
        payload,
    )?;

    // ********************* main loop

    loop {
        led.set_high().unwrap(); // Turn the LED on
        println!("LED ON");
        // Set all LEDs to red
        let pixel: [u8; 3] = red.as_ref().try_into().unwrap();
        ws2812.write_blocking(pixel.clone().into_iter()).unwrap();

        FreeRtos::delay_ms(1500); // Wait 500ms

        // let (temperature, pressure, altitude) = sensor.read_all(Resolution::UltraHighResolution)?;
        // let (temperature, pressure, altitude) = sensor.get_data().unwrap();

        // let mut values_array: Vec<ValuesDev> = vec![];

        // iterate through all devices, refresh their data and publish it:
        for cur_device in &mut ssn_devices {
            let dev = cur_device.dev_id.to_string();
            let now = Instant::now();

            match &mut cur_device.device {
                SsnDevices::Adc {
                    sensor,
                    raw_value,
                    voltage_mv,
                    raw_value_delta,
                    raw_value_last_update,
                    raw_value_last_sent_value,
                    ..
                } => {
                    let (new_raw, new_voltage) = sensor.read().unwrap();

                    // Calculate percentage change
                    let raw_value_change_pct = if *raw_value_last_sent_value != 0 {
                        ((new_raw as f32 - *raw_value_last_sent_value as f32).abs()
                            / *raw_value_last_sent_value as f32)
                            * 100.0
                    } else {
                        0.0
                    };

                    log::info!(
                        "Adc prev - Raw: {}, Voltage: {:.2}mV, delta(%)={}",
                        raw_value,
                        voltage_mv,
                        raw_value_change_pct
                    );

                    // Update values regardless of delta
                    *raw_value = new_raw;
                    *voltage_mv = new_voltage;
                    log::info!("Adc new - Raw: {}, Voltage: {:.2}mV", raw_value, voltage_mv);

                    // Check if we should send MQTT
                    if raw_value_change_pct >= *raw_value_delta
                        || raw_value_last_update.elapsed()
                            > Duration::from_secs(MAX_PERIOD_REFRESH_MQTT)
                    {
                        client.enqueue(
                            &mqtt_messages::publish_sensor_topic(&account, &object, &dev, "0"),
                            QoS::AtLeastOnce,
                            false,
                            voltage_mv.to_string().as_bytes(),
                        )?;
                        client.enqueue(
                            &mqtt_messages::publish_sensor_topic(&account, &object, &dev, "1"),
                            QoS::AtLeastOnce,
                            false,
                            raw_value.to_string().as_bytes(),
                        )?;
                        *raw_value_last_update = now;
                        *raw_value_last_sent_value = *raw_value;
                    } else {
                        log::info!("Adc - skip sending temperature to mqtt");
                    }
                }
                SsnDevices::Bmp180 {
                    sensor,
                    temperature,
                    pressure,
                    altitude,
                    temperature_delta,
                    temperature_last_update,
                    temperature_last_sent_value,
                    pressure_last_update,
                    pressure_last_sent_value,
                    pressure_delta,
                    ..
                } => {
                    log::info!(
                        "Old: temperature={:.2}, pressure={}, altitude={}",
                        temperature,
                        pressure,
                        altitude
                    );
                    // ... similar implementation for BMP180
                    let (new_temp, new_pressure, new_alt) =
                        sensor.get_data().context("bmp180 read data error").unwrap();

                    let temp_change_pct = if *temperature != 0.0 {
                        ((new_temp - *temperature).abs() / *temperature) * 100.0
                    } else {
                        0.0
                    };
                    let pressure_change_pct = if *pressure != 0 {
                        ((new_pressure - *pressure).abs() as f32 / *pressure as f32) * 100.0
                    } else {
                        0.0
                    };
                    log::info!(
                        "New: temperature={:.2}, pressure={}, altitude={}, delta_temperature(%)={:.4}, delta_pressure(%)={:.5}",
                        new_temp,
                        new_pressure,
                        new_alt,
                        temp_change_pct, pressure_change_pct
                    );

                    *temperature = new_temp;
                    *pressure = new_pressure;
                    *altitude = new_alt;

                    if temp_change_pct >= *temperature_delta
                        || temperature_last_update.elapsed()
                            > Duration::from_secs(MAX_PERIOD_REFRESH_MQTT)
                    {
                        client.enqueue(
                            &mqtt_messages::publish_sensor_topic(&account, &object, &dev, "0"),
                            QoS::AtLeastOnce,
                            false,
                            temperature.to_string().as_bytes(),
                        )?;
                        *temperature_last_update = now;
                        *temperature_last_sent_value = *temperature;
                    } else {
                        log::info!("Bmp180 - skip sending temperature to mqtt");
                    }

                    if pressure_change_pct >= *pressure_delta
                        || pressure_last_update.elapsed()
                            > Duration::from_secs(MAX_PERIOD_REFRESH_MQTT)
                    {
                        client.enqueue(
                            &mqtt_messages::publish_sensor_topic(&account, &object, &dev, "1"),
                            QoS::AtLeastOnce,
                            false,
                            pressure.to_string().as_bytes(),
                        )?;
                        *pressure_last_update = now;
                        *pressure_last_sent_value = *pressure;
                    } else {
                        log::info!("Bmp180 - skip sending pressure to mqtt");
                    }
                }

                SsnDevices::StoreInt {
                    period,
                    data,
                    delta,
                    last_update,
                    last_sent_value,
                    ..
                } => {
                    // ... similar implementation for StoreInt
                    let new_value = *data;
                    let value_change_pct = if *data != 0 {
                        ((new_value - *data).abs() as f32 / *data as f32) * 100.0
                    } else {
                        0.0
                    };

                    if value_change_pct >= *delta
                        || last_update.elapsed() > Duration::from_secs(MAX_PERIOD_REFRESH_MQTT)
                    {
                        client.enqueue(
                            &mqtt_messages::publish_sensor_topic(&account, &object, &dev, "0"),
                            QoS::AtLeastOnce,
                            false,
                            data.to_string().as_bytes(),
                        )?;
                        *last_update = now;
                        *last_sent_value = *data;
                    }
                }
            }
            // little pause between measurments
            FreeRtos::delay_ms(100); // Wait 100ms
        }

        led.set_low().unwrap(); // Turn the LED off
        println!("LED OFF");

        // Set all LEDs to blue
        let pixel: [u8; 3] = blue.as_ref().try_into().unwrap();
        ws2812.write_blocking(pixel.clone().into_iter()).unwrap();

        FreeRtos::delay_ms(1500); // Wait 500ms
    }
}

fn templated(content: impl AsRef<str>) -> String {
    format!(
        r#"
<!DOCTYPE html>
<html>
    <head>
        <meta charset="utf-8">
        <title>esp-rs web server</title>
    </head>
    <body>
        {}
    </body>
</html>
"#,
        content.as_ref()
    )
}

fn index_html() -> String {
    templated("Hello from ESP32-C3!")
}

fn temperature_html(val: f32) -> String {
    templated(format!("Chip temperature: {:.2}°C", val))
}

fn main() -> Result<()> {
    Ok(loop {
        match main_logic() {
            Ok(_) => {
                println!("Main logic completed successfully.");
                // Optionally, break or continue based on your logic
                // break; // Exit the loop if main_logic succeeds
            }
            Err(e) => {
                println!("Error occurred: {}", e);
                println!("Rebooting...");

                // Reboot the ESP32 only on error
                unsafe {
                    esp_restart();
                }
            }
        }
    })
    // Ok(())
}
