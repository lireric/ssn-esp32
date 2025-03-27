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
    time::Duration,
};
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

enum SsnDevices<P: ADCPin<Adc = ADC1>> {
    Bmp180 {
        temperature: f32,
        pressure: i32,
        altitude: f32,
        sensor: BMP180Sensor<esp_idf_hal::i2c::I2cDriver<'static>, esp_idf_hal::delay::FreeRtos>,
    },
    Adc {
        raw_value: u16,
        voltage_mv: f32,
        sensor: AdcReader<P>,
    },
}

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

    ssn_devices.push(SsnDevices::Adc {
        raw_value: 0,
        voltage_mv: 0.0,
        sensor: (AdcReader::new(peripherals.pins.gpio3, adc1)
            .context("Failed to create ADC reader")?),
    });

    let gpio_adc = peripherals.pins.gpio4;
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
    ssn_devices.push(SsnDevices::Bmp180 {
        temperature: (0.0),
        pressure: (0),
        altitude: (0.0),
        sensor: (bmp180::BMP180Sensor::new(i2c, FreeRtos)?),
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

    loop {
        led.set_high().unwrap(); // Turn the LED on
        println!("LED ON");
        // Set all LEDs to red
        let pixel: [u8; 3] = red.as_ref().try_into().unwrap();
        ws2812.write_blocking(pixel.clone().into_iter()).unwrap();

        FreeRtos::delay_ms(1500); // Wait 500ms

        // let (temperature, pressure, altitude) = sensor.read_all(Resolution::UltraHighResolution)?;
        // let (temperature, pressure, altitude) = sensor.get_data().unwrap();

        // iterate through all devices, refresh their data and publish it:
        for device in &mut ssn_devices {
            match device {
                SsnDevices::Adc {
                    sensor,
                    raw_value,
                    voltage_mv,
                    ..
                } => {
                    (*raw_value, *voltage_mv) = sensor.read()?;
                    log::info!("Raw: {}, Voltage: {:.2}mV", *raw_value, *voltage_mv);
                }
                SsnDevices::Bmp180 {
                    sensor,
                    temperature,
                    pressure,
                    altitude,
                    ..
                } => {
                    (*temperature, *pressure, *altitude) = sensor.get_data().unwrap();
                    // 3. publish CPU temperature
                    client.enqueue(
                        &mqtt_messages::temperature_data_topic(mqtt_client_id.as_str()),
                        QoS::AtLeastOnce,
                        false,
                        temperature.to_string().as_bytes(),
                    )?;
                    client.enqueue(
                        &mqtt_messages::pressure_data_topic(mqtt_client_id.as_str()),
                        QoS::AtLeastOnce,
                        false,
                        pressure.to_string().as_bytes(),
                    )?;

                    log::info!("temperature = {}", temperature.to_string());
                    log::info!("pressure = {}", pressure.to_string());
                    log::info!("altitude = {}", altitude.to_string());
                }
            }
        }

        // let (raw, voltage) = adc_reader.read()?;
        // log::info!("Raw: {}, Voltage: {:.2}mV", raw, voltage);

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
