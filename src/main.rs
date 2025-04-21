// #![cfg(feature = "smart-leds-trait")]
use anyhow::{Context, Result};
use core::str;
use embedded_hal::i2c::ErrorType;
use embedded_hal::i2c::I2c;
use esp_idf_svc::mqtt::client::EspMqttEvent;
use esp_idf_svc::mqtt::client::EventPayload;
use log::error;
use utils::calculate_percentage_change;
use utils::handle_sensor_value_update;

use embedded_svc::mqtt::client::{
    Details::Complete, EventPayload::Error, EventPayload::Received, QoS,
};
use embedded_svc::{http::Method, io::Write};

use esp_idf_hal::delay::{Ets, FreeRtos}; // For delay functionality
use esp_idf_hal::gpio::{ADCPin, Gpio0, Gpio1, Gpio2, Gpio3, Gpio4, Gpio5, Output, PinDriver}; // For GPIO control
use esp_idf_hal::prelude::*; // For peripheral access
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    hal::{
        adc::ADC1,
        delay,
        i2c::{I2cConfig, I2cDriver, I2cError},
        io::EspIOError,
        prelude::*,
    },
    http::server::{Configuration, EspHttpServer},
    mqtt::client::{Details, EspMqttClient, Event, MqttClientConfiguration},
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
mod ssn_wifi;

use bincode::{config, Decode, Encode};
use serde::{Deserialize, Serialize};
mod settings;
use log::{info, warn};

use lazy_static::lazy_static;

use ws2812_esp32_rmt_driver::driver::color::{LedPixelColor, LedPixelColorGrb24};
use ws2812_esp32_rmt_driver::driver::Ws2812Esp32RmtDriver;

mod bmp180;
use bmp180::BMP180Sensor;

mod adc_reader;
use adc_reader::AdcReader;
use esp_idf_hal::gpio::AnyIOPin;

mod ens160;
use ens160::ENS160Sensor;

mod aht21;
use aht21::AHT21Sensor;

mod utils;

// if sensor value is not changed in this period mqtt message will by sended anyway
const MAX_PERIOD_REFRESH_MQTT: u64 = 20;
const MAX_RECONNECT_ATTEMPTS: u32 = 5;
const RECONNECT_DELAY_MS: u64 = 5000;

lazy_static! {
    // static ref I2C: Mutex<Option<I2cDriver<'static>>> = Mutex::new(None);
    static ref I2C: Mutex<Option<Arc<Mutex<I2cDriver<'static>>>>> = Mutex::new(None);
}

#[derive(Clone)]
struct SharedI2c(Arc<Mutex<I2cDriver<'static>>>);
impl SharedI2c {
    pub fn new(i2c: Arc<Mutex<I2cDriver<'static>>>) -> Self {
        SharedI2c(i2c)
    }
}
impl ErrorType for SharedI2c {
    type Error = I2cError; // Replace with the actual error type from `I2cDriver`
}
impl I2c for SharedI2c {
    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        let mut guard = self.0.lock().unwrap();
        Ok(guard.read(addr, buffer, 1000)?) // Add timeout (adjust value as needed)
    }

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        let mut guard = self.0.lock().unwrap();
        Ok(guard.write(addr, bytes, 1000)?) // Directly return I2cDriver's error
    }

    fn write_read(
        &mut self,
        address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> std::result::Result<(), Self::Error> {
        self.transaction(
            address,
            &mut [
                esp_idf_hal::i2c::Operation::Write(write),
                esp_idf_hal::i2c::Operation::Read(read),
            ],
        )
    }

    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [esp_idf_hal::i2c::Operation<'_>],
    ) -> std::result::Result<(), Self::Error> {
        let mut guard = self.0.lock().unwrap();
        Ok(guard.transaction(address, operations, 1000)?) // Timeout in milliseconds
    }
}

enum SsnSensorPeriod {
    Fast,
    Slow,
}

enum SsnDevices<P: ADCPin<Adc = ADC1>> {
    Bmp180 {
        temperature: f32,
        temperature_delta: f32, // delta between previous and new measurments for trigger sending mqtt message%
        temperature_delta_over: Option<f32>, // Skip if temperature exceeds this value (None = no limit)
        temperature_last_update: Instant,
        temperature_last_sent_value: Option<f32>,
        pressure: i32,
        pressure_delta: f32,
        pressure_delta_over: Option<f32>, // Skip if pressure exceeds this value (None = no limit)
        pressure_last_update: Instant,
        pressure_last_sent_value: Option<i32>,
        altitude: f32,
        period: SsnSensorPeriod,
        // sensor: BMP180Sensor<esp_idf_hal::i2c::I2cDriver<'static>, esp_idf_hal::delay::FreeRtos>,
        sensor: BMP180Sensor<SharedI2c, esp_idf_hal::delay::FreeRtos>,
    },
    Ens160 {
        co2eq_ppm: i16,
        co2eq_delta: f32,
        co2eq_delta_over: Option<f32>, // Skip if CO2 exceeds this threshold (ppm)
        co2eq_last_update: Instant,
        co2eq_last_sent_value: Option<i16>,
        tvoc_ppb: i16,
        tvoc_delta: f32,
        tvoc_delta_over: Option<f32>, // Skip if TVOC exceeds this threshold (ppb)
        tvoc_last_update: Instant,
        tvoc_last_sent_value: Option<i16>,
        aqi: i8,
        aqi_delta: f32,
        aqi_delta_over: Option<f32>, // Skip if AQI exceeds this threshold
        aqi_last_update: Instant,
        aqi_last_sent_value: Option<i8>,
        period: SsnSensorPeriod,
        // sensor: ENS160Sensor<esp_idf_hal::i2c::I2cDriver<'static>>,
        sensor: ENS160Sensor<SharedI2c>,
    },
    Adc {
        raw_value: u16,
        raw_value_delta: f32,
        delta_over: Option<f32>,
        raw_value_last_update: Instant,
        raw_value_last_sent_value: Option<u16>,
        voltage_value_last_sent_value: Option<f32>,
        voltage_mv: f32,
        period: SsnSensorPeriod,
        sensor: AdcReader<P>,
    },
    Aht21 {
        temperature: f32,
        temperature_delta: f32, // delta between previous and new measurments for trigger sending mqtt message%
        temperature_delta_over: Option<f32>, // Skip if temperature exceeds this value (°C)
        temperature_last_update: Instant,
        temperature_last_sent_value: Option<f32>,
        humidity: f32,
        humidity_delta: f32,
        humidity_delta_over: Option<f32>, // Skip if humidity exceeds this value (%)
        humidity_last_update: Instant,
        humidity_last_sent_value: Option<f32>,
        sensor: AHT21Sensor<SharedI2c>,
        period: SsnSensorPeriod,
    },
    StoreInt {
        period: SsnSensorPeriod,
        data: i32,
        delta: f32,
        delta_over: Option<f32>, // Skip if value exceeds this threshold (None = no limit)
        last_update: Instant,
        last_sent_value: Option<i32>,
    },
}

// #[derive(Default)]
struct SsnDevice<P: ADCPin<Adc = ADC1>> {
    dev_id: String,
    is_active: bool,
    is_paused: bool,
    device: SsnDevices<P>,
}

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
            raw_value_last_sent_value: Some(0),
            raw_value_last_update: Instant::now() - Duration::from_secs(60 * 60 * 24 * 365 * 30),
            period: SsnSensorPeriod::Fast,
            sensor: (AdcReader::new(peripherals.pins.gpio4, adc1)
                .context("Failed to create ADC reader")?),
            delta_over: Some(50.0),
            voltage_value_last_sent_value: Some(0.0),
        },
    });

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
    let sysloop = EspSystemEventLoop::take()?;
    let mut my_wifi = ssn_wifi::wifi(
        app_config.wifi_ssid.as_str(),
        app_config.wifi_psk.as_str(),
        peripherals.modem,
        sysloop.clone(),
    )
    .map_err(|e| {
        anyhow::anyhow!(
            "WiFi init failed for SSID {}: {:?}",
            app_config.wifi_ssid,
            e
        )
    })?;

    log::info!("Our SSID is:");
    log::info!("{}", app_config.wifi_ssid);

    // let sda = peripherals.pins.gpio13;
    // let scl = peripherals.pins.gpio14;
    let i2c = peripherals.i2c0;
    let config = I2cConfig::new().baudrate(400.kHz().into());
    let i2c_driver = I2cDriver::new(i2c, sda, scl, &config)?;
    // let hal_i2c = i2c_driver.reverse();

    fn init_i2c(i2c: I2cDriver<'static>) {
        // *I2C.lock().unwrap() = Some(i2c);
        *I2C.lock().unwrap() = Some(Arc::new(Mutex::new(i2c)));
        log::info!("I2C initialized");
    }

    let i2c_lock = Arc::new(Mutex::new(i2c_driver));

    // let shared_i2c = SharedI2c::new(i2c_lock); // Your wrapper implementing `I2c`
    let shared_i2c = SharedI2c::new(i2c_lock.clone()); // Your wrapper implementing `I2c`

    // Initialize AHT21 sensor
    let mut aht21 = AHT21Sensor::new(shared_i2c.clone()).unwrap();

    // Read sensor data
    let (temperature, humidity) = aht21.get_data().unwrap();
    log::info!("temperature (aht21): {:.2}C", temperature);
    log::info!("humidity (aht21): {:.2}%", humidity);

    ssn_devices.push(SsnDevice {
        dev_id: "aht21-test".to_string(),
        is_active: true,
        is_paused: false,
        device: SsnDevices::Aht21 {
            temperature: 0.0,
            temperature_delta: 0.5, // 0.5%
            temperature_last_sent_value: None,
            temperature_last_update: Instant::now() - Duration::from_secs(60 * 60 * 24 * 365 * 30),
            humidity: 0.0,
            humidity_delta: 0.5, // 0.5%
            humidity_last_sent_value: None,
            humidity_last_update: Instant::now() - Duration::from_secs(60 * 60 * 24 * 365 * 30),
            period: SsnSensorPeriod::Slow,
            sensor: AHT21Sensor::new(SharedI2c::new(i2c_lock.clone())).unwrap(),
            temperature_delta_over: Some(30.0),
            humidity_delta_over: Some(30.0),
        },
    });

    ssn_devices.push(SsnDevice {
        dev_id: "ens160-test".to_string(),
        is_active: true,
        is_paused: false,
        device: SsnDevices::Ens160 {
            co2eq_ppm: (-1),
            co2eq_delta: (5.0),
            co2eq_last_update: Instant::now() - Duration::from_secs(60 * 60 * 24 * 365 * 30),
            co2eq_last_sent_value: None,
            tvoc_ppb: (-1),
            tvoc_delta: (2.0),
            tvoc_last_update: (Instant::now() - Duration::from_secs(60 * 60 * 24 * 365 * 30)),
            tvoc_last_sent_value: None,
            aqi: (-1),
            aqi_delta: (1.0),
            aqi_last_update: (Instant::now() - Duration::from_secs(60 * 60 * 24 * 365 * 30)),
            aqi_last_sent_value: None,
            period: SsnSensorPeriod::Slow,
            sensor: ens160::ENS160Sensor::new(shared_i2c).unwrap(),
            co2eq_delta_over: Some(50.0),
            tvoc_delta_over: Some(50.0),
            aqi_delta_over: Some(50.0),
        },
    });

    ssn_devices.push(SsnDevice {
        dev_id: "bmp180-test".to_string(),
        is_active: true,
        is_paused: false,
        device: SsnDevices::Bmp180 {
            temperature: (0.0),
            temperature_delta: 0.5, // 0.5%
            temperature_last_sent_value: None,
            temperature_last_update: Instant::now() - Duration::from_secs(60 * 60 * 24 * 365 * 30),
            pressure: (0),
            pressure_delta: 0.5, // 0.5%
            pressure_last_sent_value: None,
            pressure_last_update: Instant::now() - Duration::from_secs(60 * 60 * 24 * 365 * 30),
            altitude: (0.0),
            period: SsnSensorPeriod::Slow,
            sensor: (bmp180::BMP180Sensor::new(SharedI2c::new(i2c_lock.clone()), FreeRtos)?),
            temperature_delta_over: Some(30.0),
            pressure_delta_over: Some(12.0),
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
            last_sent_value: None,
            last_update: Instant::now() - Duration::from_secs(60 * 60 * 24 * 365 * 30),
            delta_over: Some(30.0),
        },
    });

    // MQTT Client configuration *******************************************************

    log::info!("MQTT Client configuration");
    let mut reconnect_attempts = 0;

    let topics_to_subscribe = vec![
        format!("/ssn/acc/{}/raw_data", &account),
        format!("/ssn/acc/{}/obj/+/commands", &account),
        format!("/ssn/acc/{}/obj/+/commands/ini", &account),
        format!("/ssn/acc/{}/obj/+/commands/json", &account),
        format!("/ssn/acc/{}/obj/+/commands/device/+/+/in", &account),
    ];

    let broker_url = if !app_config.mqtt_user.is_empty() {
        format!(
            "mqtt://{}:{}@{}",
            app_config.mqtt_user, app_config.mqtt_pass, app_config.mqtt_host
        )
    } else {
        format!("mqtt://{}", app_config.mqtt_host)
    };

    let mqtt_client_id = app_config.mqtt_client_id.clone();
    // let mut mqtt_config = MqttClientConfiguration::default();
    let mqtt_config = MqttClientConfiguration {
        client_id: Some(mqtt_client_id.as_str()),
        ..Default::default()
    };

    log::info!("Hello, world!");

    // let mut client = EspMqttClient::new_cb(&broker_url, &mqtt_config, move |_message_event| {})?;
    // let (mut client, mut connection) = EspMqttClient::new(&broker_url, &mqtt_config)?;
    let mut client = EspMqttClient::new_cb(&broker_url, &mqtt_config, move |message_event| {
        match message_event.payload() {
            EventPayload::Received { data, topic, .. } => {
                log::info!("Received message on {:?}: {:?}", topic, data);
                mqtt_messages::process_message(topic, data);
            }
            EventPayload::Subscribed(id) => info!("Subscribed to id: {}", id),
            EventPayload::Disconnected => {
                warn!("MQTT disconnected, attempting to reconnect...");
                if reconnect_attempts < MAX_RECONNECT_ATTEMPTS {
                    std::thread::sleep(Duration::from_millis(RECONNECT_DELAY_MS));
                    reconnect_attempts += 1;
                    // You'll need to recreate the client here
                    // This might require restructuring to store broker_url and config
                } else {
                    error!("Max reconnection attempts reached");
                }
            }
            EventPayload::BeforeConnect => {
                info!("MQTT BeforeConnect");
                std::thread::sleep(Duration::from_millis(1000));
            }
            EventPayload::Connected(_) => {
                info!("MQTT reconnected successfully");
                reconnect_attempts = 0;
            }
            EventPayload::Error(e) => {
                warn!("MQTT error: {:?}", e);
                std::thread::sleep(Duration::from_millis(1000));
            }
            _ => info!("Received from MQTT: {:?}", message_event.payload()),
        }
    })?;

    std::thread::sleep(Duration::from_millis(3000));
    // Subscribe to topics
    for topic in topics_to_subscribe {
        info!("MQTT subscribe to {}", &topic);
        client.subscribe(&topic, QoS::AtLeastOnce)?;
    }

    // publish start message
    // let payload: &[u8] = &[];
    client.enqueue(
        &mqtt_messages::status_topic(&account, &object, "start"),
        QoS::AtLeastOnce,
        true,
        mqtt_client_id.as_bytes(),
    )?;

    // Set the HTTP server **************************************************************
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

    // ********************* main loop

    loop {
        // Check WiFi connection status
        if !ssn_wifi::check_wifi_connection(my_wifi.wifi()) {
            log::warn!("WiFi connection lost. Attempting to reconnect...");

            my_wifi.stop()?;
            my_wifi.start()?;

            if let Err(e) = ssn_wifi::connect_wifi(&mut my_wifi, &sysloop.clone()) {
                log::error!("Failed to reconnect WiFi: {:?}", e);
                // You might want to add a delay here before retrying
                std::thread::sleep(Duration::from_secs(10));
                continue;
            }
        }

        led.set_high().unwrap(); // Turn the LED on
        println!("LED ON");
        // Set all LEDs to red
        let pixel: [u8; 3] = green.as_ref().try_into().unwrap();
        ws2812.write_blocking(pixel.clone().into_iter()).unwrap();

        FreeRtos::delay_ms(2000);

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
                    delta_over,
                    raw_value_last_update,
                    raw_value_last_sent_value,
                    voltage_value_last_sent_value,
                    ..
                } => {
                    let (new_raw, new_voltage) = sensor.read().unwrap();

                    // Calculate percentage changes
                    let raw_change_pct =
                        calculate_percentage_change(new_raw as f32, *raw_value as f32);
                    let voltage_change_pct = calculate_percentage_change(new_voltage, *voltage_mv);

                    log::info!(
                        "New: Raw={:.2}, Voltage={}, delta_Raw(%)={:.4}, delta_Voltage(%)={:.5}",
                        raw_value,
                        voltage_mv,
                        raw_change_pct,
                        voltage_change_pct
                    );

                    *raw_value = new_raw;
                    *voltage_mv = new_voltage;

                    // Handle voltage update
                    handle_sensor_value_update(
                        &mut client,
                        *voltage_mv,
                        voltage_value_last_sent_value,
                        raw_value_last_update,
                        voltage_change_pct,
                        raw_value_delta,
                        delta_over.as_ref(),
                        now,
                        Duration::from_secs(MAX_PERIOD_REFRESH_MQTT),
                        &mqtt_messages::publish_sensor_topic(&account, &object, &dev, "0"),
                        "adc-voltage",
                    )?;
                    // Handle raw update
                    handle_sensor_value_update(
                        &mut client,
                        *raw_value,
                        raw_value_last_sent_value,
                        raw_value_last_update,
                        raw_change_pct,
                        raw_value_delta,
                        delta_over.as_ref(),
                        now,
                        Duration::from_secs(MAX_PERIOD_REFRESH_MQTT),
                        &mqtt_messages::publish_sensor_topic(&account, &object, &dev, "1"),
                        "adc-raw",
                    )?;
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
                    let (new_temp, new_pressure, new_alt) =
                        sensor.get_data().context("bmp180 read data error").unwrap();

                    // Calculate percentage changes
                    let temp_change_pct = calculate_percentage_change(new_temp, *temperature);
                    let pressure_change_pct =
                        calculate_percentage_change(new_pressure as f32, *pressure as f32);

                    log::info!(
                        "New: temperature={:.2}, pressure={}, altitude={}, delta_temperature(%)={:.4}, delta_pressure(%)={:.5}",
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
                    handle_sensor_value_update(
                        &mut client,
                        *temperature,
                        temperature_last_sent_value,
                        temperature_last_update,
                        temp_change_pct,
                        temperature_delta,
                        temperature_delta_over.as_ref(),
                        now,
                        Duration::from_secs(MAX_PERIOD_REFRESH_MQTT),
                        &mqtt_messages::publish_sensor_topic(&account, &object, &dev, "0"),
                        "bmp180-temperature",
                    )?;

                    // Handle pressure update
                    handle_sensor_value_update(
                        &mut client,
                        *pressure,
                        pressure_last_sent_value,
                        pressure_last_update,
                        pressure_change_pct,
                        pressure_delta,
                        pressure_delta_over.as_ref(),
                        now,
                        Duration::from_secs(MAX_PERIOD_REFRESH_MQTT),
                        &mqtt_messages::publish_sensor_topic(&account, &object, &dev, "1"),
                        "bmp180-pressure",
                    )?;
                }
                SsnDevices::StoreInt {
                    period,
                    data,
                    delta,
                    delta_over,
                    last_update,
                    last_sent_value,
                    ..
                } => {
                    // ... similar implementation for StoreInt
                    let new_value = *data;
                    // Calculate percentage changes
                    let change_pct = calculate_percentage_change(new_value as f32, *data as f32);

                    // Handle co2eq update
                    handle_sensor_value_update(
                        &mut client,
                        *data,
                        last_sent_value,
                        last_update,
                        change_pct,
                        delta,
                        delta_over.as_ref(),
                        now,
                        Duration::from_secs(MAX_PERIOD_REFRESH_MQTT),
                        &mqtt_messages::publish_sensor_topic(&account, &object, &dev, "0"),
                        "storeInt",
                    )?;

                    *data = new_value;
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
                    let (new_co2eq, new_tvoc, new_aqi) = sensor.get_data().unwrap_or((-1, -1, -1));

                    // Calculate percentage changes
                    let co2eq_change_pct =
                        calculate_percentage_change(new_co2eq as f32, *co2eq_ppm as f32);
                    let tvoc_change_pct =
                        calculate_percentage_change(new_tvoc as f32, *tvoc_ppb as f32);
                    let aqi_change_pct = calculate_percentage_change(new_aqi as f32, *aqi as f32);

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
                    handle_sensor_value_update(
                        &mut client,
                        *co2eq_ppm,
                        co2eq_last_sent_value,
                        co2eq_last_update,
                        co2eq_change_pct,
                        co2eq_delta,
                        co2eq_delta_over.as_ref(),
                        now,
                        Duration::from_secs(MAX_PERIOD_REFRESH_MQTT),
                        &mqtt_messages::publish_sensor_topic(&account, &object, &dev, "0"),
                        "Ens160-co2eq",
                    )?;
                    // Handle co2eq update
                    handle_sensor_value_update(
                        &mut client,
                        *tvoc_ppb,
                        tvoc_last_sent_value,
                        tvoc_last_update,
                        tvoc_change_pct,
                        tvoc_delta,
                        tvoc_delta_over.as_ref(),
                        now,
                        Duration::from_secs(MAX_PERIOD_REFRESH_MQTT),
                        &mqtt_messages::publish_sensor_topic(&account, &object, &dev, "1"),
                        "Ens160-tvoc",
                    )?; // Handle co2eq update
                    handle_sensor_value_update(
                        &mut client,
                        *aqi,
                        aqi_last_sent_value,
                        aqi_last_update,
                        aqi_change_pct,
                        aqi_delta,
                        aqi_delta_over.as_ref(),
                        now,
                        Duration::from_secs(MAX_PERIOD_REFRESH_MQTT),
                        &mqtt_messages::publish_sensor_topic(&account, &object, &dev, "2"),
                        "Ens160-aqi",
                    )?;
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
                    period,
                } => {
                    log::info!(
                        "Old: temperature={:.2}°C, humidity={:.2}%",
                        temperature,
                        humidity
                    );

                    // Read sensor data with error handling
                    let (new_temp, new_humidity) = match sensor.get_data() {
                        Ok(data) => data,
                        Err(e) => {
                            log::error!("AHT21 read failed: {:?}", e);
                            continue; // Skip this iteration if reading fails
                        }
                    };

                    // Calculate percentage changes
                    let temperature_change_pct =
                        calculate_percentage_change(new_temp as f32, *temperature as f32);
                    let humidity_change_pct =
                        calculate_percentage_change(new_humidity as f32, *humidity as f32);

                    log::info!(
                        "New: temperature={:.2}°C (Δ {:.2}%), humidity={:.2}% (Δ {:.2}%)",
                        new_temp,
                        temperature_change_pct,
                        new_humidity,
                        humidity_change_pct
                    );

                    // Update values
                    *temperature = new_temp;
                    *humidity = new_humidity;

                    handle_sensor_value_update(
                        &mut client,
                        *temperature,
                        temperature_last_sent_value,
                        temperature_last_update,
                        temperature_change_pct,
                        temperature_delta,
                        temperature_delta_over.as_ref(),
                        now,
                        Duration::from_secs(MAX_PERIOD_REFRESH_MQTT),
                        &mqtt_messages::publish_sensor_topic(&account, &object, &dev, "0"),
                        "aht21-temperature",
                    )?;
                    handle_sensor_value_update(
                        &mut client,
                        *humidity,
                        humidity_last_sent_value,
                        humidity_last_update,
                        humidity_change_pct,
                        humidity_delta,
                        humidity_delta_over.as_ref(),
                        now,
                        Duration::from_secs(MAX_PERIOD_REFRESH_MQTT),
                        &mqtt_messages::publish_sensor_topic(&account, &object, &dev, "1"),
                        "aht21-temperature",
                    )?;
                }
            }
            // little pause between measurments
            FreeRtos::delay_ms(500); // Wait 500ms
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
