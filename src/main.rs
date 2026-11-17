//! `ssn-esp32` firmware composition root.
//!
//! This file wires the services together and owns the main loop. Everything else lives in its own
//! module; see `ARCHITECTURE.md` §5.

// my modules:
mod adc_reader;
mod aht21;
mod alarms;
mod bmp;
mod bmp180;
mod commons;
mod config_service;
mod device_manager;
mod device_registration;
mod ens160;
mod helpers;
mod http_server;
mod interrupts;
mod led;
mod mqtt_commands;
mod mqtt_config;
mod mqtt_messages;
mod mqtt_service;
mod settings;
mod shared_state;
mod ssn_wifi;
mod ssndevices;
mod time_manager;
mod utils;

use alarms::add_alarm;
use anyhow::Result;
use chrono::{FixedOffset, Local, TimeZone, Utc};
use commons::Colors;
use device_manager::{DeviceManager, ProcessingContext};
use embedded_svc::mqtt::client::QoS;
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::{PinDriver, Pull};
use esp_idf_hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_hal::prelude::*;
use esp_idf_hal::task::notification::Notification;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::sntp::EspSntp;
use esp_idf_sys::{self as _, esp_restart};
use std::sync::atomic::Ordering;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use time_manager::TimeManager;
use ws2812_esp32_rmt_driver::driver::Ws2812Esp32RmtDriver;

use crate::ssndevices::create_shared_devices;

// ----------------------------------
const MIDDLE_LOOP_PERIOD_MS: u64 = 20000;
const SLOW_LOOP_PERIOD_MS: u64 = 60000;
static DEF_TZ: i32 = 3; // default timezone (may by overrided from settings)

/// One full firmware lifetime: initialize every service, then run the main loop forever.
///
/// This function only returns on a fatal initialization error, which makes `main()` reboot.
fn main_logic() -> Result<()> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    // ----------------------------------------------------- shared runtime state:
    let shared_data = shared_state::create_shared_data();
    let device_commands = shared_state::create_commands_array();
    let publish_commands = shared_state::create_mqtt_commands_array();

    // Create a notification handle for the interrupt
    let notification = Arc::new(Notification::new());

    // ----------------------------------------------------- init periferals:
    let peripherals = esp_idf_hal::peripherals::Peripherals::take().unwrap();

    // settings reset button:
    // if button connect to ground then clean settings by default values
    let mut settings_reset_button = PinDriver::input(peripherals.pins.gpio8)?;
    settings_reset_button.set_pull(Pull::Up).unwrap();
    FreeRtos::delay_ms(100);

    // LED 2812:
    let led_pin = peripherals.pins.gpio10;
    let channel = peripherals.rmt.channel0;
    let mut ws2812 = Ws2812Esp32RmtDriver::new(channel, led_pin).unwrap(); // GPIO 10, RMT channel 0

    // ----------------------------------------------------- init settings:
    let mut settings = config_service::open_settings()?;

    let is_set_reset = settings_reset_button.is_low();
    log::info!("Reset button state: {}", is_set_reset);
    config_service::apply_reset(&mut settings, is_set_reset);

    let app_config = config_service::load_config(&mut settings)?;

    led::status_led(&mut ws2812, Colors::Yellow);

    // ----------------------------------------------------- init Wi-Fi + SNTP:
    let sysloop = EspSystemEventLoop::take()?;
    let mut my_wifi = ssn_wifi::wifi(
        app_config.wifi_ssid.as_str(),
        app_config.wifi_psk.as_str(),
        peripherals.modem,
        sysloop.clone(),
        &app_config.ssn_wifi_ssid,
        &app_config.ssn_wifi_pass,
    )
    .map_err(|e| {
        led::status_led(&mut ws2812, Colors::Red);
        anyhow::anyhow!(
            "WiFi init failed for SSID {}: {:?}",
            app_config.wifi_ssid,
            e
        )
    })?;

    let _sntp = EspSntp::new_default()?; // Syncs time automatically
    // TO DO: get TZ from settings
    let offset = FixedOffset::east_opt(DEF_TZ * 3600).unwrap_or(FixedOffset::east(0)); // Seconds
    let local_time = Local::now().with_timezone(&offset);
    log::info!("Sinc time.. {:?}", local_time);

    // green LED after successful WiFi:
    led::status_led(&mut ws2812, Colors::Green);

    log::info!("Our SSID is:");
    log::info!("{}", app_config.wifi_ssid);

    // ----------------------------------------------------- init I2C:
    let scl = peripherals.pins.gpio0;
    let sda = peripherals.pins.gpio1;
    let i2c = peripherals.i2c0;
    let config = I2cConfig::new().baudrate(400.kHz().into());
    let i2c_driver = I2cDriver::new(i2c, sda, scl, &config)?;
    let shared_i2c = shared_state::SharedI2c::new(Arc::new(Mutex::new(i2c_driver)));

    // ----------------------------------------------------- register devices:
    let mut devices = create_shared_devices(vec![]);
    device_registration::register_gpio_devices(
        &mut devices,
        peripherals.pins.gpio3,
        peripherals.pins.gpio4,
        peripherals.pins.gpio5,
        peripherals.pins.gpio6,
        notification.clone(),
    )?;
    device_registration::register_i2c_sensors(&mut devices, &shared_i2c);

    let mut device_manager = DeviceManager::new(devices, shared_data.clone());

    // ----------------------------------------------------- alarms:
    add_alarm(
        FixedOffset::east_opt(DEF_TZ * 3600)
            .unwrap()
            .with_ymd_and_hms(2025, 8, 2, 14, 45, 0)
            .unwrap(),
        10,
        "gpio-out4".to_string(),
        1,
        0,
    );
    add_alarm(
        FixedOffset::east_opt(DEF_TZ * 3600)
            .unwrap()
            .with_ymd_and_hms(2025, 8, 2, 14, 46, 0)
            .unwrap(),
        10,
        "gpio-out5".to_string(),
        1,
        0,
    );

    // ----------------------------------------------------- MQTT:
    let account = app_config.account.clone();
    let object = app_config.object.clone();

    let mut client = mqtt_service::start(
        &app_config,
        settings,
        shared_data.clone(),
        device_commands.clone(),
        publish_commands.clone(),
    )?;

    // ----------------------------------------------------- HTTP server:
    let _http_server = http_server::start(app_config)?;

    // preinit time counters:
    let mut time_manager = TimeManager::new(MIDDLE_LOOP_PERIOD_MS, SLOW_LOOP_PERIOD_MS);

    // ********************* main loop ****************************
    loop {
        // Check WiFi connection status
        if !ssn_wifi::check_wifi_connection(my_wifi.wifi()) {
            log::warn!("WiFi connection lost. Attempting to reconnect...");
            led::status_led(&mut ws2812, Colors::Red);

            my_wifi.stop()?;
            my_wifi.start()?;

            if let Err(e) = ssn_wifi::connect_wifi(&mut my_wifi, &sysloop) {
                log::error!("Failed to reconnect WiFi: {:?}", e);
                std::thread::sleep(Duration::from_secs(10));
                continue;
            }
        }

        let cur_time = Utc::now();
        let now = Instant::now();
        log::info!("New measurment circle t={:?}..", cur_time);

        // check messages to mqtt broker:
        if let Some(command) = shared_state::pop_mqtt_command(&publish_commands) {
            log::info!(
                "pop_mqtt_command: topic={}, message={}",
                command.topic,
                command.message
            );
            client
                .enqueue(
                    &command.topic,
                    QoS::AtLeastOnce,
                    false,
                    command.message.as_bytes(),
                )
                .map_err(|e| anyhow::anyhow!("MQTT error: {:?}", e))?;
        }

        // check for alarms and make command for device action:
        // check_alarms returm actual dev_id and value only at the moment of timer triggered or canceled
        for alarm in alarms::check_alarms() {
            log::info!("Alarm triggered dev_id={}, value={}", alarm.0, alarm.1);
            let _ = shared_state::push_command(
                &device_commands,
                shared_state::SsnDeviceCommand {
                    dev_id: alarm.0.clone(),
                    channel: 0,
                    value: alarm.1 as f32,
                },
            );
        }

        led::status_led(&mut ws2812, Colors::Blue);
        FreeRtos::delay_ms(100);

        let (command_dev_id, command_channel, command_value) =
            if let Some(command) = shared_state::pop_command(&device_commands) {
                log::info!(
                    "pop_command: dev={}, channel={}, val={}",
                    command.dev_id,
                    command.channel,
                    command.value
                );
                (command.dev_id, command.channel, command.value)
            } else {
                ("".to_string(), 0, 0.0)
            };

        let pin_num = if let Some(value) = notification.wait(0) {
            // Get interrupted pin number:
            let count = interrupts::INTERRUPT_COUNTER_GPIO.load(Ordering::SeqCst);
            println!(
                "Interrupt triggered! pin = {} Total count: {}",
                value, count
            );
            Some(u32::from(value))
        } else {
            None
        };

        // iterate through all devices, refresh their data and publish it:
        {
            let mut ctx = ProcessingContext {
                client: &mut client,
                account: &account,
                object: &object,
                now,
                command_dev_id: &command_dev_id,
                command_channel,
                command_value,
                pin_num,
            };
            device_manager.process_all_devices(&mut ctx, &mut time_manager);
        }

        println!("Measurment circle finished");

        // Set LED to green again
        led::status_led(&mut ws2812, Colors::Green);

        // reset time counters if needed:
        time_manager.reset_if_needed(now);

        FreeRtos::delay_ms(500); // Wait 500ms
    }
}

fn main() -> Result<()> {
    loop {
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
    }
}
