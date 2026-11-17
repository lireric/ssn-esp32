//! Extracted MQTT-command / alarm / GPIO-notification / LED helpers.
//!
//! SCAFFOLD: these helpers are compiled but not called by the main loop yet. The main loop
//! currently uses `services::alarms`, `app::state`, `platform::interrupts`, and `services::led`
//! directly. Keep this module in sync if the loop is migrated to it (`ARCHITECTURE.md` §19
//! item 2).

use anyhow::Result;
use embedded_svc::mqtt::client::QoS;
use esp_idf_hal::task::notification::Notification;
use esp_idf_svc::mqtt::client::EspMqttClient;
use std::sync::Arc;
use ws2812_esp32_rmt_driver::driver::Ws2812Esp32RmtDriver;

use crate::app::state::{
    pop_command, pop_mqtt_command, push_command, SharedSsnDeviceCommand, SharedSsnMqttCommand,
    SsnDeviceCommand,
};
use crate::services::alarms::check_alarms;
use crate::services::led::Colors;

pub fn process_mqtt_commands(
    client: &mut EspMqttClient,
    mqtt_commands: &SharedSsnMqttCommand,
) -> Result<()> {
    while let Some(command) = pop_mqtt_command(mqtt_commands) {
        client.enqueue(
            &command.topic,
            QoS::AtLeastOnce,
            false,
            command.message.as_bytes(),
        )?;
    }
    Ok(())
}

pub fn process_alarms(commands: &SharedSsnDeviceCommand) -> Result<()> {
    for alarm in check_alarms() {
        push_command(
            commands,
            SsnDeviceCommand {
                dev_id: alarm.0,
                channel: 0,
                value: alarm.1 as f32,
            },
        )?;
    }
    Ok(())
}

pub fn get_gpio_interrupt(notification: &Arc<Notification>) -> Option<u32> {
    if let Some(value) = notification.wait(0) {
        Some(u32::from(value))
    } else {
        None
    }
}

pub fn get_device_command(commands: &SharedSsnDeviceCommand) -> (String, u8, f32) {
    if let Some(command) = pop_command(commands) {
        (command.dev_id, command.channel, command.value)
    } else {
        ("".to_string(), 0, 0.0)
    }
}

pub fn update_led_status(ws2812: &mut Ws2812Esp32RmtDriver, color: Colors) {
    crate::services::led::status_led(ws2812, color);
}
