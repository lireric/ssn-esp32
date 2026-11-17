//! Shared runtime state owned by the main task and handed to the MQTT task.
//!
//! Everything here is a thin wrapper over `Arc<Mutex<..>>` so the composition root can create
//! one instance and clone cheap handles into callbacks and services.

use anyhow::Result;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

/// Last known value vector per device, keyed by `dev_id`.
/// Channels follow the per-variant order documented in `ARCHITECTURE.md` §7.3.
pub type SharedSensorData = Arc<Mutex<HashMap<String, Vec<f32>>>>;

pub fn create_shared_data() -> SharedSensorData {
    Arc::new(Mutex::new(HashMap::new()))
}

pub fn update_sensor_data(data: &SharedSensorData, dev_id: String, values: Vec<f32>) -> Result<()> {
    let mut guard = crate::platform::lock_recover(data);
    guard.insert(dev_id, values);
    Ok(())
}

pub fn get_sensor_values(data: &SharedSensorData, dev_id: &str) -> Option<Vec<f32>> {
    let guard = crate::platform::lock_recover(data);
    guard.get(dev_id).cloned()
}

/// A command aimed at a single device channel, produced by alarms or MQTT.
pub struct SsnDeviceCommand {
    pub dev_id: String,
    pub channel: u8,
    pub value: f32,
}

pub type SharedSsnDeviceCommand = Arc<Mutex<Vec<SsnDeviceCommand>>>;

pub fn create_commands_array() -> SharedSsnDeviceCommand {
    Arc::new(Mutex::new(Vec::new()))
}

/// A message that the main task must publish on the MQTT client (the callback must not block).
pub struct SsnMqttCommand {
    pub topic: String,
    pub message: String,
}

pub type SharedSsnMqttCommand = Arc<Mutex<Vec<SsnMqttCommand>>>;

pub fn create_mqtt_commands_array() -> SharedSsnMqttCommand {
    Arc::new(Mutex::new(Vec::new()))
}

pub fn push_command(data: &SharedSsnDeviceCommand, command: SsnDeviceCommand) -> Result<()> {
    let mut guard = crate::platform::lock_recover(data);
    log::info!(
        "push_command: dev={}, channel={}, val={}",
        command.dev_id,
        command.channel,
        command.value
    );
    guard.push(command);
    Ok(())
}

pub fn pop_command(data: &SharedSsnDeviceCommand) -> Option<SsnDeviceCommand> {
    let mut guard = crate::platform::lock_recover(data);
    guard.pop()
}

pub fn push_mqtt_command(data: &SharedSsnMqttCommand, command: SsnMqttCommand) -> Result<()> {
    let mut guard = crate::platform::lock_recover(data);
    log::info!(
        "push_command: command={}, message={}",
        command.topic,
        command.message
    );
    guard.push(command);
    Ok(())
}

pub fn pop_mqtt_command(data: &SharedSsnMqttCommand) -> Option<SsnMqttCommand> {
    let mut guard = crate::platform::lock_recover(data);
    guard.pop()
}
