//! Shared runtime state owned by the main task and handed to the MQTT task.
//!
//! Everything here is a thin wrapper over `Arc<Mutex<..>>` so the composition root can create
//! one instance and clone cheap handles into callbacks and services.

use anyhow::Result;
use embedded_hal::i2c::{ErrorType, I2c};
use esp_idf_hal::i2c::{I2cDriver, I2cError, Operation};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

/// Last known value vector per device, keyed by `dev_id`.
/// Channels follow the per-variant order documented in `ARCHITECTURE.md` §7.3.
pub type SharedSensorData = Arc<Mutex<HashMap<String, Vec<f32>>>>;

pub fn create_shared_data() -> SharedSensorData {
    Arc::new(Mutex::new(HashMap::new()))
}

pub fn update_sensor_data(data: &SharedSensorData, dev_id: String, values: Vec<f32>) -> Result<()> {
    let mut guard = data.lock().unwrap();
    guard.insert(dev_id, values);
    Ok(())
}

pub fn get_sensor_values(data: &SharedSensorData, dev_id: &str) -> Option<Vec<f32>> {
    let guard = data.lock().unwrap();
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
    let mut guard = data.lock().unwrap();
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
    let mut guard = data.lock().unwrap();
    guard.pop()
}

pub fn push_mqtt_command(data: &SharedSsnMqttCommand, command: SsnMqttCommand) -> Result<()> {
    let mut guard = data.lock().unwrap();
    log::info!(
        "push_command: command={}, message={}",
        command.topic,
        command.message
    );
    guard.push(command);
    Ok(())
}

pub fn pop_mqtt_command(data: &SharedSsnMqttCommand) -> Option<SsnMqttCommand> {
    let mut guard = data.lock().unwrap();
    guard.pop()
}

/// `I2c` handle shared by every sensor adapter.
#[derive(Clone)]
pub struct SharedI2c(Arc<Mutex<I2cDriver<'static>>>);

impl SharedI2c {
    pub fn new(i2c: Arc<Mutex<I2cDriver<'static>>>) -> Self {
        SharedI2c(i2c)
    }
}

impl ErrorType for SharedI2c {
    type Error = I2cError;
}

impl I2c for SharedI2c {
    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        let mut guard = self.0.lock().unwrap();
        Ok(guard.read(addr, buffer, 1000)?)
    }

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        let mut guard = self.0.lock().unwrap();
        Ok(guard.write(addr, bytes, 1000)?)
    }

    fn write_read(
        &mut self,
        address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> std::result::Result<(), Self::Error> {
        self.transaction(
            address,
            &mut [Operation::Write(write), Operation::Read(read)],
        )
    }

    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> std::result::Result<(), Self::Error> {
        let mut guard = self.0.lock().unwrap();
        Ok(guard.transaction(address, operations, 1000)?)
    }
}
