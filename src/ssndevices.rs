use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::{
    InputOutput, InputPin, InterruptType, Level, OutputPin, Pin, PinDriver, Pull,
};
use esp_idf_hal::task::notification::Notification;
use esp_idf_sys::{EspError, ESP_OK};
use lazy_static::lazy_static;
use log;
use serde::{Deserialize, Serialize};
use std::any::Any;
use std::collections::HashMap;
use std::num::NonZeroU32;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Instant;

use crate::adc_reader::AdcReaderTrait;
use crate::aht21::AHT21Sensor;
use crate::bmp;
use crate::bmp180::BMP180Sensor;
use crate::ens160::ENS160Sensor;
use crate::SharedI2c;

lazy_static! {
    static ref INTERRUPT_COUNTER_GPIO: AtomicU32 = AtomicU32::new(0);
    static ref INTERRUPT_GPIO_HASHMAP: Mutex<Vec<AtomicU32>> = Mutex::new(vec![]);
}

pub enum SsnSensorPeriod {
    Fast,
    Middle,
    Slow,
}

pub enum SsnDevices<'a> {
    Bmp {
        sensor_type: bmp::SensorType,
        temperature: f64,
        temperature_delta: f64,
        temperature_delta_over: Option<f64>,
        temperature_last_update: Instant,
        temperature_last_sent_value: Option<f64>,
        pressure: i32,
        pressure_delta: f64,
        pressure_delta_over: Option<f64>,
        pressure_last_update: Instant,
        pressure_last_sent_value: Option<f64>,
        humidity: Option<f64>,
        humidity_delta: f64,
        humidity_delta_over: Option<f64>,
        humidity_last_update: Instant,
        humidity_last_sent_value: Option<f64>,
        sensor: bmp::BMPSensor<SharedI2c, FreeRtos>,
    },
    Bmp180 {
        temperature: f32,
        temperature_delta: f32,
        temperature_delta_over: Option<f32>,
        temperature_last_update: Instant,
        temperature_last_sent_value: Option<f32>,
        pressure: i32,
        pressure_delta: f32,
        pressure_delta_over: Option<f32>,
        pressure_last_update: Instant,
        pressure_last_sent_value: Option<i32>,
        altitude: f32,
        sensor: BMP180Sensor<SharedI2c, FreeRtos>,
    },
    Ens160 {
        co2eq_ppm: i16,
        co2eq_delta: f32,
        co2eq_delta_over: Option<f32>,
        co2eq_last_update: Instant,
        co2eq_last_sent_value: Option<i16>,
        tvoc_ppb: i16,
        tvoc_delta: f32,
        tvoc_delta_over: Option<f32>,
        tvoc_last_update: Instant,
        tvoc_last_sent_value: Option<i16>,
        aqi: i8,
        aqi_delta: f32,
        aqi_delta_over: Option<f32>,
        aqi_last_update: Instant,
        aqi_last_sent_value: Option<i8>,
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
        sensor: Box<dyn AdcReaderTrait + 'a>,
    },
    Aht21 {
        temperature: f32,
        temperature_delta: f32,
        temperature_delta_over: Option<f32>,
        temperature_last_update: Instant,
        temperature_last_sent_value: Option<f32>,
        humidity: f32,
        humidity_delta: f32,
        humidity_delta_over: Option<f32>,
        humidity_last_update: Instant,
        humidity_last_sent_value: Option<f32>,
        sensor: AHT21Sensor<SharedI2c>,
    },
    StoreInt {
        data: HashMap<String, i32>,
        delta: f32,
        delta_over: Option<f32>,
        last_update: HashMap<String, Instant>,
        last_sent_value: HashMap<String, Option<i32>>,
    },
    GpioInput {
        data: u8,
        delta: f32,
        interrupt_type: InterruptType,
        notificator: Arc<Notification>,
        counter: u32,
        pull_type: Pull,
        delta_over: Option<f32>,
        last_update: Instant,
        last_sent_value: Option<u8>,
        last_sent_counter: Option<u32>,
        sensor: Box<dyn PinDriverBasicInput + 'static>,
    },
    GpioOutput {
        data: bool,
        delta: f32,
        delta_over: Option<f32>,
        last_update: Instant,
        last_sent_value: Option<bool>,
        sensor: Box<dyn PinDriverTraitOutput + 'a>,
    },
}

pub trait PinDriverBasicInput: Any + Send {
    fn set_pull(&mut self, pull: Pull) -> Result<(), EspError>;
    fn get_value(&mut self) -> Result<bool, EspError>;
    fn set_interrupt_type(&mut self, interript_type: InterruptType) -> Result<(), EspError>;
    fn subscribe_boxed(&mut self, nf: Arc<Notification>) -> Result<(), EspError>;
    fn enable_interrupt(&mut self) -> Result<(), EspError>;
    fn get_pin(&mut self) -> Result<i32, EspError>;
}

impl dyn PinDriverBasicInput {
    pub fn as_any(&mut self) -> &mut dyn Any {
        self
    }
}

pub trait PinDriverTraitOutput {
    fn set_pull(&mut self, pull: Pull) -> Result<(), EspError>;
    fn get_value(&mut self) -> Result<bool, EspError>;
    fn set_high(&mut self) -> Result<(), EspError>;
    fn set_low(&mut self) -> Result<(), EspError>;
}

impl<'d, T: Pin + InputPin + OutputPin> PinDriverTraitOutput for PinDriver<'d, T, InputOutput> {
    fn set_high(&mut self) -> Result<(), EspError> {
        PinDriver::set_high(self)
    }
    fn set_low(&mut self) -> Result<(), EspError> {
        PinDriver::set_low(self)
    }
    fn set_pull(&mut self, pull: Pull) -> Result<(), EspError> {
        PinDriver::set_pull(self, pull)
    }
    fn get_value(&mut self) -> Result<bool, EspError> {
        Ok(PinDriver::get_level(self) == Level::High)
    }
}

#[derive(Debug)]
pub struct SsnChannelInfo {
    pub index: usize,
    pub name: &'static str,
    pub description: &'static str,
}

pub struct SsnDevice<'a> {
    pub dev_id: String,
    pub period: SsnSensorPeriod,
    pub is_active: bool,
    pub is_paused: bool,
    pub device: SsnDevices<'a>,
}

pub type SharedSsnDevice<'a> = Arc<Mutex<SsnDevice<'a>>>;

pub fn create_shared_devices(devices: Vec<SsnDevice>) -> Vec<SharedSsnDevice> {
    devices
        .into_iter()
        .map(|d| Arc::new(Mutex::new(d)))
        .collect()
}

impl<'a> SsnDevice<'a> {
    pub fn get_device_by_id<'b>(
        devices: &'b [SharedSsnDevice<'b>],
        dev_id: &str,
    ) -> Option<SharedSsnDevice<'b>> {
        devices
            .iter()
            .find(|d| d.lock().unwrap().dev_id == dev_id)
            .map(Arc::clone)
    }

    pub fn dev_init(&mut self) {
        log::info!("device {} preinit", self.dev_id);
        if let SsnDevices::GpioInput {
            sensor,
            notificator,
            pull_type,
            interrupt_type,
            ..
        } = &mut self.device
        {
            log::info!(
                "GpioInput preinit set_pull={:?}, interrupt_type={:?}",
                *pull_type,
                *interrupt_type
            );
            let _ = sensor.set_pull(*pull_type);
            let _ = sensor.set_interrupt_type(*interrupt_type);
            let _ = sensor.subscribe_boxed(notificator.clone());
            let _ = sensor.enable_interrupt();
        }
    }

    pub fn add_shared_device(
        devices: &mut Vec<SharedSsnDevice<'a>>,
        mut new_device: SsnDevice<'a>,
    ) {
        new_device.dev_init();
        devices.push(Arc::new(Mutex::new(new_device)));
    }
}

impl<'a> SsnDevices<'a> {
    // pub fn get_value_by_channel(
    //     devices: &[SsnDevice],
    //     dev_id: &str,
    //     channel_index: usize,
    // ) -> Option<f32> {
    //     devices
    //         .iter()
    //         .find(|d| d.dev_id == dev_id)
    //         .and_then(|device| device.get_value_by_channel(channel_index))
    // }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum DeviceType {
    Bmp,
    Bmp180,
    Ens160,
    Adc,
    Aht21,
    StoreInt,
    GpioInput,
    GpioOutput,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct DeviceConfig {
    pub dev_id: String,
    pub device_type: DeviceType,
    pub period: Option<String>, // "Fast", "Middle", "Slow"
    pub is_active: Option<bool>,
    pub is_paused: Option<bool>,

    // Device-specific configuration
    pub sensor_type: Option<String>, // For BMP devices
    pub temperature_delta_over: Option<f64>,
    pub pressure_delta_over: Option<f64>,
    pub humidity_delta_over: Option<f64>,
    pub co2eq_delta_over: Option<f32>,
    pub tvoc_delta_over: Option<f32>,
    pub aqi_delta_over: Option<f32>,
    pub delta_over: Option<f32>, // For ADC and generic devices

    // GPIO-specific configuration
    pub gpio_pin: Option<i32>,
    pub interrupt_type: Option<String>, // "AnyEdge", "PosEdge", "NegEdge", "LowLevel", "HighLevel"
    pub pull_type: Option<String>,      // "None", "Up", "Down"
    pub initial_state: Option<bool>,    // For GPIO output

    // ADC-specific configuration
    pub adc_channel: Option<u8>,
    pub voltage_reference: Option<f32>, // in mV

    // I2C-specific configuration
    pub i2c_address: Option<u8>,

    // StoreInt-specific configuration
    pub store_keys: Option<Vec<String>>,

    // Additional metadata
    pub description: Option<String>,
    pub unit: Option<String>,
    pub min_value: Option<f32>,
    pub max_value: Option<f32>,
}
