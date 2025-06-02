use adc_reader::AdcReaderTrait;
// #![cfg(feature = "smart-leds-trait")]
use anyhow::{Context, Result};
use chrono::DateTime;
use chrono::FixedOffset;
use chrono::Utc;
use esp_idf_hal::gpio::AnyInputPin;
use esp_idf_hal::gpio::AnyOutputPin;
use esp_idf_hal::gpio::Input;
use esp_idf_hal::gpio::InputOutput;
use esp_idf_hal::gpio::InputPin;
use esp_idf_hal::gpio::InterruptType;
use esp_idf_hal::gpio::Level;
use esp_idf_hal::gpio::OutputPin;
use esp_idf_hal::gpio::Pin;
use esp_idf_hal::gpio::Pull;
use esp_idf_hal::task::notification::Notification;
use esp_idf_sys::EspError;
use core::str;
use std::any::Any;
use std::num::NonZero;
use embedded_hal::i2c::ErrorType;
use embedded_hal::i2c::I2c;
use esp_idf_hal::peripherals;
use std::sync::atomic::{AtomicU32, Ordering};
use chrono::{Local, Timelike, TimeZone, Datelike};

use esp_idf_svc::mqtt::client::EspMqttEvent;
use esp_idf_svc::mqtt::client::EventPayload;
use esp_idf_svc::sntp::{EspSntp, SyncMode};

use log::error;
use std::borrow::Borrow;
use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;
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
use commons::Colors;
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

// ----------------------------------
// if sensor value is not changed in this period mqtt message will by sended anyway
const MAX_PERIOD_REFRESH_MQTT: u64 = 60; // sec.
const MAX_RECONNECT_ATTEMPTS: u32 = 5;
const RECONNECT_DELAY_MS: u64 = 5000;
const MAIN_LOOP_DELAY_MS: u64 = 100;
const MIDDLE_LOOP_PERIOD_MS: u64 = 20000;
const SLOW_LOOP_PERIOD_MS: u64 = 60000;
static DEF_TZ: i32 = 3; // default timezone (may by overrided from settings)

// ----------------------------------
static INTERRUPT_COUNTER_GPIO: AtomicU32 = AtomicU32::new(0);

// struct SharedPinDriver<T: Pin>(Arc<Mutex< PinDriver<'static, T, Input>>>);
lazy_static! {
    static ref INTERRUPT_GPIO_HASHMAP: Mutex<[AtomicU32; 10]> = Mutex::new([
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
]);
}

lazy_static! {
    // static ref I2C: Mutex<Option<I2cDriver<'static>>> = Mutex::new(None);
    static ref I2C: Mutex<Option<Arc<Mutex<I2cDriver<'static>>>>> = Mutex::new(None);
}

// ----------------------------------
// Alarm structure
#[derive(Clone)]
struct Alarm {
    trigger_time: DateTime<Utc>,
    active_duration_secs: u32,
    dev_id: String,
    active_level: u8,
    inactive_level: u8,
    previously_active: bool,
}

// Shared alarm storage
lazy_static! {
    static ref ALARMS: Mutex<Vec<Alarm>> = Mutex::new(Vec::new());
}

// Add new alarm
fn add_alarm(trigger_time: DateTime<FixedOffset>, duration_secs: u32, dev_id: String, active_level: u8, inactive_level: u8) {
    let trigger_time_utc = trigger_time.with_timezone(&Utc);
    ALARMS.lock().unwrap().push(Alarm {
        trigger_time: trigger_time_utc,
        active_duration_secs: duration_secs,
        dev_id,
        active_level,
        previously_active: false,
        inactive_level,
    });
    log::info!("added alarm: {:?} [at UTC={:?}], duration = {}", trigger_time, trigger_time_utc, duration_secs);
}

fn check_alarms() -> Vec<(String, u8)> {
    let now = Utc::now();
    let current_time = now.time();
    // let offset = FixedOffset::east_opt(DEF_TZ * 3600).unwrap(); // Seconds
    // let now = Local::now().with_timezone(&offset);
 
    let mut alarms = ALARMS.lock().unwrap();
    let mut actions: Vec<(String, u8)> = Vec::new();

    for alarm in alarms.iter_mut() {
        // TO DO: work with absolute dates or hourly/minutly etc variants
        let alarm_time = alarm.trigger_time.time();
        let end_time = alarm_time + chrono::Duration::seconds(alarm.active_duration_secs as i64);
        let is_active = if alarm_time <= end_time {
            current_time >= alarm_time && current_time <= end_time
        } else {
            // Handle midnight wrap-around
            current_time >= alarm_time || current_time <= end_time
        };

        // let is_active = now >= alarm.trigger_time && 
        //                 now <= alarm.trigger_time + chrono::Duration::seconds(alarm.active_duration_secs as i64);
        
        // Only return actions when state changes
        if is_active != alarm.previously_active {
            let level = if is_active { alarm.active_level } else { alarm.inactive_level };
            actions.push((alarm.dev_id.clone(), level));
            alarm.previously_active = is_active;
        }
    }

    actions
}

// ----------------------------------
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
    Middle,
    Slow,
}

enum SsnDevices<'a> {
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
        sensor: Box<dyn AdcReaderTrait + 'a>,
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
    },
    StoreInt {
        data: HashMap<String, i32>,
        delta: f32,
        delta_over: Option<f32>, // Skip if value exceeds this threshold (None = no limit)
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
        delta_over: Option<f32>, // Skip if value exceeds this threshold (None = no limit)
        last_update: Instant,
        last_sent_value: Option<u8>,
        last_sent_counter: Option<u32>,
        sensor: Box<dyn PinDriverBasicInput + 'static>,
    },
    GpioOutput {
        data: bool,
        delta: f32,
        delta_over: Option<f32>, // Skip if value exceeds this threshold (None = no limit)
        last_update: Instant,
        last_sent_value: Option<bool>,
        sensor:  Box<dyn PinDriverTraitOutput + 'a>,
    },
}

pub trait PinDriverBasicInput: Any+Send {
    fn set_pull(&mut self, pull: Pull)-> Result<(), esp_idf_sys::EspError>;
    fn get_value(&mut self) -> Result<bool, esp_idf_sys::EspError>;
    fn set_interrupt_type(&mut self, interript_type: InterruptType) -> Result<(), esp_idf_sys::EspError>;
    // fn subscribe_boxed(&mut self, callback: Box<dyn FnMut() + Send + 'static>) -> Result<(), EspError>;
    fn subscribe_boxed(&mut self, nf: Arc<Notification>) -> Result<(), EspError>;
    fn enable_interrupt(&mut self) -> Result<(), esp_idf_sys::EspError>;
    fn get_pin(&mut self) -> Result<i32, esp_idf_sys::EspError>;
}

impl dyn PinDriverBasicInput {
    pub fn as_any(&mut self) -> &mut dyn Any {
        self
    }
}
// pub trait PinDriverInterrupt: PinDriverBasicInput {
//     // fn subscribe<F>(&mut self, callback: F) -> Result<(), esp_idf_sys::EspError> where
//     //     F: FnMut() + 'static  + Send;
//         fn subscribe_boxed(&mut self, callback: Box<dyn FnMut() + Send + 'static>) -> Result<(), EspError>;
//     }

impl<T: Pin + InputPin + OutputPin> PinDriverBasicInput for PinDriver<'static, T, Input> {
    fn get_pin(&mut self) -> Result<i32, esp_idf_sys::EspError> {
        Ok(PinDriver::pin(self))
    }
    fn get_value(&mut self) -> Result<bool, EspError> {
        Ok(PinDriver::get_level(self) == Level::High)
    }
    fn set_pull(&mut self, pull: Pull) -> Result<(), esp_idf_sys::EspError>{
        PinDriver::set_pull(self, pull)
    }
    fn set_interrupt_type(&mut self, interript_type: InterruptType) -> Result<(), esp_idf_sys::EspError> {
        PinDriver::set_interrupt_type(self, interript_type)
    }
    // fn subscribe_boxed(&mut self, callback: Box<dyn FnMut() + Send + 'static>) -> Result<(), EspError> {
    //     log::info!("GpioInput subscribe callback");
    //     unsafe { PinDriver::subscribe(self, callback) }
    // }
    fn subscribe_boxed(&mut self, nf: Arc<Notification>) -> Result<(), EspError> {
        // unsafe { PinDriver::subscribe(self,move || interrupt_handler_gpio_t()) }
        let notifier = nf.notifier().clone();
        let pin_number = self.pin() as u32;
        log::info!("GpioInput subscribe callback for pin {}", pin_number);
        unsafe { PinDriver::subscribe(self,move || 
            {
                // Increment the counter atomically
                INTERRUPT_COUNTER_GPIO.fetch_add(1, Ordering::SeqCst);
                INTERRUPT_GPIO_HASHMAP.lock().unwrap()[pin_number as usize].fetch_add(1, Ordering::SeqCst);
                
                // Notify the main task
                notifier.notify_and_yield(std::num::NonZeroU32::new(pin_number).unwrap());
            }) }
    }
    fn enable_interrupt(&mut self) -> Result<(), esp_idf_sys::EspError> {
        PinDriver::enable_interrupt(self)
    }
}

pub trait PinDriverTraitOutput {
    fn set_pull(&mut self, pull: Pull)-> Result<(), esp_idf_sys::EspError>;
    fn get_value(&mut self) -> Result<bool, esp_idf_sys::EspError>;
    fn set_high(&mut self) -> Result<(), esp_idf_sys::EspError>;
    fn set_low(&mut self) -> Result<(), esp_idf_sys::EspError>;
}
impl<'d, T: Pin + InputPin + OutputPin> PinDriverTraitOutput for PinDriver<'d, T, InputOutput> {
    fn set_high(&mut self) -> Result<(), esp_idf_sys::EspError> {
        PinDriver::set_high(self)
    }
    fn set_low(&mut self) -> Result<(), esp_idf_sys::EspError> {
        PinDriver::set_low(self)
    }
    fn set_pull(&mut self, pull: Pull) -> Result<(), esp_idf_sys::EspError>{
        PinDriver::set_pull(self, pull)
    }
    
    fn get_value(&mut self) -> Result<bool, esp_idf_sys::EspError> {
        Ok(PinDriver::get_level(self) == Level::High)
    }
}

#[derive(Debug)]
pub struct SsnChannelInfo {
    pub index: usize,
    pub name: &'static str,
    pub description: &'static str,
}

impl <'a> SsnDevices <'a>{
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
impl<'a> SsnDevice<'a> {
    pub fn get_device_by_id<'b>(
        devices: &'b[SharedSsnDevice<'b>],
        dev_id: &str,
    ) -> Option<SharedSsnDevice<'b>> {
        devices
            .iter()
            .find(|d| d.lock().unwrap().dev_id == dev_id)
            .map(Arc::clone)
    }
    // func is called at device creating:
    pub fn dev_init(&mut self) {
        log::info!("device {} preinit", self.dev_id);
        match &mut self.device {
            SsnDevices::GpioInput { data, delta, interrupt_type, pull_type, delta_over, last_update, last_sent_value, sensor, notificator, counter, last_sent_counter } => {
                log::info!("GpioInput preinit set_pull={:?}, interrupt_type={:?}", *pull_type, *interrupt_type);
                let _ = sensor.set_pull(*pull_type);
                let _ = sensor.set_interrupt_type(*interrupt_type);

                log::info!("GpioInput subscribe callback and enable interrupt");
                // let _ = sensor.subscribe_boxed(Box::new(|| interrupt_handler_gpio_t()));
                let _ = sensor.subscribe_boxed(notificator.clone());
                let _ = sensor.enable_interrupt();

                // if let Some(base) = sensor.as_any().downcast_mut::<Box<dyn PinDriverInterrupt>>() {
                //     log::info!("GpioInput subscribe int callback");
                //     let _ = base.subscribe_boxed(Box::new(|| interrupt_handler_gpio_t()));
                // }
            },
            SsnDevices::GpioOutput { data, delta, delta_over, last_update, last_sent_value, sensor } => {
                // let _ = sensor.set_pull(*pull_type);
            },
            _ => (),
        }
    }
}

// #[derive(Default)]
struct SsnDevice <'a>{
    dev_id: String,
    period: SsnSensorPeriod,
    is_active: bool,
    is_paused: bool,
    device: SsnDevices<'a>,
}
type SharedSsnDevice <'a>= Arc<Mutex<SsnDevice<'a>>>;

fn create_shared_devices(
    devices: Vec<SsnDevice>,
) -> Vec<SharedSsnDevice> {
    devices
        .into_iter()
        .map(|d| Arc::new(Mutex::new(d)))
        .collect()
}

impl <'a>SsnDevice <'a>{
    pub fn add_shared_device(devices: &mut Vec<SharedSsnDevice<'a>>, mut new_device: SsnDevice<'a>) {
        new_device.dev_init(); // preinit device if needed
        devices.push(Arc::new(Mutex::new(new_device)));
    }
}

// fn read_device_value(
//     device: &SharedSsnDevice,
//     channel: usize,
// ) -> Option<f32> {
//     let guard = device.lock().unwrap(); // Blocks until lock acquired
//     guard.get_value_by_channel(channel)
// }

type SharedSensorData = Arc<Mutex<HashMap<String, Vec<f32>>>>;

fn create_shared_data() -> SharedSensorData {
    Arc::new(Mutex::new(HashMap::new()))
}

// update function
fn update_sensor_data(data: &SharedSensorData, dev_id: String, values: Vec<f32>) -> Result<()> {
    let mut guard = data.lock().unwrap();
    guard.insert(dev_id, values);
    Ok(())
}

// read function
fn get_sensor_values(data: &SharedSensorData, dev_id: &str) -> Option<Vec<f32>> {
    let guard = data.lock().unwrap();
    guard.get(dev_id).cloned()
}

struct SsnDeviceCommand {
    dev_id: String,
    channel: u8,
    value: f32,
}

type SharedSsnDeviceCommand = Arc<Mutex<Vec<SsnDeviceCommand>>>;
fn create_commands_array() -> SharedSsnDeviceCommand {
    Arc::new(Mutex::new(Vec::new()))
}
// push command
fn push_command(data: &SharedSsnDeviceCommand, command: SsnDeviceCommand) -> Result<()> {
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
// push command
fn pop_command(data: &SharedSsnDeviceCommand) -> Option<SsnDeviceCommand> {
    let mut guard = data.lock().unwrap();
    guard.pop()
}

// =====================================================================================
// Main logic that might return an error
fn main_logic() -> Result<()> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let mut ssn_devices = vec![];
    let mut shared_devices = create_shared_devices(ssn_devices);
    let shared_data = create_shared_data();
    let shared_data_ref = shared_data.clone();

    let commands_array = create_commands_array();
    let commands_array_ref = commands_array.clone();

    // Create a notification handle for the interrupt
    let notification = Arc::new(Notification::new());
    let notification_clone = notification.clone();

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
    let account_ref = app_config.account.clone();

    log::info!(
        "Retrieved config: {}, size={}",
        app_config.clone(),
        app_config_size
    );

    let peripherals = esp_idf_hal::peripherals::Peripherals::take().unwrap();

    // LED 2812 ---------------------------------------------
    let led_pin = peripherals.pins.gpio10;
    let channel = peripherals.rmt.channel0;

    let mut ws2812 = Ws2812Esp32RmtDriver::new(channel, led_pin).unwrap(); // GPIO 10, RMT channel 0

    pub fn status_led(led: &mut Ws2812Esp32RmtDriver, col: Colors) {
        let color = match col {
            Colors::Red => LedPixelColorGrb24::new_with_rgb(30, 0, 0),
            Colors::Green => LedPixelColorGrb24::new_with_rgb(0, 30, 0),
            Colors::Blue => LedPixelColorGrb24::new_with_rgb(0, 0, 30),
            Colors::Yellow => LedPixelColorGrb24::new_with_rgb(30, 30, 0),
            Colors::White => LedPixelColorGrb24::new_with_rgb(30, 30, 30),
            Colors::Black => LedPixelColorGrb24::new_with_rgb(0, 0, 0),
        };
        let pixel: [u8; 3] = color.as_ref().try_into().unwrap();
        led.write_blocking(pixel.into_iter()).unwrap();
    }

    // let _led = PinDriver::output(peripherals.pins.gpio10).unwrap();

    status_led(&mut ws2812, Colors::Yellow);

    // ADC ---------------------------------------------
    let adc1 = peripherals.adc1;

    // ssn_devices.push(SsnDevice {
    let new_device = SsnDevice {
        dev_id: "adc-test".to_string(),
        is_active: true,
        is_paused: false,
        period: SsnSensorPeriod::Fast,
        device: SsnDevices::Adc {
            raw_value: 0,
            voltage_mv: 0.0,
            raw_value_delta: 1.0, // 1%
            raw_value_last_sent_value: Some(0),
            raw_value_last_update: Instant::now() - Duration::from_secs(60 * 60 * 24 * 365 * 30),
            sensor: Box::new(AdcReader::new(peripherals.pins.gpio4, adc1)
                .context("Failed to create ADC reader")?),
            delta_over: Some(50.0),
            voltage_value_last_sent_value: Some(0.0),
        },
    };
    SsnDevice::add_shared_device(&mut shared_devices, new_device);

    // let pins = peripherals.pins;
    let scl = peripherals.pins.gpio0;
    let sda = peripherals.pins.gpio1;

    // // Initialize Wi-Fi
    let sysloop = EspSystemEventLoop::take()?;
    let mut my_wifi = ssn_wifi::wifi(
        app_config.wifi_ssid.as_str(),
        app_config.wifi_psk.as_str(),
        peripherals.modem,
        sysloop.clone(),
    )
    .map_err(|e| {
        // let pixel: [u8; 3] = red.as_ref().try_into().unwrap();
        // ws2812.write_blocking(pixel.clone().into_iter()).unwrap();
        status_led(&mut ws2812, Colors::Red);
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
    status_led(&mut ws2812, Colors::Green);

    log::info!("Our SSID is:");
    log::info!("{}", app_config.wifi_ssid);

    // let sda = peripherals.pins.gpio13;
    // let scl = peripherals.pins.gpio14;
    let i2c = peripherals.i2c0;
    let config = I2cConfig::new().baudrate(400.kHz().into());
    let i2c_driver = I2cDriver::new(i2c, sda, scl, &config)?;

    fn init_i2c(i2c: I2cDriver<'static>) {
        *I2C.lock().unwrap() = Some(Arc::new(Mutex::new(i2c)));
        log::info!("I2C initialized");
    }

    let i2c_lock = Arc::new(Mutex::new(i2c_driver));

    let shared_i2c = SharedI2c::new(i2c_lock.clone()); // Your wrapper implementing `I2c`

    // // Initialize AHT21 sensor
    // let mut aht21 = AHT21Sensor::new(shared_i2c.clone()).unwrap();

    // // Read sensor data
    // let (temperature, humidity) = aht21.get_data().unwrap();
    // log::info!("temperature (aht21): {:.2}C", temperature);
    // log::info!("humidity (aht21): {:.2}%", humidity);

    // let mut pin_b = Box::new(PinDriver::output(peripherals.pins.gpio5)?);
    // pin_b.set_high();

        SsnDevice::add_shared_device(
        &mut shared_devices,
        SsnDevice {
            dev_id: "gpio-out5".to_string(),
            period: SsnSensorPeriod::Fast,
            is_active: true,
            is_paused: false,
            device: SsnDevices::GpioOutput { data: false,
                 delta: 0.0, delta_over: None, last_update: Instant::now() - Duration::from_secs(60 * 60 * 24 * 365 * 30), 
                 last_sent_value: None, 
                //  sensor: peripherals.pins.gpio5.into(),
                 sensor: Box::new(PinDriver::input_output(peripherals.pins.gpio5)?),
                },
        },
    );
    SsnDevice::add_shared_device(
        &mut shared_devices,
        SsnDevice {
            dev_id: "gpio-in6".to_string(),
            period: SsnSensorPeriod::Fast,
            is_active: true,
            is_paused: false,
            device: SsnDevices::GpioInput { data: 0,
                 delta: 0.0, delta_over: None, last_update: Instant::now() - Duration::from_secs(60 * 60 * 24 * 365 * 30), 
                 last_sent_value: None, 
                 interrupt_type: InterruptType::AnyEdge,
                 pull_type: Pull::Up,
                 sensor: Box::new(PinDriver::input(peripherals.pins.gpio6)?),
                notificator: notification_clone.clone(),
                counter: 0,
                last_sent_counter: None,
                },
                
        },
    );
    SsnDevice::add_shared_device(
        &mut shared_devices,
        SsnDevice {
            dev_id: "gpio-in7".to_string(),
            period: SsnSensorPeriod::Fast,
            is_active: true,
            is_paused: false,
            device: SsnDevices::GpioInput { data: 0,
                 delta: 0.0, delta_over: None, last_update: Instant::now() - Duration::from_secs(60 * 60 * 24 * 365 * 30), 
                 last_sent_value: None, 
                 interrupt_type: InterruptType::AnyEdge,
                 pull_type: Pull::Up,
                 sensor: Box::new(PinDriver::input(peripherals.pins.gpio7)?),
                notificator: notification_clone.clone(),
                counter: 0,
                last_sent_counter: None,
                },
                
        },
    );
    SsnDevice::add_shared_device(
        &mut shared_devices,
        SsnDevice {
            dev_id: "aht21-test".to_string(),
            period: SsnSensorPeriod::Middle,
            is_active: true,
            is_paused: false,
            device: SsnDevices::Aht21 {
                temperature: 0.0,
                temperature_delta: 0.5, // 0.5%
                temperature_last_sent_value: None,
                temperature_last_update: Instant::now()
                    - Duration::from_secs(60 * 60 * 24 * 365 * 30),
                humidity: 0.0,
                humidity_delta: 0.5, // 0.5%
                humidity_last_sent_value: None,
                humidity_last_update: Instant::now() - Duration::from_secs(60 * 60 * 24 * 365 * 30),
                sensor: AHT21Sensor::new(SharedI2c::new(i2c_lock.clone())).unwrap(),
                temperature_delta_over: Some(30.0),
                humidity_delta_over: Some(30.0),
            },
        },
    );

    SsnDevice::add_shared_device(
        &mut shared_devices,
        SsnDevice {
            dev_id: "ens160-test".to_string(),
            period: SsnSensorPeriod::Slow,
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
                sensor: ens160::ENS160Sensor::new(shared_i2c).unwrap(),
                co2eq_delta_over: Some(20.0),
                tvoc_delta_over: Some(20.0),
                aqi_delta_over: Some(30.0),
            },
        },
    );

    SsnDevice::add_shared_device(
        &mut shared_devices,
        SsnDevice {
            dev_id: "bmp180-test".to_string(),
            period: SsnSensorPeriod::Slow,
            is_active: true,
            is_paused: false,
            device: SsnDevices::Bmp180 {
                temperature: (0.0),
                temperature_delta: 0.5, // 0.5%
                temperature_last_sent_value: None,
                temperature_last_update: Instant::now()
                    - Duration::from_secs(60 * 60 * 24 * 365 * 30),
                pressure: (0),
                pressure_delta: 0.5, // 0.5%
                pressure_last_sent_value: None,
                pressure_last_update: Instant::now() - Duration::from_secs(60 * 60 * 24 * 365 * 30),
                altitude: (0.0),
                sensor: (bmp180::BMP180Sensor::new(SharedI2c::new(i2c_lock.clone()), FreeRtos)?),
                temperature_delta_over: Some(10.0),
                pressure_delta_over: Some(5.0),
            },
        },
    );

    SsnDevice::add_shared_device(
        &mut shared_devices,
        SsnDevice {
            dev_id: "store_int-test".to_string(),
            period: SsnSensorPeriod::Fast,
            is_active: true,
            is_paused: false,
            device: SsnDevices::StoreInt {
                data: HashMap::new(),
                delta: 1.0,
                last_sent_value: HashMap::new(),
                last_update: HashMap::new(),
                delta_over: None,
            },
        },
    );

    // ---------------------------- Alarms:
    add_alarm(
        FixedOffset::east_opt(DEF_TZ * 3600).unwrap().with_ymd_and_hms(2025, 8, 2, 0, 25, 0).unwrap(),
        20,
        "gpio-out5".to_string(),
        1,
        0
    );

    // MQTT Client configuration *******************************************************

    log::info!("MQTT Client configuration");
    let mut reconnect_attempts = 0;

    let topics_to_subscribe = vec![
        format!("/ssn/acc/{}/raw_data", &account),
        format!("/ssn/acc/{}/obj/{}/commands", &account, &object),
        format!("/ssn/acc/{}/obj/{}/commands/ini", &account, &object),
        format!("/ssn/acc/{}/obj/{}/commands/json", &account, &object),
        format!(
            "/ssn/acc/{}/obj/{}/commands/device/+/+/in",
            &account, &object
        ),
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

    log::info!("SSN. Hello world!");

    // let mut client = EspMqttClient::new_cb(&broker_url, &mqtt_config, move |_message_event| {})?;
    // let (mut client, mut connection) = EspMqttClient::new(&broker_url, &mqtt_config)?;
    let mut client = EspMqttClient::new_cb(&broker_url, &mqtt_config, move |message_event| {
        match message_event.payload() {
//                                     ********************************* process receive data from topic:
            EventPayload::Received { data, topic, .. } => {
                log::info!("Received message on {:?}: {:?}", topic, data);
                if let Some(topic) = topic {
                    // mqtt_messages::process_message(topic, data);
                    // let (account_rcv, root_token, subtokens) = mqtt_messages::parse_topic(&topic);

                    let float_str = std::str::from_utf8(data).unwrap(); // Convert to string
                    if let Some((account_rcv, root_token, subtokens)) = mqtt_messages::parse_topic(topic) {
                        log::info!("account_rcv = {}, root_token = {}, subtokens = {:?}", account_rcv, root_token, subtokens);
                        // filter only our account
                        if account_ref.eq(account_rcv) {
                            if let Some(parsed) = mqtt_messages::parse_token_array(root_token, subtokens) {
                                // Use the parsed HashMap here
                                log::info!("parsed subtokens HashMap = {:?}", parsed);
                                match *parsed.get("subToken").unwrap() {
                                    // process hi level commands:
                                    "commands" => {
                                        // process device operations:
                                        let device = *parsed.get("device").unwrap();
                                        let channel = *parsed.get("channel").unwrap();
                                        match channel.parse::<u8>() {
                                            Ok(channel_u8) => {
                                                match *parsed.get("action").unwrap() {
                                                    "in" => {
                                                        // process set new device value
                                                        match float_str.parse::<f32>() {
                                                            Ok(val) => {
                                                                log::info!("Parsed float: {}", val);
                                                                let _ = push_command(
                                                                    &commands_array_ref.clone(),
                                                                    SsnDeviceCommand {
                                                                        dev_id: device.to_string(),
                                                                        channel: channel_u8,
                                                                        value: val,
                                                                    },
                                                                );
                                                            }
                                                            Err(e) => {
                                                                log::info!("Error parsing float: {}", e)
                                                            }
                                                        }
                                                    },
                                                    "reset" => {
                                                        // TO DO
                                                    }, 
                                                    &_ => {}
            
                                                }
                                            }
                                        Err(_) => todo!(),
                                        }
                                    },
                                    &_ => {}

                                }
                                         } else {
                                log::warn!("Failed to parse tokens for topic: {}", topic);
                            }
                        }
    
                }
                }
                

                if let Some(vals) = get_sensor_values(&shared_data.clone(), "aht21-test") {
                    log::info!("sensor val: {:?}", vals);
                };
                // let value = read_device_value(&shared_devices[0], 0);
                // let sensor1 = SsnDevice::get_device_by_id(&shared_devices, "aht21-test");
                // if let Some(sensor) = sensor1 {
                //     log::info!("sensor val: {:?}", read_device_value(&sensor, 0));
                // }
                // SsnDevices::get_value_by_channel(&ssn_devices, "aht21-test", 0);
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

    std::thread::sleep(Duration::from_millis(4000));
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

    // preinit time counters:
    // let mut time_counter_freq = Instant::now();
    let mut time_counter_middle = Instant::now();
    let mut time_counter_seldom = Instant::now();

    fn is_fire_measurement(
        dev_period: &SsnSensorPeriod,
        time_counter_middle: &mut Instant,
        time_counter_seldom: &mut Instant,
    ) -> bool {
        match dev_period {
            SsnSensorPeriod::Slow => {
                if time_counter_seldom.elapsed() > Duration::from_millis(SLOW_LOOP_PERIOD_MS) {
                    log::info!("fire seldom {:?}", time_counter_seldom);
                    return true;
                }
            }
            SsnSensorPeriod::Middle => {
                if time_counter_middle.elapsed() > Duration::from_millis(MIDDLE_LOOP_PERIOD_MS) {
                    log::info!("fire middle {:?}", time_counter_middle);
                    return true;
                }
            }
            SsnSensorPeriod::Fast => {
                return true;
            }
        }
        false
    }

    // ********************* main loop ****************************
    loop {

        // Check WiFi connection status
        if !ssn_wifi::check_wifi_connection(my_wifi.wifi()) {
            log::warn!("WiFi connection lost. Attempting to reconnect...");
            status_led(&mut ws2812, Colors::Red);

            my_wifi.stop()?;
            my_wifi.start()?;

            if let Err(e) = ssn_wifi::connect_wifi(&mut my_wifi, &sysloop.clone()) {
                log::error!("Failed to reconnect WiFi: {:?}", e);
                // You might want to add a delay here before retrying
                std::thread::sleep(Duration::from_secs(10));
                continue;
            }
        }

        let cur_time = Utc::now();
        let now = Instant::now();
        log::info!("New measurment circle t={:?}..", cur_time);
        
        // TO DO: Trigger at 14:30 every day .. change to flexible logic
        // if cur_time.hour() == 14 && cur_time.minute() == 30 {
        //     println!("Scheduled task triggered at {:?}", cur_time);
        //     // Your task logic here
        // }

        // check for alarms and make command for device action:
        // check_alarms returm actual dev_id and value only at the moment of timer triggered or canceled
        check_alarms().iter().for_each(|alarm| {
            log::info!("Alarm triggered dev_id={}, value={}", alarm.0, alarm.1);
            let _ = push_command(
                &commands_array.clone(),
                SsnDeviceCommand {
                    dev_id: alarm.0.clone(),
                    channel: 0,
                    value: alarm.1 as f32,
                });
        });

        // log::info!(
        //     "now = {:?},
        //     \ntime_counter_middle = {:?} \ntime_counter_seldom = {:?}",
        //     &now,
        //     &time_counter_middle,
        //     &time_counter_seldom
        // );
        status_led(&mut ws2812, Colors::Blue);

        FreeRtos::delay_ms(100);
        let (command_dev_id, command_channel, command_value) =
            if let Some(command) = pop_command(&commands_array) {
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
                let count = INTERRUPT_COUNTER_GPIO.load(Ordering::SeqCst);
                println!("Interrupt triggered! pin = {} Total count: {}", value, count);
                Some(u32::from(value))
            } else {None};

        // log::info!(
        //     "pop_command: command_dev_id={}, command_channel={}, command_value={}",
        //     command_dev_id,
        //     command_channel,
        //     command_value
        // );

        // iterate through all devices, refresh their data and publish it:
        shared_devices.iter().for_each(|device| {
            let mut cur_device = device.lock().unwrap(); // Blocks until lock acquired
            let dev = cur_device.dev_id.to_string();


            // check measure or not in this circle:
            if is_fire_measurement(&cur_device.period, &mut time_counter_middle, &mut time_counter_seldom) {

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
                                        );
                                        // Handle raw update
                                        let _ = handle_sensor_value_update(
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
                                        );
                                        let _ = update_sensor_data(&shared_data_ref, dev, vec![*voltage_mv, *raw_value as f32]);
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
                                        let (new_temp, new_pressure, new_alt) =
                                            sensor.get_data().context("bmp180 read data error").unwrap();

                                        // Calculate percentage changes
                                        let temp_change_pct = calculate_percentage_change(new_temp, *temperature);
                                        let pressure_change_pct =
                                            calculate_percentage_change(new_pressure as f32, *pressure as f32);

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
                                        );

                                        // Handle pressure update
                                        let _ = handle_sensor_value_update(
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
                                        );
                                        let _ = update_sensor_data(&shared_data_ref, dev, vec![*temperature, *pressure as f32]);
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
                                        if dev.eq(&command_dev_id) {
                                            log::info!("set value StoreInt {}", command_value);
                                            data.insert(command_channel.to_string(), command_value as i32);
                                        };
                                        // iterate for all values:
                                        data.iter_mut().for_each(|d| {
                                            let old_value = *d.1;
                                            let new_value = old_value;


                                            // Calculate percentage changes
                                            let change_pct = calculate_percentage_change(new_value as f32,old_value as f32);

                                            // Handle update
                                            let mut cur_last_sent_value = 
                                                match last_sent_value.get(d.0) {
                                                Some(v) => v.map(|x| x as f32),
                                                    None => None
                                            };
                                            let mut cur_last_update = 
                                                match last_update.get(d.0) {
                                                    Some(v) => *v,
                                                    None => Instant::now() - Duration::from_secs(60 * 60 * 24 * 365 * 30),
                                                                                    };

                                            let _ = handle_sensor_value_update(
                                                &mut client,
                                                old_value as f32,
                                                &mut cur_last_sent_value,
                                                &mut cur_last_update,
                                                change_pct,
                                                delta,
                                                delta_over.as_ref(),
                                                now,
                                                Duration::from_secs(MAX_PERIOD_REFRESH_MQTT),
                                                &mqtt_messages::publish_sensor_topic(&account, &object, &dev, d.0),
                                                "storeInt",
                                            );
                                            // let _ = update_sensor_data(&shared_data_ref, dev, vec![data]);
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
                                        let _ = handle_sensor_value_update(
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
                                        );
                                        // Handle co2eq update
                                        let _ = handle_sensor_value_update(
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
                                        ); // Handle co2eq update
                                        let _ = handle_sensor_value_update(
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
                                        );
                                        let _ = update_sensor_data(&shared_data_ref, dev, vec![*co2eq_ppm as f32, *tvoc_ppb as f32, *aqi  as f32]);
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
                                        let (new_temp, new_humidity) = sensor.get_data().unwrap();
                                        //  {
                                        //     Ok(data) => data,
                                        //     // Err(e) => {
                                        //     //     log::error!("AHT21 read failed: {:?}", e);
                                        //     //     // continue; // Skip this iteration if reading fails
                                        //     // }
                                        // };

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
                                        );
                                        let _ = handle_sensor_value_update(
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
                                            "aht21-humidity",
                                        );
                                        let _ = update_sensor_data(&shared_data_ref, dev, vec![*temperature, *humidity]);
                                    },
                    SsnDevices::GpioInput {
                        data, 
                        delta, 
                        delta_over, 
                        last_update, 
                        last_sent_value, 
                        sensor, 
                        interrupt_type, 
                        pull_type,
                        notificator,
                        counter,
                        last_sent_counter, 
                        } => {
                        if dev.eq(&command_dev_id) {
                            log::info!("set value {}", command_value);
                            *data = command_value as u8;
                            // sensor.into()
                        };
                        let current_pin = sensor.get_pin().unwrap() as u32;
                        let old_counter = *counter;

                        if pin_num == Some(current_pin) {
                            *counter += INTERRUPT_GPIO_HASHMAP.lock().unwrap()[current_pin as usize].load(Ordering::SeqCst);
                            INTERRUPT_GPIO_HASHMAP.lock().unwrap()[current_pin as usize].store(0, Ordering::SeqCst); // reset counter again
                            println!("Interrupt pin = {:?} current_pin: {}", pin_num, current_pin);
                            let _ = sensor.enable_interrupt();
                        };
                        let new_data = sensor.get_value().unwrap() as u8;
                        let data_change_pct = calculate_percentage_change(new_data as f32, *data as u8 as f32);
                        let counter_change_pct = calculate_percentage_change(*counter as f32, old_counter as f32);

                        let _ = handle_sensor_value_update(
                            &mut client,
                            new_data,
                            last_sent_value,
                            last_update,
                            data_change_pct,
                            delta,
                            delta_over.as_ref(),
                            now,
                            Duration::from_secs(MAX_PERIOD_REFRESH_MQTT),
                            &mqtt_messages::publish_sensor_topic(&account, &object, &dev, "0"),
                            "gpio-in",
                        );
                        *data = new_data;
                        let _ = handle_sensor_value_update(
                            &mut client,
                            *counter  as f64,
                            &mut last_sent_counter.map(|v| v as f64),
                            last_update,
                            counter_change_pct,
                            delta,
                            delta_over.as_ref(),
                            now,
                            Duration::from_secs(MAX_PERIOD_REFRESH_MQTT),
                            &mqtt_messages::publish_sensor_topic(&account, &object, &dev, "1"),
                            "gpio-in-counter",
                        );
                        let _ = update_sensor_data(&shared_data_ref, dev, vec![new_data as f32, *counter as f32]);
                    },
                    SsnDevices::GpioOutput { 
                        data, 
                        delta, 
                        delta_over, 
                        last_update, 
                        last_sent_value, 
                        sensor } => {
                            
                            if dev.eq(&command_dev_id) {
                                log::info!("set value StoreInt {}", command_value);
                                if command_value != 0.0 {
                                    sensor.set_high();
                                    *data = true;
                                }
                                else {
                                    sensor.set_low();
                                    *data = false;
                                };
                            };
                        },
                }
                // little pause between measurments
                FreeRtos::delay_ms(500); // Wait 500ms
            }
        }
    );
        // led.set_low().unwrap();
        println!("Measurment circle finished");

        // Set LED to green again
        status_led(&mut ws2812, Colors::Green);

        // reset time counters if needed:
        if now.duration_since(time_counter_seldom) > Duration::from_millis(SLOW_LOOP_PERIOD_MS) {
            time_counter_seldom = Instant::now();
        }
        if now.duration_since(time_counter_middle) > Duration::from_millis(MIDDLE_LOOP_PERIOD_MS) {
            time_counter_middle = Instant::now();
        }

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

// // Shared interrupt handler
// fn interrupt_handler_gpio(pin_num: u8) {
//     INTERRUPT_GPIO_MASK.fetch_or(1 << pin_num, Ordering::SeqCst);
// }

// extern "C" fn interrupt_handler_gpio_t() {
//     println!("interrupt_handler_gpio_t !!!");
//     INTERRUPT_GPIO_MASK.fetch_or(1 << 3, Ordering::SeqCst);
// }