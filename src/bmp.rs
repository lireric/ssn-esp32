use anyhow::{Context, Result};
// use bmp180_driver::error::CustomError;
// use bmp180_driver::{Common, InitializedBMP180, Resolution, BMP180};
use bmpe280::bme280::BME280;
use bmpe280::bmp280::BMP280;

use embedded_hal::i2c::Error as I2cError;
// use esp_idf_hal::delay::FreeRtos;
use esp_idf_svc::hal::{
    delay,
    i2c::{I2cConfig, I2cDriver},
};
use log::*;
// use std::error::Error;
// use std::fmt::Debug;
// use std::fmt::Display;

// different types Bosh sensors
pub enum SensorType {
    BMP280,
    BME280,
}

/// Custom error type for BMP180 operations
// #[derive(Debug)]
// pub enum BmpError {
//     /// Error from the BMP driver
//     // DriverError(CustomError<E>),
//     /// Other errors (e.g., I2C errors)
//     Other(String),
// }
// impl std::fmt::Display for BmpError {
//     fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
//         match self {
//             BmpError::DriverError(e) => write!(f, "BMP Driver Error: {:?}", e),
//             BmpError::Other(msg) => write!(f, "Other Error: {}", msg),
//         }
//     }
// }
// impl Error for BmpError {}
// impl<E: Debug> From<CustomError<E>> for BmpError<E> {
//     fn from(error: CustomError<E>) -> Self {
//         BmpError::DriverError(error)
//     }
// }
// impl I2cError for BmpError {
//     fn kind(&self) -> embedded_hal::i2c::ErrorKind {
//         embedded_hal::i2c::ErrorKind::Other
//     }
// }
pub struct BMPSensor<I2C, D>
where
    I2C: embedded_hal::i2c::I2c,
    D: embedded_hal::delay::DelayNs,
{
    sensor_type: SensorType,
    bmp280: Option<BMP280<I2C>>,
    bme280: Option<BME280<I2C, D>>,
}

impl<I2C, D> BMPSensor<I2C, D>
where
    I2C: embedded_hal::i2c::I2c,
    D: embedded_hal::delay::DelayNs,
{
    pub fn new(i2c: I2C, delay: D, sensor_type: SensorType) -> Result<Self> {
        match sensor_type {
            SensorType::BMP280 => {
                log::info!("Initializing BMP280 sensor");
                let bmp280 = BMP280::new(i2c);
                Ok(Self {
                    sensor_type,
                    bmp280: Some(bmp280),
                    bme280: None,
                })
            }
            SensorType::BME280 => {
                log::info!("Initializing BME280 sensor");
                let bme280 = BME280::new(i2c, delay);
                Ok(Self {
                    sensor_type,
                    bmp280: None,
                    bme280: Some(bme280),
                })
            }
        }
    }

    pub fn get_data(&mut self) -> Result<(f64, f64, Option<f64>)> {
        match self.sensor_type {
            SensorType::BMP280 => {
                let m = self.bmp280.as_mut().unwrap().measure();
                Ok((m.temperature, m.pressure, None))
            }
            SensorType::BME280 => {
                let m = self.bme280.as_mut().unwrap().measure();
                Ok((m.temperature, m.pressure, Some(m.humidity)))
            }
        }
    }
}
