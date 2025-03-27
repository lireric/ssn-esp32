use anyhow::{Context, Result};
use bmp180_driver::error::CustomError;
use bmp180_driver::{Common, InitializedBMP180, Resolution, BMP180};
use embedded_hal::i2c::Error as I2cError;
use esp_idf_hal::delay::FreeRtos; // For delay functionality
use esp_idf_svc::hal::{
    delay,
    i2c::{I2cConfig, I2cDriver},
};
use log::*;
use std::error::Error;
use std::fmt::Debug;
use std::fmt::Display;

/// Custom error type for BMP180 operations
#[derive(Debug)]
pub enum Bmp180Error<E: Debug> {
    /// Error from the BMP180 driver
    DriverError(CustomError<E>),
    /// Other errors (e.g., I2C errors)
    Other(String),
}

impl<E: Debug> std::fmt::Display for Bmp180Error<E> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Bmp180Error::DriverError(e) => write!(f, "BMP180 Driver Error: {:?}", e),
            Bmp180Error::Other(msg) => write!(f, "Other Error: {}", msg),
        }
    }
}

impl<E: Debug> Error for Bmp180Error<E> {}

impl<E: Debug> From<CustomError<E>> for Bmp180Error<E> {
    fn from(error: CustomError<E>) -> Self {
        Bmp180Error::DriverError(error)
    }
}

impl<E: Debug> I2cError for Bmp180Error<E> {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        embedded_hal::i2c::ErrorKind::Other
    }
}

pub struct BMP180Sensor<I2C, Delay> {
    sensor: InitializedBMP180<I2C, Delay>,
}

impl<I2C, Delay> BMP180Sensor<I2C, Delay>
where
    <I2C as embedded_hal::i2c::ErrorType>::Error: 'static,
    I2C: embedded_hal::i2c::I2c, // + embedded_hal::i2c::Error,
    Delay: embedded_hal::delay::DelayNs,
    // E: Debug + Display + Send + Sync + 'static,
{
    pub fn new(i2c: I2C, delay: Delay) -> Result<Self, Bmp180Error<I2C::Error>> {
        // pub fn new(i2c: I2C, delay: Delay) -> Result<Self, Box<dyn Error>> {
        let mut sensor = BMP180::new(i2c, delay);

        log::info!("Check connection to the sensor BMP180");
        sensor.check_connection()?;

        log::info!("Initialize the sensor BMP180");
        let sensor = sensor.initialize()?;

        Ok(BMP180Sensor { sensor })
    }

    pub fn get_data(&mut self) -> Result<(f32, i32, f32), Bmp180Error<I2C::Error>> {
        let (temperature, pressure, altitude) =
            self.sensor.read_all(Resolution::UltraHighResolution)?;
        Ok((temperature, pressure, altitude))
    }
}
