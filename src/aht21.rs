use anyhow::{Context, Result};
use embedded_hal::i2c::ErrorType;
use embedded_hal::i2c::I2c;
use std::fmt::Debug;
use std::ops::Deref;

use ahtx0::blocking::AHTx0 as Aht20;
use embedded_hal::i2c::Error as I2cError;
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::io::Read;
use esp_idf_hal::io::Write;
use esp_idf_hal::{gpio::PinDriver, prelude::Peripherals};
use esp_idf_sys as _;
use log::*;

pub struct AHT21Sensor<I2C>
where
    I2C: embedded_hal::i2c::I2c,
{
    sensor: Aht20<I2C>,
}

#[derive(Debug)]
pub enum AHT21Error<E: Debug> {
    I2CError(E),
    SensorError,
    DataNotReady,
}

impl<E: Debug> From<E> for AHT21Error<E> {
    fn from(err: E) -> Self {
        AHT21Error::I2CError(err)
    }
}

impl<E: Debug> std::fmt::Display for AHT21Error<E> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AHT21Error::I2CError(e) => write!(f, "I2C error: {:?}", e),
            AHT21Error::SensorError => write!(f, "Sensor error"),
            AHT21Error::DataNotReady => write!(f, "Sensor data not ready"),
        }
    }
}

impl<E> I2cError for AHT21Error<E>
where
    E: core::fmt::Debug + I2cError,
{
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match self {
            AHT21Error::I2CError(e) => e.kind(),
            _ => embedded_hal::i2c::ErrorKind::Other,
        }
    }
}

impl<I2C, E: I2cError> ErrorType for AHT21Sensor<I2C>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
{
    type Error = AHT21Error<E>;
}

impl<I2C, E> AHT21Sensor<I2C>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
    E: embedded_hal::i2c::Error + Debug,
{
    pub fn new(i2c: I2C) -> Result<Self, AHT21Error<E>> {
        log::info!("Initializing AHT21 sensor");
        // let mut sensor = Aht20::new(i2c).map_err(|e| AHT21Error::SensorError)?;
        let mut sensor = Aht20::new(i2c);
        FreeRtos::delay_ms(100); // Allow sensor to stabilize
        let _ = sensor.calibrate(&mut FreeRtos);
        FreeRtos::delay_ms(100);
        let device_id = sensor
            .state()
            .inspect_err(|e| log::error!("failed to read state: {:?}", e));
        log::info!("device state = {:?}", device_id);
        Ok(AHT21Sensor { sensor })
    }

    pub fn get_data(&mut self) -> Result<(f32, f32), AHT21Error<E>> {
        let measurement = self.sensor.measure(&mut FreeRtos).map_err(|e| {
            log::error!("Failed to measure AHT21 sensor: {:?}", e);
            AHT21Error::SensorError
        })?;
        Ok((
            measurement.temperature.as_degrees_celsius(),
            measurement.humidity.as_percent(),
        ))
    }
}
