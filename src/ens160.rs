use anyhow::{Context, Result};
use embedded_hal::i2c::ErrorType;
use embedded_hal::i2c::I2c;
use std::fmt::Debug;
use std::ops::Deref;

use embedded_hal::i2c::Error as I2cError;
use ens160::AirQualityIndex;
use ens160::ECo2;
use ens160::Ens160;
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::io::Read;
use esp_idf_hal::io::Write;
use esp_idf_hal::{gpio::PinDriver, prelude::Peripherals};
use esp_idf_sys as _;
use log::*;

pub struct ENS160Sensor<I2C> {
    sensor: Ens160<I2C>,
}
#[derive(Debug)]
pub enum Ens160Error<E: Debug> {
    I2CError(E),
    SensorError,
    DataNotReady,
}

impl<E: Debug> From<E> for Ens160Error<E> {
    fn from(err: E) -> Self {
        Ens160Error::I2CError(err)
    }
}

impl<E: Debug> std::fmt::Display for Ens160Error<E> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Ens160Error::I2CError(e) => write!(f, "I2C error: {:?}", e),
            Ens160Error::SensorError => write!(f, "Sensor error"),
            Ens160Error::DataNotReady => write!(f, "Sensor data not ready"),
        }
    }
}

impl<E> I2cError for Ens160Error<E>
where
    E: core::fmt::Debug + I2cError, // Both Debug and I2cError required
{
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match self {
            Ens160Error::I2CError(e) => e.kind(),
            _ => embedded_hal::i2c::ErrorKind::Other,
        }
    }
}

impl<I2C, E: I2cError> ErrorType for ENS160Sensor<I2C>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
{
    type Error = Ens160Error<E>;
}

//the traits `embedded_hal::i2c::ErrorType` and `embedded_hal::i2c::I2c` must be implemented

impl<I2C, E> ENS160Sensor<I2C>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
    E: embedded_hal::i2c::Error + Debug, // E must implement I2c Error and Debug
{
    pub fn new(i2c: I2C) -> Result<Self, Ens160Error<E>> {
        log::info!("initialize device Ens160");
        let mut sensor = Ens160::new(i2c, 0x53);
        let (a, b, c) = sensor.firmware_version()?;
        log::info!("firmware version of the sensor: {}.{}.{}", a, b, c);
        sensor.reset()?;
        FreeRtos::delay_ms(250);
        log::info!("go to operational mode sensor Ens160");
        sensor.operational()?;
        Ok(ENS160Sensor { sensor })
    }

    pub fn get_data(&mut self) -> Result<(i16, i16, i8), Ens160Error<E>> {
        let status = self.sensor.status()?;
        let mut co2eq_ppm: i16 = -1;
        let mut tvoc_ppb: i16 = -1;
        let mut aqi: i8 = -1;

        if !status.data_is_ready() {
            return Err(Ens160Error::DataNotReady);
        }

        tvoc_ppb = match self.sensor.tvoc() {
            Ok(v) => v as i16,
            Err(e) => return Err(e.into()),
        };

        co2eq_ppm = match self.sensor.eco2() {
            Ok(v) => *v.deref() as i16,
            Err(e) => return Err(e.into()),
        };

        aqi = match self.sensor.air_quality_index() {
            Ok(v) => v as i8,
            Err(e) => return Err(e.into()),
        };

        Ok((co2eq_ppm, tvoc_ppb, aqi))
    }
}
