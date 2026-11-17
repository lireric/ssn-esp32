use ahtx0::blocking::AHTx0 as Aht20;
use anyhow::Result;
use embedded_hal::i2c::{Error as I2cError, ErrorType, I2c};
use esp_idf_hal::delay::FreeRtos;
use esp_idf_sys as _;
use log::*;

pub struct AHT21Sensor<I2C: I2c> {
    sensor: Aht20<I2C>,
}

#[derive(Debug)]
pub enum AHT21Error<E>
where
    E: std::fmt::Debug,
{
    I2CError(E),
    SensorError,
    DataNotReady,
}

impl<E> From<E> for AHT21Error<E>
where
    E: std::fmt::Debug,
{
    fn from(err: E) -> Self {
        AHT21Error::I2CError(err)
    }
}
impl<E> std::fmt::Display for AHT21Error<E>
where
    E: std::fmt::Debug,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AHT21Error::I2CError(e) => write!(f, "I2C error: {:?}", e),
            AHT21Error::SensorError => write!(f, "Sensor error"),
            AHT21Error::DataNotReady => write!(f, "Sensor data not ready"),
        }
    }
}

impl<E: I2cError> I2cError for AHT21Error<E> {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match self {
            AHT21Error::I2CError(e) => e.kind(),
            _ => embedded_hal::i2c::ErrorKind::Other,
        }
    }
}

impl<I2C: I2c> ErrorType for AHT21Sensor<I2C> {
    type Error = AHT21Error<I2C::Error>;
}

impl<I2C: I2c> AHT21Sensor<I2C>
where
    I2C::Error: std::fmt::Debug,
{
    pub fn new(i2c: I2C) -> Result<Self, ahtx0::Error<I2C::Error>> {
        info!("Initializing AHT21 sensor");
        let mut sensor = Aht20::new(i2c);
        FreeRtos::delay_ms(100);
        sensor.calibrate(&mut FreeRtos)?;
        FreeRtos::delay_ms(100);
        info!("device state = {:?}", sensor.state()?);
        Ok(Self { sensor })
    }

    pub fn get_data(&mut self) -> Result<(f32, f32), AHT21Error<I2C::Error>> {
        let measurement = self.sensor.measure(&mut FreeRtos).map_err(|e| {
            error!("Failed to measure AHT21 sensor: {:?}", e);
            AHT21Error::SensorError
        })?;
        Ok((
            measurement.temperature.as_degrees_celsius(),
            measurement.humidity.as_percent(),
        ))
    }
}
