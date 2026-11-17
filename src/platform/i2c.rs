//! Shared I2C bus wrapper used by every sensor adapter.

use embedded_hal::i2c::{ErrorType, I2c};
use esp_idf_hal::i2c::{I2cDriver, I2cError, Operation};
use std::sync::{Arc, Mutex};

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
        let mut guard = crate::platform::lock_recover(&self.0);
        Ok(guard.read(addr, buffer, 1000)?)
    }

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        let mut guard = crate::platform::lock_recover(&self.0);
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
        let mut guard = crate::platform::lock_recover(&self.0);
        Ok(guard.transaction(address, operations, 1000)?)
    }
}
