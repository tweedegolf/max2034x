#![no_std]

use embedded_hal::blocking::i2c::{Write, WriteRead};

#[derive(Debug)]
pub enum Error<EBUS> {
    Bus(EBUS),
    BadDeviceId,
}

impl<EBUS> From<EBUS> for Error<EBUS> {
    fn from(e_bus: EBUS) -> Self {
        Self::Bus(e_bus)
    }
}

pub type Result<T, EBUS> = core::result::Result<T, Error<EBUS>>;
pub struct Max2034x<I2C> {
    i2c: I2C,
}

impl<I2C, E> Max2034x<I2C>
where
    I2C: Write<Error = E> + WriteRead<Error = E>,
{
    pub fn new(i2c: I2C) -> Result<Self, E> {
        let mut max2034x = Self { i2c };
        if max2034x.get_device_id()? != 0x02 {
            return Err(Error::BadDeviceId);
        }
        Ok(max2034x)
    }

    pub fn get_device_id(&mut self) -> Result<u8, E> {
        let mut buffer = [0u8; 1];
        self.i2c.write_read(0x68, &[0x00], &mut buffer)?;
        Ok(buffer[0])
    }
}
