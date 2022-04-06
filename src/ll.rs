use crate::devices::DeviceVersion;
use crate::error::DeviceError;
use core::fmt::Debug;
use core::iter::once;
use core::marker::PhantomData;
use device_driver::ll::register::RegisterInterface;
use device_driver::{create_low_level_device, implement_registers};
use embedded_hal::blocking::i2c::{WriteIter, WriteIterRead};

pub struct Max2034xInterface<V, I2C> {
    i2c: I2C,
    _marker: PhantomData<V>,
}

impl<V, I2C, EBUS> Max2034xInterface<V, I2C>
where
    V: DeviceVersion,
    I2C: WriteIter<Error = EBUS> + WriteIterRead<Error = EBUS>,
{
    pub fn new(i2c: I2C, version: V) -> Self {
        let _ = version;
        Self {
            i2c,
            _marker: PhantomData,
        }
    }

    pub fn free(self) -> I2C {
        self.i2c
    }
}

impl<V, I2C, EBUS> RegisterInterface for Max2034xInterface<V, I2C>
where
    V: DeviceVersion,
    I2C: WriteIter<Error = EBUS> + WriteIterRead<Error = EBUS>,
    EBUS: Debug,
{
    type Address = u8;
    type InterfaceError = DeviceError<EBUS>;

    fn read_register(
        &mut self,
        address: Self::Address,
        value: &mut [u8],
    ) -> Result<(), DeviceError<EBUS>> {
        self.i2c.write_iter_read(V::ADDR, once(address), value)?;
        Ok(())
    }

    fn write_register(
        &mut self,
        address: Self::Address,
        value: &[u8],
    ) -> Result<(), DeviceError<EBUS>> {
        self.i2c
            .write(V::ADDR, once(address).chain(value.iter().copied()))?;
        Ok(())
    }
}

impl<V, I2C, EBUS> HardwareInterface for Max2034xInterface<V, I2C>
where
    V: DeviceVersion,
    I2C: WriteIter<Error = EBUS> + WriteIterRead<Error = EBUS>,
    EBUS: Debug,
{
    const CHIP_ID: u8 = V::CHIP_ID;
}

create_low_level_device!(Max2034xLL {
    errors: [],
    hardware_interface_requirements: {RegisterInterface<Address = u8>},
    hardware_interface_capabilities: {
        const CHIP_ID: u8;
    },
});

implement_registers!(Max2034xLL.registers<u8> = {
    chip_id(RO, 0x00, 1) = {
        id: u8 = RO 0..=7,
    },
});
