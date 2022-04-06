#[derive(Debug)]
pub enum DeviceError<EBUS> {
    Bus(EBUS),
    BadDeviceId,
}

impl<EBUS> From<EBUS> for DeviceError<EBUS> {
    fn from(e_bus: EBUS) -> Self {
        Self::Bus(e_bus)
    }
}

// pub type Result<T, EBUS> = core::result::Result<T, DeviceError<EBUS>>;
