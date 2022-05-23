//! Error type

#[derive(Debug)]
/// Device error type
pub enum DeviceError<EBUS> {
    /// Bus error.
    Bus(EBUS),
    /// Device ID not as expected.
    BadDeviceId,
}

impl<EBUS> From<EBUS> for DeviceError<EBUS> {
    fn from(e_bus: EBUS) -> Self {
        Self::Bus(e_bus)
    }
}
