use crate::{
    state::{Disabled, Enabled, InitializedState},
    Inductor,
};

pub trait DeviceVersion {
    /// State in which the device boots:
    type BootState: InitializedState;
    /// Chip ID reported by device
    const CHIP_ID: u8;
    /// I2C slave address
    const ADDR: u8;
    /// For which inductor value the device is
    /// configured by default.
    /// Derived from BBstFETScale default value.
    const DEFAULT_INDUCTOR_CONFIG: Inductor;
}

macro_rules! device_version {
    ($device:ident, $expected_chip_id:literal, $addr:literal, $boot_state:ident, $inductor:expr) => {
        pub struct $device;

        impl DeviceVersion for $device {
            type BootState = $boot_state;
            const CHIP_ID: u8 = $expected_chip_id;
            const ADDR: u8 = $addr;
            const DEFAULT_INDUCTOR_CONFIG: Inductor = $inductor;
        }
    };
}

device_version!(Max20343B, 0x02, 0x68, Disabled, Inductor::L2_2uH);
device_version!(Max20343E, 0x02, 0x68, Disabled, Inductor::L2_2uH);
device_version!(Max20343F, 0x02, 0x68, Enabled, Inductor::L1uH);
device_version!(Max20343G, 0x02, 0x68, Disabled, Inductor::L2_2uH);
device_version!(Max20343H, 0x02, 0x68, Disabled, Inductor::L1uH);
device_version!(Max20343I, 0x02, 0x68, Disabled, Inductor::L1uH);
device_version!(Max20343K, 0x02, 0x68, Disabled, Inductor::L2_2uH);
device_version!(Max20343J, 0x03, 0x68, Disabled, Inductor::L2_2uH);
device_version!(Max20343N, 0x03, 0x68, Enabled, Inductor::L1uH);
device_version!(Max20344E, 0x02, 0x6C, Disabled, Inductor::L2_2uH);
