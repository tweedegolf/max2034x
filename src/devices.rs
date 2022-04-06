pub trait DeviceVersion {
    const CHIP_ID: u8;
    const ADDR: u8;
}

macro_rules! device_version {
    ($device: ident, $expected_chip_id:literal, $addr:literal) => {
        pub struct $device;

        impl DeviceVersion for $device {
            const CHIP_ID: u8 = $expected_chip_id;
            const ADDR: u8 = $addr;
        }
    };
}

device_version!(Max20343B, 0x02, 0x68);
device_version!(Max20343E, 0x02, 0x68);
device_version!(Max20343F, 0x02, 0x68);
device_version!(Max20343G, 0x02, 0x68);
device_version!(Max20343H, 0x02, 0x68);
device_version!(Max20343I, 0x02, 0x68);
device_version!(Max20343K, 0x02, 0x68);
device_version!(Max20343J, 0x03, 0x68);
device_version!(Max20343N, 0x03, 0x68);
device_version!(Max20344E, 0x02, 0x6C);
