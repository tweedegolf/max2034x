//! Device type definitions.
use num_enum::{IntoPrimitive, TryFromPrimitive};

#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Copy, Clone, Eq, PartialEq, IntoPrimitive, TryFromPrimitive)]
/// Device interrupt status and mask.
pub enum InterruptStatus {
    /// No interrupt.
    /// Output has not reached full power capability, and
    /// V<sub>IN</sub> high enough for full power operation
    None = 0x00,
    /// InUVLO interrupt.
    /// Power may be limited due to low V<sub>IN</sub>
    InUVLO = 0x01,
    /// OutGood interrupt.
    /// Output voltage is high enough to support full power capability.
    OutGood = 0x02,
    /// Both InUVLO and OutGood are active.
    Both = 0x03,
}

impl InterruptStatus {
    pub fn in_uvlo(&self) -> bool {
        use InterruptStatus::*;
        matches!(self, InUVLO | Both)
    }

    pub fn out_good(&self) -> bool {
        use InterruptStatus::*;
        matches!(self, OutGood | Both)
    }
}

impl core::ops::Not for InterruptStatus {
    type Output = Self;

    fn not(self) -> Self::Output {
        match Self::try_from_primitive(0x03 - self as u8) {
            Ok(v) => v,
            Err(_) => unreachable!(),
        }
    }
}

#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Copy, Clone, Eq, PartialEq, IntoPrimitive, TryFromPrimitive)]
/// Device Buck-boost mode
pub enum BuckBoostMode {
    /// Buck-boost mode.
    BuckBoost = 0x00,
    /// Buck-only mode. Can be used if V<sub>OUT</sub> > V<sub>IN</sub>,
    /// and uses less power.
    BuckOnly = 0x01,
}

#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Copy, Clone, Eq, PartialEq, IntoPrimitive, TryFromPrimitive)]
/// Switch over mode. Defines whether the device internals
/// are powered from V<sub>IN</sub> or V<sub>OUT</sub>
pub enum SwitchOverMode {
    Vout = 0x00,
    Vin = 0x01,
}

/// Buck-boost output voltage setting
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OutputVoltage {
    pub(crate) raw: u8,
}

impl OutputVoltage {
    /// Build OutputVoltage from number of millivolts.
    /// Value is clipped at bounds `2500 <= mV <= 5500`
    pub fn from_millivolts(millivolts: u16) -> Self {
        let millivolts = millivolts.max(2500).min(5500);
        let raw = ((millivolts - 2500) / 50) as u8;
        debug_assert!(raw < 0b111100);
        Self { raw }
    }

    /// Get the voltage in minivolts
    pub const fn millivolts(&self) -> u16 {
        debug_assert!(self.raw < 0b111100);
        let mv = (self.raw as u16) * 50 + 2500;
        debug_assert!(matches!(mv, 2500..=5500));
        mv
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CurrentLimit {
    milliamps: u16,
}

impl CurrentLimit {
    pub fn from_milliamps(milliamps: u16) -> Self {
        let milliamps = milliamps.min(750);
        Self { milliamps }
    }

    pub fn milliamps(&self) -> u16 {
        self.milliamps
    }

    pub(crate) fn raw(&self, inductor: Inductor) -> u8 {
        use Inductor::*;
        match inductor {
            L1uH => self.milliamps / 50,
            L2_2uH => self.milliamps / 25,
        }
        .min(0x0F) as u8
    }
}

/// Inductor that is being used with this device.
/// Dictates the value of BBstFETScale bit in
/// the BBstCfg1 register
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Inductor {
    /// 1 uH inductor
    L1uH,
    /// 2.2 uH inductor
    L2_2uH,
}

#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Copy, Clone, Eq, PartialEq, IntoPrimitive, TryFromPrimitive)]
pub enum FrequencyThreshold {
    Rising25kFalling6_125k = 0x00,
    Rising35kFalling8_25k = 0x01,
    Rising50kFalling12_5k = 0x10,
    Rising100kFalling25k = 0x11,
}
