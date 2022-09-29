//! Low-level interface

use crate::{
    devices::DeviceVersion, error::DeviceError, state::InitializedState, BuckBoostMode,
    FrequencyThreshold, Inductor, InterruptStatus, SwitchOverMode,
};
use core::{fmt::Debug, marker::PhantomData};
use device_driver::{
    create_low_level_device, implement_registers, ll::register::RegisterInterface, Bit,
};

#[cfg(feature = "eh-02")]
use embedded_hal02::blocking::i2c::{Write, WriteRead};
#[cfg(feature = "eh-1")]
use embedded_hal1::i2c::I2c;

/// Interface to the device
pub struct Max2034xInterface<V, I2C> {
    i2c: I2C,
    _marker: PhantomData<V>,
}

impl<V, I2C> Max2034xInterface<V, I2C>
where
    V: DeviceVersion,
{
    /// Create a new interface to the device
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

#[cfg(feature = "eh-02")]
impl<V, I2C, EBUS> RegisterInterface for Max2034xInterface<V, I2C>
where
    V: DeviceVersion,
    I2C: Write<Error = EBUS> + WriteRead<Error = EBUS>,
    EBUS: Debug,
{
    type Address = u8;
    type InterfaceError = DeviceError<EBUS>;

    fn read_register(
        &mut self,
        address: Self::Address,
        value: &mut [u8],
    ) -> Result<(), DeviceError<EBUS>> {
        self.i2c.write_read(V::ADDR, &[address], value)?;
        Ok(())
    }

    fn write_register(
        &mut self,
        address: Self::Address,
        value: &[u8],
    ) -> Result<(), DeviceError<EBUS>> {
        // All registers are 1 byte, so value is only ever 1 byte long
        debug_assert_eq!(value.len(), 1);
        self.i2c
            .write(V::ADDR, &[address, value[0]])?;
        Ok(())
    }
}

#[cfg(feature = "eh-02")]
impl<V, I2C, EBUS> HardwareInterface for Max2034xInterface<V, I2C>
where
    V: DeviceVersion,
    I2C: Write<Error = EBUS> + WriteRead<Error = EBUS>,
    EBUS: Debug,
{
    type BootState = V::BootState;
    const CHIP_ID: u8 = V::CHIP_ID;
    const DEFAULT_INDUCTOR_CONFIG: Inductor = V::DEFAULT_INDUCTOR_CONFIG;
}

#[cfg(feature = "eh-1")]
impl<V, I2C, EBUS> RegisterInterface for Max2034xInterface<V, I2C>
where
    V: DeviceVersion,
    I2C: I2c<Error = EBUS>,
    EBUS: Debug,
{
    type Address = u8;
    type InterfaceError = DeviceError<EBUS>;

    fn read_register(
        &mut self,
        address: Self::Address,
        value: &mut [u8],
    ) -> Result<(), DeviceError<EBUS>> {
        self.i2c.write_read(V::ADDR, &[address], value)?;
        Ok(())
    }

    fn write_register(
        &mut self,
        address: Self::Address,
        value: &[u8],
    ) -> Result<(), DeviceError<EBUS>> {
        // All registers are 1 byte, so value is only ever 1 byte long
        debug_assert_eq!(value.len(), 1);
        self.i2c
            .write(V::ADDR, &[address, value[0]])?;
        Ok(())
    }
}

#[cfg(feature = "eh-1")]
impl<V, I2C, EBUS> HardwareInterface for Max2034xInterface<V, I2C>
where
    V: DeviceVersion,
    I2C: I2c<Error = EBUS>,
    EBUS: Debug,
{
    type BootState = V::BootState;
    const CHIP_ID: u8 = V::CHIP_ID;
    const DEFAULT_INDUCTOR_CONFIG: Inductor = V::DEFAULT_INDUCTOR_CONFIG;
}

create_low_level_device!(
#[doc = "Low-level device, used to directly read and modify register values."]
Max2034xLL {
    errors: [],
    hardware_interface_requirements: {RegisterInterface<Address = u8>},
    hardware_interface_capabilities: {
        type BootState: InitializedState;
        const CHIP_ID: u8;
        const DEFAULT_INDUCTOR_CONFIG: Inductor;
    },
});

implement_registers!(
#[doc = "Register definitions"]
Max2034xLL.registers<u8> = {
    #[doc = "ChipID (0x00)"]
    chip_id(RO, 0x00, 1) = {
        /// Indicates the version of the device in use
        id: u8 = RO 0..=7,
    },
    #[doc = "BBstCfg0 (0x01)"]
    b_bst_cfg0(RW, 0x01, 1) = {
        /// Buck-boost enable
        /// - 0 = Buck-boost disabled
        /// - 1 = Buck-boost enabled
        b_bst_en: u8 as Bit = RW 7..=7,
        /// Buck-boost ramp enable
        /// - 0 = Output voltage setting transition is performed without intermediate steps
        /// - 1 = Output voltage setting increases are performed with a digital ramp of
        /// 50mV every 50μs
        b_bst_ramp_en: u8 as Bit = RW 6..=6,
        /// Buck-Boost Pretrigger Mode Setting
        /// Increases the quiescent current of the buck-boost to improve output
        /// regulation during load transients.
        /// - 0 = Normal, low quiescent current operation
        /// - 1 = Fast response mode enabled. Quiescent current increased to 35μA (typ).
        b_bst_fast: u8 as Bit = RW 5..=5,
        /// Buck-Boost Zero-Crossing Comparator Disable.
        /// Latched internally, it can only
        /// be changed when BBstEn = 0
        /// - 0 = Enabled
        /// - 1 = Disabled
        b_bst_zc_cmp_dis: u8 as Bit = RW 4..=4,
        /// Buck-Boost Low EMI Mode
        /// Increases the rise/fall time of HVLX/LVLX to reduce EMI, at the cost of
        /// efficiency.
        /// - 0 = Normal operation
        /// - 1 = Increase rise/fall time on HVLX/LVLX by 3x
        b_bst_low_emi: u8 as Bit = RW 3..=3,
        /// Buck-Boost Operating Mode
        /// Configures the regulator to operate in buck-boost or buck-only mode. Latched
        /// internally, can only be changed while BBstEn = 0.
        /// - 0 = Buck-boost mode
        /// - 1 = Buck-only mode
        b_bst_mode: u8 as BuckBoostMode = RW 2..=2,
        /// Buck-Boost Active Discharge Control
        /// - 0 = Buck-boost not actively discharged
        /// - 1 = Buck-boost actively discharged on shutdown
        b_bst_act_dsc: u8 as Bit = RW 1..=1,
        /// Buck-Boost Passive Discharge Control
        /// - 0 = Buck-boost not passively discharged
        /// - 1 = Buck-boost passively discharged on shutdown
        b_bst_psv_dsc: u8 as Bit = RW 0..=0,
    },
    #[doc = "BBstVSet (0x02)"]
    b_bst_v_set(RW, 0x02, 1) = {
        /// Buck-Boost f<sub>HIGH</sub> Thresholds
        /// Selects the switching frequency threshold f<sub>HIGH</sub>. If the buck-boost switching
        /// frequency exceeds the f<sub>HIGH</sub> rising threshold, all the blocks are kept ON (I<sub>Q</sub> is
        /// higher) until the frequency reaches the f<sub>HIGH</sub> falling threshold. A small glitch
        /// on V<sub>OUT</sub> can be present at the f<sub>HIGH</sub> crossover.
        /// - 00 = 25kHz rising / 6.125kHz falling
        /// - 01 = 35kHz rising / 8.25kHz falling
        /// - 10 = 50kHz rising / 12.5kHz falling
        /// - 11 = 100kHz rising / 25kHz falling
        b_bst_fhigh_sh: u8 as FrequencyThreshold = RW 6..=7,
        /// Buck-Boost Output Voltage Setting
        /// 2.5V to 5.5V, Linear Scale, 50mV increments
        /// - 000000 = 2.5V
        /// - 000001 = 2.55V
        /// - ...
        /// - ≥111100 = 5.5V
        b_bst_v_set: u8 = RW 0..=5,
    },
    #[doc = "BBstISet (0x03)"]
    b_bst_i_set(RW, 0x03, 1) = {
        /// Buck-Boost Nominal Maximum Peak Current Setting
        /// See buck-boost operation section for a description of the peak current
        /// settings. 0mA (minimum t<sub>ON</sub>) to 618.75mA, linear scale, 41.25mA increments
        /// for BBstFETScale = 0. 0mA (minimum t<sub>ON</sub>) to 375mA, linear scale, 25mA
        /// increments for BBstFETScale = 1.
        ///
        /// **BBstFETScale = 0:**
        /// - 0000 = 0mA (minimum t<sub>ON</sub>)
        /// - 0001 = 50mA
        /// - ...
        /// - 1111 = 750mA
        ///
        /// Recommended settings
        /// - V<sub>OUT</sub> ≤ 2.65V: 500mA(1010)
        /// - 2.65V < V<sub>OUT</sub> ≤ 3.05V: 450mA(1001)
        /// - 3.05V < V<sub>OUT</sub> ≤ 3.60V: 400mA(1000)
        /// - 3.60V < V<sub>OUT</sub> ≤ 4.35V: 350mA(0111)
        /// - V<sub>OUT</sub> > 4.35V: 300mA(0110)
        ///
        /// **BBstFETScale = 1:**
        ///
        /// - 0000 = 0mA (minimum t<sub>ON</sub>)
        /// - 0001 = 25mA
        /// - ...
        /// - 1111 = 375mA
        ///
        /// Recommended settings
        ///
        /// - V<sub>OUT</sub> ≤ 2.65V: 250mA(1010)
        /// - 2.65V < V<sub>OUT</sub> ≤ 3.05V: 225mA(1001)
        /// - 3.05V < V<sub>OUT</sub> ≤ 3.60V: 200mA(1000)
        /// - 3.60V < V<sub>OUT</sub> ≤ 4.35V: 175mA(0111)
        /// - V<sub>OUT</sub> > 4.35V: 150mA(0110)
        b_bst_ip_set2: u8 = RW 4..=7,
        /// Buck-boost nominal peak current setting 1
        /// Nominal peak current when charging inductor between V<sub>IN</sub> and GND. See
        /// buck-boost operation section for a description of the peak current settings.
        /// 0mA (minimum t<sub>ON</sub>) to 618.75mA, linear scale, 41.25mA increments for
        /// BBstFETScale = 0. 0mA (minimum t<sub>ON</sub>) to 375mA, linear scale, 25mA
        /// increments for BBstFETScale = 1.
        ///
        /// **BBstFETScale = 0:**
        ///
        /// - 0000 = 0mA (minimum t<sub>ON</sub>)
        /// - 0001 = 50mA
        /// - ...
        /// - 1111 = 750mA
        ///
        /// Recommended settings
        ///
        /// - V<sub>OUT</sub> ≤ 3.40V: 200mA
        /// - 3.40V < V<sub>OUT</sub> ≤ 3.80V: 250mA(0101)
        /// - 3.80V < V<sub>OUT</sub> ≤ 4.15V: 300mA(0110)
        /// - 4.15V < V<sub>OUT</sub> ≤ 4.55V: 350mA(0111)
        /// - 4.55V < V<sub>OUT</sub> ≤ 4.90V: 400mA(1000)
        /// - 4.90V < V<sub>OUT</sub> ≤ 5.30V: 450mA(1001)
        /// - V<sub>OUT</sub> > 5.30V: 500mA(1010)
        ///
        /// **BBstFETScale = 1:**
        ///
        /// - 0000 = 0mA (minimum t<sub>ON</sub>)
        /// - 0001 = 25mA
        /// - ...
        /// - 1111 = 375mA
        ///
        /// Recommended settings
        ///
        /// - V<sub>OUT</sub> ≤ 3.40V: 100mA(0100)
        /// - 3.40V < V<sub>OUT</sub> ≤ 3.80V: 125mA(0101)
        /// - 3.80V < V<sub>OUT</sub> ≤ 4.15V: 150mA(0110)
        /// - 4.20V < V<sub>OUT</sub> ≤ 4.55V: 175mA(0111)
        /// - 4.60V < V<sub>OUT</sub> ≤ 4.90V: 200mA(1000)
        /// - 4.95V < V<sub>OUT</sub> ≤ 5.30V: 225mA(1001)
        /// - V<sub>OUT</sub> > 5.30V: 250mA(1010)
        b_bst_ip_set1: u8 = RW 0..=3,
    },
    #[doc = "BBstCfg1 (0x04)"]
    b_bst_cfg1(RW, 0x04, 1) = {
        /// FAST Comparator Enable
        /// The FAST mode comparator is enabled by the logical AND of the FAST pin
        /// and FstCmpEn.
        ///
        /// - 0 = FAST pin does not control FAST mode
        /// - 1 = FAST pin can set the device into FAST mode
        fst_cmp_en: u8 as Bit = RW 7..=7,
        /// Pass Through Mode
        /// Bypasses the regulator to connect V<sub>OUT</sub> to V<sub>IN</sub>. This can only be enabled
        /// when BBstEn = 0 (register 0x01).
        ///
        /// - 0 = Pass Through Mode disabled
        /// - 1 = Pass Through Mode enabled. Enable only when BBstEn = 0.
        pas_thr_mode: u8 as Bit = RW 6..=6,
        /// Force Switch-Over
        /// Controls how the device powers the internal circuitry.
        ///
        /// - 0 = Switch-over supply forced to V<sub>OUT</sub> when V<sub>OUT</sub> > V<sub>OUT_UVLO_R</sub>.
        /// - 1 = Switch-over supply forced to V<sub>IN</sub>.
        swo_frc_in: u8 as SwitchOverMode = RW 5..=5,
        /// Buck-Boost Integrator Enable
        /// The Integrator can be disabled to improve settling time on load transients at
        /// the cost of load regulation error. Latched internally, it can only be changed
        /// when BBstEn = 0.
        ///
        /// - 0 = Integrator disabled
        /// - 1 = Integrator enabled
        b_bst_integ_en: u8 as Bit = RW 2..=2,
        /// Adaptive Peak/Valley Current Adjustment Disable
        ///
        /// - 0 = Enabled
        /// - 1 = Disabled, peak current fixed to the values set by BBstIPSet1 and
        /// BBstIPSet2.
        ///
        /// Valley current is fixed to 0mA. This setting is equivalent to forcing
        /// discontinuous conduction mode and greatly diminishes the output power
        /// capability of the part. Generally this is not a recommended setting.
        b_bst_ip_adpt_dis: u8 as Bit = RW 1..=1,
        /// FET Scale
        /// Reduces FET sizes by a factor of 2. This setting can be used to optimize
        /// efficiency for
        /// lighter loads if it is acceptable to support lower maximum output power.
        /// If BBstFETScale = 0, the part requires a 1μH inductor and at least twice the
        /// derated capacitance in Figure 9. If BBstFETScale = 1, the part requires a
        /// 2.2μH inductor and at least the derated capacitance in Figure 9. Latched
        /// internally, it can only be changed when BBstEn = 0.
        ///
        /// - 0 = FET scaling disabled
        /// - 1 = FET scaling enabled
        b_bst_fet_scale: u8 as Bit = RW 0..=0,
    },
    #[doc = "Status (0x05)"]
    status(RO, 0x05, 1) = {
        /// Device status
        status: u8 as InterruptStatus = RO 0..=1,
    },
    #[doc = "Int (0x06)"]
    int(RO, 0x06, 1) = {
        /// Interrupt status, can be read to
        /// find out what caused an IRQ.
        int: u8 as InterruptStatus = RO 0..=1,
    },
    #[doc = "Mask (0x07)"]
    mask(RW, 0x07, 1) = {
        /// Interrupt mask
        mask: u8 as InterruptStatus = RW 0..=1,
    },
    #[doc = "LockMsk (0x50)"]
    lock_msk(RW, 0x50, 1) = {
        /// Lock Mask for Buck-Boost Registers
        ///
        /// - 0 = Buck-Boost Registers not masked from locking/unlocking
        /// - 1 = Buck-Boost Registers masked from locking/unlocking
        b_bst_lck: u8 as Bit = RW 0..=0,
    },
    #[doc = "LockUnlock (0x51)"]
    lock_unlock(RW, 0x51, 1) = {
        /// Lock/Unlock Password
        ///
        /// - Write 0xAA with BBLck unmasked to lock the BBstVSet[5:0] field
        /// - Write 0x55 with BBLck unmasked to unlock the BBstVSet[5:0] field
        passwd: u8 = RW 0..=7,
    }

});
