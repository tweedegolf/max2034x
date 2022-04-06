#![no_std]

use core::marker::PhantomData;

use device_driver::ll::{register::RegisterInterface, LowLevelDevice};
use error::DeviceError;
use ll::Max2034xLL;
use state::{Initialized, State, Uninitialized};

pub mod devices;
pub mod error;
pub mod ll;

pub struct Max2034x<I: ll::HardwareInterface, S: State> {
    ll: Max2034xLL<I>,
    _marker: PhantomData<S>,
}

impl<I: ll::HardwareInterface> Max2034x<I, Uninitialized> {
    pub fn new(interface: I) -> Self {
        Self {
            ll: Max2034xLL::new(interface),
            _marker: PhantomData,
        }
    }

    pub fn init(
        mut self,
    ) -> Result<Max2034x<I, Initialized>, DeviceError<<I as RegisterInterface>::InterfaceError>>
    {
        let chip_id = self.ll.registers().chip_id().read()?.id();
        if chip_id != I::CHIP_ID {
            return Err(DeviceError::BadDeviceId);
        }
        Ok(Max2034x {
            ll: self.ll,
            _marker: PhantomData,
        })
    }
}

pub mod state {
    pub trait State {}

    macro_rules! state {
        ($state:ident) => {
            pub struct $state;

            impl State for $state {}
        };
    }

    state!(Uninitialized);
    state!(Initialized);
}
