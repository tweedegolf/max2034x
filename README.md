# MAX2034x
Driver crate for the MAX20343/MAX20344 family of buck/boost converters.
Based on [`device_driver`](https://docs.rs/device-driver).  

*Documentation cites from and refers to the datasheet, which can be found [here](https://datasheets.maximintegrated.com/en/ds/MAX20343-MAX20344.pdf).*

Uses [typestate](https://docs.rust-embedded.org/book/static-guarantees/typestate-programming.html) to enforce correct usage.

## Example use
```rust,ignore
use max2034x::devices::Max20343F;
use max2034x::Inductor;
use max2034x::Pins;

let i2c = ...;
let pins = Pins { ... };
// Create high-level interface, passing the inductor used with the device and
// the optional pins.
let mut buck_boost =
    max2034x::Max2034x::new(i2c.acquire(), Max20343F, pins, Inductor::L2_2uH)
        .unwrap();

buck_boost.enable_fast_boost(true).unwrap();
buck_boost.enable_fast_boost_pin(true).unwrap();
```

## EH 1.0 support

By default this crate uses embedded-hal 0.2, but that can changed to (the alpha of) embedded-hal 1.0 with a feature.
Just disable the deafault features and enable the `eh-1` feature. (They cannot be active at the same time)

Because EH-1.0 is still unstable, it falls outside of the semver guarantees.
