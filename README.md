# `LTR-303ALS`


[![Build status](https://github.com/Ardelean-Calin/ltr303-rs/actions/workflows/main.yml/badge.svg)](https://github.com/Ardelean-Calin/ltr303-rs/actions/workflows/main.yml)
[![Crates.io Version][crates-io-badge]][crates-io]
[![Crates.io Downloads][crates-io-download-badge]][crates-io-download]
![No Std][no-std-badge]


This is a platform-agnostic Rust driver for the [`LTR-303 Ambient Light Sensor`](https://optoelectronics.liteon.com/en-global/Led/LED-Component/Detail/926/0/0/16/200) using [`embedded-hal`](https://github.com/rust-embedded/embedded-hal) traits.

## Supported devices
Tested with the following sensor(s):
- [LTR-303ALS-01](https://www.mouser.com/datasheet/2/239/Lite-On_LTR-303ALS-01_DS_ver%201.1-1175269.pdf)
  
## Status

- [x] Starting a measurement with configurable *gain*, *integration time* and *measurement rate*. See: `start_measurement()`
- [x] Polling for new data. See: `data_ready()`
- [x] Checking latest measurement status. See: `get_status()`
- [x] Reading the latest illuminance value in **lux**. See: `get_lux_data()`
- [x] Putting the sensor in standby. See: `standby()`
- [x] Reading part ID and manufacturer ID. See: `get_part_id()` and `get_mfc_id()`
- [ ] Option to pass a delay function to the driver. Similar to the [opt300x driver](https://github.com/eldruin/opt300x-rs).
- [ ] Sensor reset in case of error.
- [ ] Wait for sensor start-up before triggering measurement after cold startup (100ms) 
- [ ] Interrupts.


## Examples
On Linux using i2cdev:
```rust
use linux_embedded_hal::I2cdev;
use ltr303::{LTR303, LTR303Config};

fn main() {
    let dev = I2cdev::new("/dev/i2c-1").unwrap();
    let mut sensor = LTR303::init(dev);
    let config = LTR303Config::default();
    sensor.start_measurement(&config).unwrap();

    loop {
        while sensor.data_ready().unwrap() != true {
            // Wait for measurement ready
        }

        let lux_val = sensor.get_lux_data().unwrap();
        println!("LTR303 current lux phys: {}", lux_val.lux_phys);
    }
}
```
---

On an ESP32-based development board:

```rust
use embedded_hal::prelude::*;
use esp_idf_sys as _;
use esp_idf_hal::{delay::FreeRtos, i2c};
use esp_idf_hal::peripherals::Peripherals;
use ltr303::{LTR303, LTR303Config};

fn main() {
    esp_idf_sys::link_patches();

    let _peripherals = Peripherals::take().unwrap();
    // The i2c pins
    let sda = _peripherals.pins.gpio4.into_input_output().unwrap();
    let scl = _peripherals.pins.gpio6.into_output().unwrap();

    let _cfg = i2c::config::MasterConfig::new().baudrate(10000.into());
    let _i2c = i2c::Master::new(_peripherals.i2c0, i2c::MasterPins { sda, scl }, _cfg).unwrap();

    let mut ltr303 = LTR303::init(_i2c);
    let ltr303_config =
        LTR303Config::default().with_integration_time(ltr303::IntegrationTime::Ms400);
    ltr303.start_measurement(&ltr303_config).unwrap();

    loop {
        while ltr303.data_ready().unwrap() != true {
            // Wait for measurement ready
        }

        let lux_val = ltr303.get_lux_data().unwrap();
        println!("LTR303 current lux phys: {}", lux_val.lux_phys);

        FreeRtos.delay_ms(3000_u32);
    }
}

```

## Development

For easy development, a `flake.nix` is provided. Make sure you have [Nix](https://nixos.org/) installed, as well as the flake command enabled (for example by adding `experimental-features = nix-command flakes` to `~/.config/nix/nix.conf`)
and then simply run

```
nix develop
```

inside the project folder to have a complete build and development environment with all required dependencies for `ltr303-rs`.

[crates-io]: https://crates.io/crates/ltr303
[crates-io-badge]: https://img.shields.io/crates/v/ltr303.svg?maxAge=3600
[crates-io-download]: https://crates.io/crates/ltr303
[crates-io-download-badge]: https://img.shields.io/crates/d/ltr303.svg?maxAge=3600
[no-std-badge]: https://img.shields.io/badge/no__std-yes-blue
