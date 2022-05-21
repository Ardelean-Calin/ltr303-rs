#![no_std]
#[macro_use]
extern crate num_derive;
use anyhow::Result;
use embedded_hal::blocking::i2c;
use types::LuxData;
use types::RawData;

mod fields;
mod registers;
mod types;
pub use crate::fields::*;
pub use crate::registers::*;
pub use crate::types::Error;

const DEVICE_BASE_ADDRESS: u8 = 0x29;

struct LTR303Config {
    mode: Mode,
    gain: Gain,
    integration_time: IntegrationTime,
    measurement_rate: MeasurementRate,
    threshold_up: u16,
    threshold_low: u16,
}

impl Default for LTR303Config {
    fn default() -> Self {
        LTR303Config {
            mode: crate::Mode::STANDBY,
            gain: crate::Gain::Gain1x,
            integration_time: crate::IntegrationTime::Ms100,
            measurement_rate: crate::MeasurementRate::Ms500,
            threshold_up: 0x0000,
            threshold_low: 0xFFFF,
        }
    }
}

struct SensorStatus {
    data_status: bool,
    int_status: bool,
    gain: Gain,
    valid: bool,
}

struct LTR303<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C, E> LTR303<I2C>
where
    I2C: i2c::WriteRead<Error = E> + i2c::Read<Error = E> + i2c::Write<Error = E>,
{
    /// Initializes the LTR303 driver while consuming the i2c bus
    pub fn init(i2c: I2C) -> Self {
        LTR303 {
            i2c,
            address: DEVICE_BASE_ADDRESS,
        }
    }

    pub fn get_mfc_id(&mut self) -> Result<u8, Error<E>> {
        self.read_register(Register::MANUFAC_ID)
    }

    pub fn get_part_id(&mut self) -> Result<u8, Error<E>> {
        self.read_register(Register::PART_ID)
    }

    /// Destroy driver instance, return I²C bus instance.
    pub fn destroy(self) -> I2C {
        self.i2c
    }

    // Starts a single-shot measurement!
    pub fn start_measurement(&mut self, config: &LTR303Config) -> Result<(), Error<E>> {
        // Configure gain, set active mode
        let control_reg = ControlRegister::default()
            .with_gain(config.gain)
            .with_mode(Mode::ACTIVE);

        // Then configure the integration time & measurement rate
        let meas_rate_reg = MeasRateRegister::default()
            .with_integration_time(config.integration_time)
            .with_measurement_rate(config.measurement_rate);

        self.write_register(Register::ALS_MEAS_RATE, meas_rate_reg.value())?;

        // Then, configure the thresholds for the interrupt!
        self.write_register(
            Register::ALS_THRES_LOW_0,
            config.threshold_low.to_be_bytes()[1],
        )?;
        self.write_register(
            Register::ALS_THRES_LOW_1,
            config.threshold_low.to_be_bytes()[0],
        )?;
        self.write_register(
            Register::ALS_THRES_UP_0,
            config.threshold_up.to_be_bytes()[1],
        )?;
        self.write_register(
            Register::ALS_THRES_UP_1,
            config.threshold_up.to_be_bytes()[0],
        )?;

        // Then enable interrupts
        // TODO: Implement similar to the other registers, with bits and InterruptReg.set_high(Flags::ISREnable)
        self.write_register(Register::INTERRUPT, 0b00000010)?;

        // Then we start a measurement
        self.write_register(Register::ALS_CONTR, control_reg.value())?;

        Ok(())
    }

    /// Returns the contents of the ALS_STATUS register.
    pub fn get_status(&mut self) -> Result<StatusRegister, Error<E>> {
        let data = self.read_register(Register::ALS_STATUS)?;

        let status_reg: StatusRegister = data.into();
        Ok(status_reg)
    }

    // TODO: CH1 data should be read before CH0 data (see pg. 17 of datasheet)
    fn get_raw_data(&mut self) -> Result<RawData, Error<E>> {
        // Read raw data
        let ch1_0 = self.read_register(Register::ALS_DATA_CH1_0)? as u16;
        let ch1_1 = self.read_register(Register::ALS_DATA_CH1_1)? as u16;
        let ch0_0 = self.read_register(Register::ALS_DATA_CH0_0)? as u16;
        let ch0_1 = self.read_register(Register::ALS_DATA_CH0_1)? as u16;

        Ok(RawData {
            ch0_raw: (ch0_1 << 8) | ch0_0,
            ch1_raw: (ch1_1 << 8) | ch1_0,
        })
    }

    pub fn get_lux_data(&mut self, config: &LTR303Config) -> Result<LuxData, Error<E>> {
        let raw_data = self.get_raw_data()?;

        Ok(LuxData {
            lux_raw: raw_data,
            lux_phys: raw_to_lux(raw_data.ch1_raw, raw_data.ch0_raw, config),
        })
    }

    pub fn standby(&mut self) -> Result<(), Error<E>> {
        self.write_register(
            Register::ALS_CONTR,
            ControlRegister::default().with_mode(Mode::STANDBY).value(),
        )?;
        Ok(())
    }
}

impl<I2C, E> LTR303<I2C>
where
    I2C: i2c::WriteRead<Error = E> + i2c::Write<Error = E>,
{
    fn write_register(&mut self, register: u8, data: u8) -> Result<(), Error<E>> {
        self.i2c
            .write(self.address, &[register, data])
            .map_err(Error::I2C)
            .and(Ok(()))
    }

    fn read_register(&mut self, register: u8) -> Result<u8, Error<E>> {
        let mut data = [0];
        self.i2c
            .write_read(self.address, &[register], &mut data)
            .map_err(Error::I2C)
            .and(Ok(u8::from(data[0])))
    }
}

fn raw_to_lux(ch1_data: u16, ch0_data: u16, ltr303_config: &LTR303Config) -> f32 {
    let ratio = ch1_data as f32 / (ch0_data as f32 + ch1_data as f32);
    let als_gain: f32 = (&ltr303_config.gain).into();
    let int_time: f32 = (&ltr303_config.integration_time).into();

    let lux_phys = if ratio < 0.45 {
        ((1.7743 * f32::from(ch0_data)) + (1.1059 * f32::from(ch1_data))) / als_gain / int_time
    } else if ratio >= 0.45 && ratio < 0.64 {
        ((4.2785 * f32::from(ch0_data)) - (1.9548 * f32::from(ch1_data))) / als_gain / int_time
    } else if ratio >= 0.64 && ratio < 0.85 {
        ((0.5926 * f32::from(ch0_data)) - (0.1185 * f32::from(ch1_data))) / als_gain / int_time
    } else {
        0.0
    };

    lux_phys
}

#[cfg(test)]
mod tests {
    // this code lives inside a `tests` module

    extern crate std;
    use crate::{DataStatus, DataValidity, Gain, IntStatus};

    use super::*;

    use embedded_hal_mock::i2c;
    const LTR303_ADDR: u8 = 0x29;

    #[test]
    fn manufacturer_info() {
        let expectations = [i2c::Transaction::write_read(
            LTR303_ADDR,
            std::vec![Register::MANUFAC_ID],
            std::vec![0x05],
        )];
        let mock = i2c::Mock::new(&expectations);

        let mut ltr303 = LTR303::init(mock);
        let mfc = ltr303.get_mfc_id().unwrap();
        assert_eq!(0x05, mfc);

        let mut mock = ltr303.destroy();
        mock.done(); // verify expectations
    }

    #[test]
    fn part_id() {
        let expectations = [i2c::Transaction::write_read(
            LTR303_ADDR,
            std::vec![Register::PART_ID],
            std::vec![0xA0],
        )];
        let mock = i2c::Mock::new(&expectations);

        let mut ltr303 = LTR303::init(mock);
        let part_id = ltr303.get_part_id().unwrap();
        assert_eq!(0xA0, part_id);

        let mut mock = ltr303.destroy();
        mock.done(); // verify expectations
    }

    #[test]
    fn sensor_status() {
        let expectations = [i2c::Transaction::write_read(
            LTR303_ADDR,
            std::vec![Register::ALS_STATUS],
            std::vec![0b11111010],
        )];
        let mock = i2c::Mock::new(&expectations);

        let mut ltr303 = LTR303::init(mock);
        let sensor_status = ltr303.get_status().unwrap();

        assert_eq!(sensor_status.data_status.value, DataStatus::Old);
        assert_eq!(sensor_status.data_valid.value, DataValidity::DataInvalid);
        assert_eq!(sensor_status.gain.value, Gain::Gain96x);
        assert_eq!(sensor_status.int_status.value, IntStatus::Active);

        assert_eq!(sensor_status.value(), 0b11111000);

        let mut mock = ltr303.destroy();
        mock.done(); // verify expectations
    }

    #[test]
    fn start_measurement() {
        let expectations = [
            i2c::Transaction::write(
                LTR303_ADDR,
                std::vec![Register::ALS_MEAS_RATE, 0b00010101], // 200ms integration time & 2000ms meas rate
            ),
            i2c::Transaction::write(LTR303_ADDR, std::vec![Register::ALS_THRES_LOW_0, 0xFF]),
            i2c::Transaction::write(LTR303_ADDR, std::vec![Register::ALS_THRES_LOW_1, 0xFF]),
            i2c::Transaction::write(LTR303_ADDR, std::vec![Register::ALS_THRES_UP_0, 0x00]),
            i2c::Transaction::write(LTR303_ADDR, std::vec![Register::ALS_THRES_UP_1, 0x00]),
            i2c::Transaction::write(LTR303_ADDR, std::vec![Register::INTERRUPT, 0b00000010]),
            i2c::Transaction::write(
                LTR303_ADDR,
                std::vec![Register::ALS_CONTR, 0b00000001], // Active mode, default otherwise
            ),
        ];

        let mock = i2c::Mock::new(&expectations);

        let mut config = crate::LTR303Config::default();
        config.integration_time = crate::IntegrationTime::Ms200;
        config.measurement_rate = crate::MeasurementRate::Ms2000;

        let mut ltr303 = LTR303::init(mock);

        ltr303.start_measurement(&config).unwrap();

        let mut mock = ltr303.destroy();
        mock.done(); // verify expectations
    }

    // Do a complete measurement from start to finish and test that we get the proper data
    #[test]
    fn single_shot_measurement() {
        // Start a Gain4x single measurement with integration time 100ms and measurement rate 2000ms,
        // then after we got a result, put the sensor to sleep.
        let expectations = [
            i2c::Transaction::write(LTR303_ADDR, std::vec![Register::ALS_MEAS_RATE, 0b0000_0101]), // set times
            i2c::Transaction::write(LTR303_ADDR, std::vec![Register::ALS_THRES_LOW_0, 0xFF]), // set thresholds
            i2c::Transaction::write(LTR303_ADDR, std::vec![Register::ALS_THRES_LOW_1, 0xFF]), // set thresholds
            i2c::Transaction::write(LTR303_ADDR, std::vec![Register::ALS_THRES_UP_0, 0x00]), // set thresholds
            i2c::Transaction::write(LTR303_ADDR, std::vec![Register::ALS_THRES_UP_1, 0x00]), // set thresholds
            i2c::Transaction::write(LTR303_ADDR, std::vec![Register::INTERRUPT, 0b00000010]), // enable interrupts
            i2c::Transaction::write(LTR303_ADDR, std::vec![Register::ALS_CONTR, 0b0000_1001]), // wake-up sensor. start
            i2c::Transaction::write_read(
                LTR303_ADDR,
                std::vec![Register::ALS_STATUS],
                std::vec![0b1010_0000],
            ), // current status: still measuring
            i2c::Transaction::write_read(
                LTR303_ADDR,
                std::vec![Register::ALS_STATUS],
                std::vec![0b0010_0100],
            ), // current status: new data available
            i2c::Transaction::write_read(
                LTR303_ADDR,
                std::vec![Register::ALS_DATA_CH1_0],
                std::vec![0xAD],
            ), // Reading channel data
            i2c::Transaction::write_read(
                LTR303_ADDR,
                std::vec![Register::ALS_DATA_CH1_1],
                std::vec![0xDE],
            ), // Reading channel data
            i2c::Transaction::write_read(
                LTR303_ADDR,
                std::vec![Register::ALS_DATA_CH0_0],
                std::vec![0xEF],
            ), // Reading channel data
            i2c::Transaction::write_read(
                LTR303_ADDR,
                std::vec![Register::ALS_DATA_CH0_1],
                std::vec![0xBE],
            ), // Reading channel data
            i2c::Transaction::write(LTR303_ADDR, std::vec![Register::ALS_CONTR, 0b0000_0000]), // put sensor to sleep
        ];
        let mock = i2c::Mock::new(&expectations);

        // Created expected bus communication. Below is the expected developer workflow:
        let mut config = crate::LTR303Config::default();
        config.integration_time = crate::IntegrationTime::Ms100;
        config.measurement_rate = crate::MeasurementRate::Ms2000;
        config.gain = crate::Gain::Gain4x;

        let mut ltr303 = LTR303::init(mock);
        ltr303.start_measurement(&config).unwrap();

        while (ltr303.get_status().unwrap().data_status.value != DataStatus::New) {}

        let lux_data = ltr303.get_lux_data(&config).unwrap();
        ltr303.standby().unwrap();

        assert_eq!(lux_data.lux_raw.ch1_raw, 0xDEAD);
        assert_eq!(lux_data.lux_raw.ch0_raw, 0xBEEF);

        let mut mock = ltr303.destroy();
        mock.done(); // verify expectations
    }

    #[cfg(test)]
    mod unit_tests {
        use crate::{
            raw_to_lux, ControlRegister, Field, Gain, IntegrationTime, MeasRateRegister,
            MeasurementRate, Mode, ResetStatus,
        };

        #[test]
        fn calculate_lux_from_raw() {
            let ch0_data: u16 = 0x0000;
            let ch1_data: u16 = 0xFFFF;

            // First, test that CH1 >> CH0 returns 0 lux
            let ltr303_config = crate::LTR303Config::default();

            let lux = raw_to_lux(ch1_data, ch0_data, &ltr303_config);

            assert_eq!(lux, 0.0);

            // Then a normal random value testing ratio >= 0.45 && ratio < 0.64
            let ch0_data: u16 = 0x1000;
            let ch1_data: u16 = 0x1000;
            let lux = raw_to_lux(ch1_data, ch0_data, &ltr303_config);

            assert_eq!(lux, 9517.875);
        }

        #[test]
        fn test_registers() {
            // Test that the register API works as expected!
            let control_reg = ControlRegister::default()
                .with_mode(Mode::STANDBY)
                .with_gain(Gain::Gain8x);

            assert_eq!(control_reg.gain.value, Gain::Gain8x);
            assert_eq!(control_reg.value(), 0b000_011_0_0);

            let measrate_reg = MeasRateRegister::default()
                .with_integration_time(IntegrationTime::Ms200)
                .with_measurement_rate(MeasurementRate::Ms2000);

            assert_eq!(measrate_reg.integration_time.value, IntegrationTime::Ms200);
            assert_eq!(measrate_reg.value(), 0b00_010_101);
        }

        #[test]
        fn test_register_from_u8() {
            // Tests that we can properly transform a u8 value to a register with fields!
            let contr_reg_val: u8 = 0b000_010_1_1;
            let control_reg: ControlRegister = contr_reg_val.into();

            assert_eq!(control_reg.gain.value, Gain::Gain4x);
            assert_eq!(control_reg.sw_reset.value, ResetStatus::Resetting);
            assert_eq!(control_reg.mode.value, Mode::ACTIVE);
        }

        #[test]
        fn test_fields() {
            // Field one should be 0b00010110
            let field1 = Field {
                start_index: 1,
                width: 4,
                value: 0x0Bu8,
            };
            assert_eq!(field1.bits(), 0b0001_0110)
        }
    }
}
