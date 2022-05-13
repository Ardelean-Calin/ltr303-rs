use anyhow::Result;
use embedded_hal::blocking::i2c;
use modular_bitfield::{
    bitfield,
    prelude::{B1, B2, B3},
};

mod types;
pub use crate::types::Error;

const DEVICE_BASE_ADDRESS: u8 = 0x29;

struct Register;
impl Register {
    const ALS_CONTR: u8 = 0x80;
    const ALS_MEAS_RATE: u8 = 0x85;
    const PART_ID: u8 = 0x86;
    const MANUFAC_ID: u8 = 0x87;
    const ALS_DATA_CH1_0: u8 = 0x88;
    const ALS_DATA_CH1_1: u8 = 0x89;
    const ALS_DATA_CH0_0: u8 = 0x8A;
    const ALS_DATA_CH0_1: u8 = 0x8B;
    const ALS_STATUS: u8 = 0x8C;
    const INTERRUPT: u8 = 0x8F;
    const ALS_THRES_UP_0: u8 = 0x97;
    const ALS_THRES_UP_1: u8 = 0x98;
    const ALS_THRES_LOW_0: u8 = 0x99;
    const ALS_THRES_LOW_1: u8 = 0x9A;
    const INTERRUPT_PERSIST: u8 = 0x9E;
}

#[derive(Clone, Copy)]
pub enum Mode {
    STANDBY = 0x00,
    ACTIVE = 0x01,
}

#[derive(Clone, Copy)]
pub enum Gain {
    Gain1x = 0x00,
    Gain2x = 0x01,
    Gain4x = 0x02,
    Gain8x = 0x03,
    Gain48x = 0x06,
    Gain96x = 0x07,
}

impl Into<f32> for &Gain {
    fn into(self) -> f32 {
        match self {
            Gain::Gain1x => 1.0,
            Gain::Gain2x => 2.0,
            Gain::Gain4x => 4.0,
            Gain::Gain8x => 8.0,
            Gain::Gain48x => 48.0,
            Gain::Gain96x => 96.0,
        }
    }
}

#[derive(Clone, Copy)]
pub enum MeasurementRate {
    Ms50 = 0x00,
    Ms100 = 0x01,
    Ms200 = 0x02,
    Ms500 = 0x03,
    Ms1000 = 0x04,
    Ms2000 = 0x05,
}

#[derive(Clone, Copy)]
pub enum IntegrationTime {
    Ms100 = 0x00,
    Ms50 = 0x01,
    Ms200 = 0x02,
    Ms400 = 0x03,
    Ms150 = 0x04,
    Ms250 = 0x05,
    Ms300 = 0x06,
    Ms350 = 0x07,
}
impl Into<f32> for &IntegrationTime {
    // See: https://github.com/aniketpalu/LTR303/blob/main/LTR-303%20329_Appendix%20A%20Ver_1.0_22%20Feb%202013.pdf
    fn into(self) -> f32 {
        match self {
            IntegrationTime::Ms50 => 0.5,
            IntegrationTime::Ms100 => 1.0,
            IntegrationTime::Ms150 => 1.5,
            IntegrationTime::Ms200 => 2.0,
            IntegrationTime::Ms250 => 2.5,
            IntegrationTime::Ms300 => 3.0,
            IntegrationTime::Ms350 => 3.5,
            IntegrationTime::Ms400 => 4.0,
        }
    }
}

struct Config {
    mode: Mode,
    gain: Gain,
    integration_time: IntegrationTime,
    measurement_rate: MeasurementRate,
    threshold_up: u16,
    threshold_low: u16,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            mode: crate::Mode::STANDBY,
            gain: crate::Gain::Gain1x,
            integration_time: crate::IntegrationTime::Ms100,
            measurement_rate: crate::MeasurementRate::Ms500,
            threshold_up: 0x0000,
            threshold_low: 0xFFFF,
        }
    }
}

#[bitfield]
struct ControlReg {
    mode: B1,
    sw_reset: B1,
    gain: B3,
    na: B3,
}

impl Default for ControlReg {
    fn default() -> Self {
        ControlReg::new()
            .with_na(0)
            .with_gain(Gain::Gain1x as u8)
            .with_mode(Mode::STANDBY as u8)
            .with_sw_reset(0)
    }
}

#[bitfield]
struct MeasRateReg {
    measurement_rate: B3,
    integration_time: B3,
    na: B2,
}

impl Default for MeasRateReg {
    fn default() -> Self {
        // Default 100ms integration time with 500ms measurement rate
        MeasRateReg::new()
            .with_na(0)
            .with_integration_time(IntegrationTime::Ms100 as u8)
            .with_measurement_rate(MeasurementRate::Ms500 as u8)
    }
}

// struct BitFlags;
// impl BitFlags{
//     const
// }

struct LTR303<I2C> {
    i2c: I2C,
    address: u8,
    measurement_started: bool,
    data_ready: bool,
    config: Config,
}

impl<I2C, E> LTR303<I2C>
where
    I2C: i2c::WriteRead<Error = E> + i2c::Read<Error = E> + i2c::Write<Error = E>,
{
    /// Initializes the LTR303 driver while consuming the i2c bus
    pub fn init(i2c: I2C, config: Config) -> Self {
        LTR303 {
            i2c,
            address: DEVICE_BASE_ADDRESS,
            measurement_started: false,
            data_ready: false,
            config: config,
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
    pub fn start_measurement(&mut self) -> Result<(), Error<E>> {
        // Configure gain, set active mode
        let control_reg = ControlReg::default()
            .with_gain(self.config.gain as u8)
            .with_mode(Mode::ACTIVE as u8);

        // Then configure the integration time & measurement rate
        let meas_rate_reg = MeasRateReg::default()
            .with_integration_time(self.config.integration_time as u8)
            .with_measurement_rate(self.config.measurement_rate as u8);
        self.write_register(
            Register::ALS_MEAS_RATE,
            meas_rate_reg.bytes.first().unwrap().to_owned(),
        )?;

        // Then, configure the thresholds for the interrupt!
        self.write_register(
            Register::ALS_THRES_LOW_0,
            self.config.threshold_low.to_be_bytes()[1],
        )?;
        self.write_register(
            Register::ALS_THRES_LOW_1,
            self.config.threshold_low.to_be_bytes()[0],
        )?;
        self.write_register(
            Register::ALS_THRES_UP_0,
            self.config.threshold_up.to_be_bytes()[1],
        )?;
        self.write_register(
            Register::ALS_THRES_UP_1,
            self.config.threshold_up.to_be_bytes()[0],
        )?;

        // Then enable interrupts
        // TODO: Implement similar to the other registers, with bits and InterruptReg.set_high(Flags::ISREnable)
        self.write_register(Register::INTERRUPT, 0b00000010)?;

        // Then we start a measurement
        self.write_register(
            Register::ALS_CONTR,
            control_reg.bytes.first().unwrap().to_owned(),
        )?;
        self.measurement_started = true;

        Ok(())
    }

    // TODO: CH1 data should be read before CH0 data (see pg. 17 of datasheet)
    pub fn get_raw_data(&mut self) -> Result<(), Error<E>> {
        // At the end, put the sensor in standby, since we don't need periodic measurements!
        self.write_register(
            Register::ALS_CONTR,
            ControlReg::default()
                .with_mode(Mode::STANDBY as u8)
                .bytes
                .first()
                .unwrap()
                .to_owned(),
        )?;
        todo!()
    }
    pub fn get_lux_data(&mut self) {}
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

fn raw_to_lux(ch1_data: u16, ch0_data: u16, ltr303_config: &Config) -> f32 {
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

    use super::{Error, Register, LTR303};
    use embedded_hal_mock::i2c;
    const LTR303_ADDR: u8 = 0x29;

    #[test]
    fn manufacturer_info() {
        let expectations = [i2c::Transaction::write_read(
            LTR303_ADDR,
            vec![Register::MANUFAC_ID],
            vec![0x05],
        )];
        let mock = i2c::Mock::new(&expectations);

        let mut ltr303 = LTR303::init(mock, crate::Config::default());
        let mfc = ltr303.get_mfc_id().unwrap();
        assert_eq!(0x05, mfc);

        let mut mock = ltr303.destroy();
        mock.done(); // verify expectations
    }

    #[test]
    fn part_id() {
        let expectations = [i2c::Transaction::write_read(
            LTR303_ADDR,
            vec![Register::PART_ID],
            vec![0xA0],
        )];
        let mock = i2c::Mock::new(&expectations);

        let mut ltr303 = LTR303::init(mock, crate::Config::default());
        let part_id = ltr303.get_part_id().unwrap();
        assert_eq!(0xA0, part_id);

        let mut mock = ltr303.destroy();
        mock.done(); // verify expectations
    }

    #[test]
    fn start_measurement() {
        let expectations = [
            i2c::Transaction::write(
                LTR303_ADDR,
                vec![Register::ALS_MEAS_RATE, 0b00010101], // 200ms integration time & 2000ms meas rate
            ),
            i2c::Transaction::write(LTR303_ADDR, vec![Register::ALS_THRES_LOW_0, 0xFF]),
            i2c::Transaction::write(LTR303_ADDR, vec![Register::ALS_THRES_LOW_1, 0xFF]),
            i2c::Transaction::write(LTR303_ADDR, vec![Register::ALS_THRES_UP_0, 0x00]),
            i2c::Transaction::write(LTR303_ADDR, vec![Register::ALS_THRES_UP_1, 0x00]),
            i2c::Transaction::write(LTR303_ADDR, vec![Register::INTERRUPT, 0b00000010]),
            i2c::Transaction::write(
                LTR303_ADDR,
                vec![Register::ALS_CONTR, 0b00000001], // Active mode, default otherwise
            ),
        ];

        let mock = i2c::Mock::new(&expectations);

        let mut config = crate::Config::default();
        config.integration_time = crate::IntegrationTime::Ms200;
        config.measurement_rate = crate::MeasurementRate::Ms2000;

        let mut ltr303 = LTR303::init(mock, config);

        ltr303.start_measurement().unwrap();

        let mut mock = ltr303.destroy();
        mock.done(); // verify expectations
    }

    #[cfg(test)]
    mod unit_tests {
        use crate::raw_to_lux;

        #[test]
        fn calculate_lux_from_raw() {
            let ch0_data: u16 = 0x0000;
            let ch1_data: u16 = 0xFFFF;

            // First, test that CH1 >> CH0 returns 0 lux
            let ltr303_config = crate::Config::default();

            let lux = raw_to_lux(ch1_data, ch0_data, &ltr303_config);

            assert_eq!(lux, 0.0);

            // Then a normal random value testing ratio >= 0.45 && ratio < 0.64
            let ch0_data: u16 = 0x1000;
            let ch1_data: u16 = 0x1000;
            let lux = raw_to_lux(ch1_data, ch0_data, &ltr303_config);

            assert_eq!(lux, 9517.875);
        }
    }
}

// #[cfg(test)]
// mod tests {
//     use super::*;

//     use std::io::ErrorKind;

//     use embedded_hal_mock::delay::MockNoop as NoopDelay;
//     use embedded_hal_mock::i2c::{Mock as I2cMock, Transaction};
//     use embedded_hal_mock::MockError;

//     const LTR303_ADDR: u8 = 0x29;

//     mod device_info {
//         use super::*;
//         /// Test the `get_part_id` function.
//         #[test]
//         fn part_id_register() {
//             let expectations = [
//                 Transaction::write(LTR303_ADDR, vec![Register::PART_ID]),
//                 Transaction::read(LTR303_ADDR, vec![0xA0]),
//             ];
//             let mock = I2cMock::new(&expectations);
//             let mut ltr303 = LTR303::new(mock);
//             let val = ltr303.get_part_id().unwrap();

//             assert_eq!(val, 0xA0);

//             ltr303.destroy().done();
//         }

//         // /// Test the `device_identifier` function.
//         // #[test]
//         // fn device_identifier() {
//         //     let msb = 0b00001000;
//         //     let lsb = 0b00000111;
//         //     let crc = crc8(&[msb, lsb]);
//         //     let expectations = [
//         //         Transaction::write(SHT_ADDR, vec![0xef, 0xc8]),
//         //         Transaction::read(SHT_ADDR, vec![msb, lsb, crc]),
//         //     ];
//         //     let mock = I2cMock::new(&expectations);
//         //     let mut sht = shtc3(mock);
//         //     let ident = sht.device_identifier().unwrap();
//         //     assert_eq!(ident, 0b01000111);
//         //     sht.destroy().done();
//         // }
//     }
// }
