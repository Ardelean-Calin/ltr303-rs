use crate::{
    types::{LuxData, RawData},
    ControlRegister, DataStatus, Gain, IntegrationTime, LTR303Config, MeasRateRegister, Mode,
    Register, StatusRegister, LTR303_BASE_ADDRESS,
};

use embedded_hal_async::i2c;

/// `async`-Implementation of the LTR303 device driver
pub struct LTR303Async<I2C> {
    i2c: I2C,
    gain: Gain,
    integration_time: IntegrationTime,
}

impl<I2C, E> LTR303Async<I2C>
where
    I2C: i2c::I2c<Error = E>,
{
    /// Initializes the LTR303 driver while consuming the i2c bus
    pub fn init(i2c: I2C) -> Self {
        Self {
            i2c,
            gain: Gain::Gain1x,
            integration_time: IntegrationTime::Ms100,
        }
    }

    /// Get the manufacturer ID stored inside LTR303. This ID should be 0x05.
    pub async fn get_mfc_id(&mut self) -> Result<u8, E> {
        self.read_register(Register::MANUFAC_ID).await
    }

    /// Get the part ID stored inside LTR303. This ID should be 0xA0.
    pub async fn get_part_id(&mut self) -> Result<u8, E> {
        self.read_register(Register::PART_ID).await
    }

    /// Destroy driver instance, return I²C bus instance.
    pub async fn destroy(self) -> I2C {
        self.i2c
    }

    // Starts a single-shot measurement!
    pub async fn start_measurement(&mut self, config: &LTR303Config) -> Result<(), E> {
        // Save the current gain and integration times => To be used when translating raw to phys
        self.gain = config.gain;
        self.integration_time = config.integration_time;

        // Configure gain, set active mode
        let control_reg = ControlRegister::default()
            .with_gain(config.gain)
            .with_mode(Mode::ACTIVE);

        // Then configure the integration time & measurement rate
        let meas_rate_reg = MeasRateRegister::default()
            .with_integration_time(config.integration_time)
            .with_measurement_rate(config.measurement_rate);

        self.write_register(Register::ALS_MEAS_RATE, meas_rate_reg.value())
            .await?;

        // Then, configure the thresholds for the interrupt!
        self.write_register(
            Register::ALS_THRES_LOW_0,
            config.int_thrsh_down.to_be_bytes()[1],
        )
        .await?;
        self.write_register(
            Register::ALS_THRES_LOW_1,
            config.int_thrsh_down.to_be_bytes()[0],
        )
        .await?;
        self.write_register(
            Register::ALS_THRES_UP_0,
            config.int_thrsh_up.to_be_bytes()[1],
        )
        .await?;
        self.write_register(
            Register::ALS_THRES_UP_1,
            config.int_thrsh_up.to_be_bytes()[0],
        )
        .await?;

        // Then enable interrupts
        // TODO: Implement similar to the other registers, with bits and InterruptReg.set_high(Flags::ISREnable)
        self.write_register(Register::INTERRUPT, 0b00000010).await?;

        // Then we start a measurement
        self.write_register(Register::ALS_CONTR, control_reg.value())
            .await?;

        Ok(())
    }

    /// Returns the contents of the ALS_STATUS register.
    pub async fn get_status(&mut self) -> Result<StatusRegister, E> {
        let data = self.read_register(Register::ALS_STATUS).await?;

        let status_reg: StatusRegister = data.into();
        Ok(status_reg)
    }

    /// Check if new sensor data is ready.
    pub async fn data_ready(&mut self) -> Result<bool, E> {
        let status = self.get_status().await?;
        Ok(status.data_status.value == DataStatus::New)
    }

    /// Reads the Ambient Light Level from LTR303's registers and returns the physical
    /// lux value.
    pub async fn get_lux_data(&mut self) -> Result<LuxData, E> {
        let raw_data = self.get_raw_data().await?;

        Ok(LuxData {
            lux_raw: raw_data,
            gain: self.gain,
            integration_time: self.integration_time,
        })
    }

    /// Puts the sensor in a low-power Standby mode where it consumes 5uA of current.
    pub async fn standby(&mut self) -> Result<(), E> {
        self.write_register(
            Register::ALS_CONTR,
            ControlRegister::default().with_mode(Mode::STANDBY).value(),
        )
        .await?;
        Ok(())
    }
}

impl<I2C, E> LTR303Async<I2C>
where
    I2C: i2c::I2c<Error = E>,
{
    async fn write_register(&mut self, register: u8, data: u8) -> Result<(), E> {
        self.i2c
            .write(LTR303_BASE_ADDRESS, &[register, data])
            .await
            .and(Ok(()))
    }

    async fn read_register(&mut self, register: u8) -> Result<u8, E> {
        let mut data: [u8; 1] = [0];
        self.i2c
            .write_read(LTR303_BASE_ADDRESS, &[register], &mut data)
            .await
            .and(Ok(data[0]))
    }

    async fn get_raw_data(&mut self) -> Result<RawData, E> {
        // Read raw illuminance data
        // Use a single transaction to ensure that the data is from the same measurement
        // (see pg. 17 of datasheet)
        let mut data: [u8; 4] = [0; 4];
        self.i2c
            .write_read(LTR303_BASE_ADDRESS, &[Register::ALS_DATA_CH1_0], &mut data)
            .await?;
        let ch1_raw = u16::from_le_bytes([data[0], data[1]]);
        let ch0_raw = u16::from_le_bytes([data[2], data[3]]);

        Ok(RawData { ch0_raw, ch1_raw })
    }
}
