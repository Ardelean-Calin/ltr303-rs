use crate::{Gain, IntegrationTime};

/// Raw measurement data from the sensor.
#[derive(Clone, Copy)]
pub struct RawData {
    pub ch0_raw: u16,
    pub ch1_raw: u16,
}

/// A measurement from the sensor.
#[derive(Clone, Copy)]
pub struct LuxData {
    /// Raw sensor data.
    pub(crate) lux_raw: RawData,

    /// [`Gain`] this data was measured with.
    pub(crate) gain: Gain,

    /// [`IntegrationTime`] this data was measured with.
    pub(crate) integration_time: IntegrationTime,
}

impl LuxData {
    /// Raw value returned from the sensor
    pub fn lux_raw(&self) -> &RawData {
        &self.lux_raw
    }

    /// Value in milli-Lux.
    pub fn millilux_phys(&self) -> u32 {
        raw_to_millilux(
            self.lux_raw.ch1_raw,
            self.lux_raw.ch0_raw,
            self.gain,
            self.integration_time,
        )
    }
}

fn raw_to_millilux(ch1_data: u16, ch0_data: u16, gain: Gain, itime: IntegrationTime) -> u32 {
    // See: https://github.com/aniketpalu/LTR303/blob/main/LTR-303%20329_Appendix%20A%20Ver_1.0_22%20Feb%202013.pdf
    let ratio = (ch1_data as u32 * 1000).checked_div(ch0_data as u32 + ch1_data as u32);
    let als_gain = match gain {
        Gain::Gain1x => 1,
        Gain::Gain2x => 2,
        Gain::Gain4x => 4,
        Gain::Gain8x => 8,
        Gain::Gain48x => 48,
        Gain::Gain96x => 96,
    };

    // scaled 10x
    let int_time = match itime {
        IntegrationTime::Ms50 => 5,
        IntegrationTime::Ms100 => 10,
        IntegrationTime::Ms150 => 15,
        IntegrationTime::Ms200 => 20,
        IntegrationTime::Ms250 => 25,
        IntegrationTime::Ms300 => 30,
        IntegrationTime::Ms350 => 35,
        IntegrationTime::Ms400 => 40,
    };

    // scaled 10000x
    let factors = match ratio {
        Some(0..=449) => (17743, 11059),
        Some(450..=639) => (42785, -19548),
        Some(640..=849) => (5926, 1185),
        _ => (0, 0),
    };

    (((factors.0 * ch0_data as i64) + (factors.1 * ch1_data as i64)) / als_gain as i64) as u32
        / int_time
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn calculate_lux_from_raw() {
        let ch0_data: u16 = 0x0000;
        let ch1_data: u16 = 0xFFFF;

        // First, test that CH1 >> CH0 returns 0 lux
        let ltr303_config = crate::LTR303Config::default();

        let lux = raw_to_lux(
            ch1_data,
            ch0_data,
            ltr303_config.gain,
            ltr303_config.integration_time,
        );

        assert_eq!(lux, 0.0);

        // Then a normal random value testing ratio >= 0.45 && ratio < 0.64
        let ch0_data: u16 = 0x1000;
        let ch1_data: u16 = 0x1000;
        let lux = raw_to_lux(
            ch1_data,
            ch0_data,
            ltr303_config.gain,
            ltr303_config.integration_time,
        );
        let lux_u32 = raw_to_lux_u32(
            ch1_data,
            ch0_data,
            ltr303_config.gain,
            ltr303_config.integration_time,
        );

        assert_eq!(lux, 9517.875);
        assert_eq!(lux_u32, 9517_875);
    }
}
