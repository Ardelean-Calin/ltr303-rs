// General Field structure
pub struct Field<T> {
    pub startIndex: u8,
    pub width: u8,
    pub value: T,
}

impl<T> Field<T>
where
    T: Into<u8>,
{
    pub fn bits(self) -> u8 {
        // First create a mask of N '1' bits to be used to truncate the value
        let mut mask: u8 = 0x00;
        for _ in 0..self.width {
            mask = mask << 1;
            mask |= 1;
        }

        let val: u8 = self.value.into();
        let tmp: u8 = (val & mask) << self.startIndex;
        tmp
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Mode {
    STANDBY = 0x00,
    ACTIVE = 0x01,
}

impl Into<u8> for Mode {
    fn into(self) -> u8 {
        self as u8
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Gain {
    Gain1x = 0x00,
    Gain2x = 0x01,
    Gain4x = 0x02,
    Gain8x = 0x03,
    Gain48x = 0x06,
    Gain96x = 0x07,
}

impl Into<u8> for Gain {
    fn into(self) -> u8 {
        self as u8
    }
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

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MeasurementRate {
    Ms50 = 0x00,
    Ms100 = 0x01,
    Ms200 = 0x02,
    Ms500 = 0x03,
    Ms1000 = 0x04,
    Ms2000 = 0x05,
}

impl Into<u8> for MeasurementRate {
    fn into(self) -> u8 {
        self as u8
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
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

impl Into<u8> for IntegrationTime {
    fn into(self) -> u8 {
        self as u8
    }
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
