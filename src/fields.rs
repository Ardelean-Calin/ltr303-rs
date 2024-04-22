#[derive(Debug, Clone, Copy, PartialEq, FromPrimitive, ToPrimitive)]
pub enum Mode {
    STANDBY = 0x00,
    ACTIVE = 0x01,
}

#[derive(Debug, Clone, Copy, PartialEq, FromPrimitive, ToPrimitive)]
pub enum Gain {
    Gain1x = 0x00,
    Gain2x = 0x01,
    Gain4x = 0x02,
    Gain8x = 0x03,
    Gain48x = 0x06,
    Gain96x = 0x07,
}

#[derive(Debug, Clone, Copy, PartialEq, FromPrimitive, ToPrimitive)]
pub enum MeasurementRate {
    Ms50 = 0x00,
    Ms100 = 0x01,
    Ms200 = 0x02,
    Ms500 = 0x03,
    Ms1000 = 0x04,
    Ms2000 = 0x05,
}

#[derive(Debug, Clone, Copy, PartialEq, FromPrimitive, ToPrimitive)]
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

#[derive(Debug, Clone, Copy, PartialEq, FromPrimitive, ToPrimitive)]
pub enum ISRMode {
    INACTIVE = 0,
    ACTIVE = 1,
}

#[derive(Debug, Clone, Copy, PartialEq, FromPrimitive, ToPrimitive)]
pub enum ISRPol {
    ActiveLow = 0,
    ActiveHigh = 1,
}

#[derive(Debug, Clone, Copy, PartialEq, FromPrimitive, ToPrimitive)]
pub enum DataValidity {
    DataValid = 0,
    DataInvalid = 1,
}

#[derive(Debug, Clone, Copy, PartialEq, FromPrimitive, ToPrimitive)]
pub enum DataStatus {
    Old = 0,
    New = 1,
}

#[derive(Debug, PartialEq, FromPrimitive, ToPrimitive)]
pub enum IntStatus {
    Inactive = 0,
    Active = 1,
}

#[derive(Debug, PartialEq, FromPrimitive, ToPrimitive)]
pub enum ResetStatus {
    Idle = 0,
    Resetting = 1,
}
