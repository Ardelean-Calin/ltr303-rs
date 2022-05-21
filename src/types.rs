/// All possible errors in this crate
#[derive(Debug)]
pub enum Error<E> {
    /// I²C bus error.
    I2C(E),
    /// A manual-configuration-mode-only was attempted while in automatic
    /// configuration mode.
    OperationNotAvailable,
}

#[derive(Clone, Copy)]
pub struct RawData {
    pub ch0_raw: u16,
    pub ch1_raw: u16,
}

#[derive(Clone, Copy)]
pub struct LuxData {
    pub lux_raw: RawData,
    pub lux_phys: f32,
}
