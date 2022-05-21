/// All possible errors in this crate
#[derive(Debug)]
pub enum Error<E> {
    /// I²C bus error.
    I2C(E),
    /// A manual-configuration-mode-only was attempted while in automatic
    /// configuration mode.
    OperationNotAvailable,
}

pub struct LuxData {
    pub ch0_raw: u16,
    pub ch1_raw: u16,
    pub lux_phys: f32,
}
