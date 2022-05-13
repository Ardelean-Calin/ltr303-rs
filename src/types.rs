/// All possible errors in this crate
#[derive(Debug)]
pub enum Error<E> {
    /// I²C bus error.
    I2C(E),
    /// A manual-configuration-mode-only was attempted while in automatic
    /// configuration mode.
    OperationNotAvailable,
}