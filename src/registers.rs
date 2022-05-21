use crate::fields::*;
use paste::paste;

/// Register definitions
pub struct Register;
impl Register {
    pub const ALS_CONTR: u8 = 0x80;
    pub const ALS_MEAS_RATE: u8 = 0x85;
    pub const PART_ID: u8 = 0x86;
    pub const MANUFAC_ID: u8 = 0x87;
    pub const ALS_DATA_CH1_0: u8 = 0x88;
    pub const ALS_DATA_CH1_1: u8 = 0x89;
    pub const ALS_DATA_CH0_0: u8 = 0x8A;
    pub const ALS_DATA_CH0_1: u8 = 0x8B;
    pub const ALS_STATUS: u8 = 0x8C;
    pub const INTERRUPT: u8 = 0x8F;
    pub const ALS_THRES_UP_0: u8 = 0x97;
    pub const ALS_THRES_UP_1: u8 = 0x98;
    pub const ALS_THRES_LOW_0: u8 = 0x99;
    pub const ALS_THRES_LOW_1: u8 = 0x9A;
    pub const INTERRUPT_PERSIST: u8 = 0x9E;
}

/// Defines a standard structure for a 8-bit register.
///
/// This macro takes `StructName, {structfield1: type1, structfield2: type2, ...}` as arguments
/// and generates a structure:
///
/// ```compile_fail
/// struct StructName {
///     structfield1: Field<type1>,
///     structfield2: Field<type2>,
///     ...
/// }
/// ```
///
/// The structure will have automatic `with_structfieldX()` factory methods created, as well
/// as a `value()` function that returns the encoded u8 data.
///
#[macro_export]
macro_rules! create_register {
    ($reg_name:ident, {$($element: ident: $ty: ty),*}) => {
        pub struct $reg_name { $(pub $element: Field<$ty>),* }

        paste! {
            impl $reg_name {
                pub fn value(self) -> u8 {
                    let mut temp: u8 = 0x00;
                    $(
                        temp |= self.$element.bits();
                    )*
                    temp
                }

            // Creates with_<variable> methods
            $(
                pub fn [<with_ $element>] (self, paste!{[<new_ $element>]}: $ty) -> Self {
                    let mut tmp = $reg_name{..self};
                    tmp.$element.value = paste!{[<new_ $element>]};
                    tmp
                }
            )*
            }
        }
    }
}

create_register!(ControlRegister, {mode: Mode, gain: Gain, sw_reset: bool});

impl Default for ControlRegister {
    fn default() -> Self {
        ControlRegister {
            mode: Field {
                startIndex: 0,
                width: 1,
                value: Mode::STANDBY,
            },
            gain: Field {
                startIndex: 2,
                width: 3,
                value: Gain::Gain1x,
            },
            sw_reset: Field {
                startIndex: 1,
                width: 1,
                value: false,
            },
        }
    }
}

create_register!(MeasRateRegister, {measurement_rate: MeasurementRate, integration_time: IntegrationTime});

impl Default for MeasRateRegister {
    fn default() -> Self {
        MeasRateRegister {
            measurement_rate: Field {
                startIndex: 0,
                width: 3,
                value: MeasurementRate::Ms500,
            },
            integration_time: Field {
                startIndex: 3,
                width: 3,
                value: IntegrationTime::Ms100,
            },
        }
    }
}
