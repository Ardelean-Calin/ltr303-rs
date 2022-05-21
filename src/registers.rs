extern crate num as num_renamed;
use crate::create_register;
use crate::fields::*;
use num_renamed::FromPrimitive;
use num_renamed::ToPrimitive;
use paste::paste;

pub mod helpers {
    #[inline]
    pub fn get_mask(start_index: u8, width: u8) -> u8 {
        ((1u8 << width) - 1u8) << start_index
    }
}

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

// General Field structure used by registers
pub struct Field<T> {
    pub start_index: u8,
    pub width: u8,
    pub value: T,
}

impl<T> Field<T>
where
    T: ToPrimitive,
{
    pub fn bits(self) -> u8 {
        // First create a mask of N '1' bits to be used to truncate the value
        // The algorithm: ((1 << length) - 1) << pos
        let mask: u8 = self::helpers::get_mask(self.start_index, self.width);

        let val: u8 = num_renamed::ToPrimitive::to_u8(&self.value).unwrap();
        let tmp: u8 = (val << self.start_index) & mask;
        tmp
    }
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

        paste! {
            // Creates a From<u8> implementation for this register
            impl From<u8> for $reg_name {
                fn from(val: u8) -> Self {
                    let new_reg = $reg_name::default();

                    $(
                        let [<$element _mask>] = self::helpers::get_mask(new_reg.$element.start_index, new_reg.$element.width);
                        let [<$element _val>] = FromPrimitive::from_u8( (val & [<$element _mask>]) >> new_reg.$element.start_index ).unwrap();
                        let new_reg = new_reg.[<with_ $element>]([<$element _val>]);
                    )*

                    new_reg
                }
            }
        }
    }
}

create_register!(ControlRegister, {mode: Mode, gain: Gain, sw_reset: ResetStatus});

impl Default for ControlRegister {
    fn default() -> Self {
        ControlRegister {
            mode: Field {
                start_index: 0,
                width: 1,
                value: Mode::STANDBY,
            },
            gain: Field {
                start_index: 2,
                width: 3,
                value: Gain::Gain1x,
            },
            sw_reset: Field {
                start_index: 1,
                width: 1,
                value: ResetStatus::Idle,
            },
        }
    }
}

create_register!(MeasRateRegister, {measurement_rate: MeasurementRate, integration_time: IntegrationTime});

impl Default for MeasRateRegister {
    fn default() -> Self {
        MeasRateRegister {
            measurement_rate: Field {
                start_index: 0,
                width: 3,
                value: MeasurementRate::Ms500,
            },
            integration_time: Field {
                start_index: 3,
                width: 3,
                value: IntegrationTime::Ms100,
            },
        }
    }
}

create_register!(InterruptRegister, {interrupt_mode: ISRMode, interrupt_polarity: ISRPol});

impl Default for InterruptRegister {
    fn default() -> Self {
        InterruptRegister {
            interrupt_mode: Field {
                start_index: 1,
                width: 1,
                value: ISRMode::INACTIVE,
            },
            interrupt_polarity: Field {
                start_index: 2,
                width: 1,
                value: ISRPol::ActiveLow,
            },
        }
    }
}

create_register!(StatusRegister, {data_valid: DataValidity, gain: Gain, int_status: IntStatus, data_status: DataStatus});

impl Default for StatusRegister {
    fn default() -> Self {
        StatusRegister {
            data_valid: Field {
                start_index: 7,
                width: 1,
                value: DataValidity::DataValid,
            },
            gain: Field {
                start_index: 4,
                width: 3,
                value: Gain::Gain1x,
            },
            int_status: Field {
                start_index: 3,
                width: 1,
                value: IntStatus::Inactive,
            },
            data_status: Field {
                start_index: 2,
                width: 1,
                value: DataStatus::Old,
            },
        }
    }
}
