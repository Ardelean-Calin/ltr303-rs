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
