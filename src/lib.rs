#![cfg_attr(target_arch = "xtensa", no_std)]

pub mod can;
#[cfg(target_arch = "xtensa")]
pub mod constants;
pub mod gnss;
#[cfg(any(test, target_arch = "xtensa"))]
mod lora_scheduler;
#[cfg(all(target_arch = "xtensa", feature = "lora-timing-debug"))]
mod lora_timing;
pub mod lora_uplink;
pub mod payload;
#[cfg(target_arch = "xtensa")]
pub mod state;
#[cfg(target_arch = "xtensa")]
pub mod tasks;

#[cfg(target_arch = "xtensa")]
pub use gnss::{
    DYNAMIC_MODEL_AIRBORNE_4G, FixQuality, GLL_DELETE, GSA_DELETE, GST_ENABLE_UART1, GSV_DELETE,
    GgaData, GgaParseError, GstData, MEAS_RATE, NmeaParseError, QZSS_L1S_ENABLE, RmcData,
    UART_BAUD, UtcTime, VTG_DELETE, gnss_setting, parse_gga, parse_gst, parse_rmc_movement,
};
