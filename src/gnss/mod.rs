pub mod nmea;
#[cfg(target_arch = "xtensa")]
pub mod settings;
pub mod telemetry;

pub use nmea::{
    FixQuality, GgaData, GgaParseError, GstData, NmeaParseError, RmcData, UtcTime, parse_gga,
    parse_gst, parse_rmc_movement,
};
#[cfg(target_arch = "xtensa")]
pub use settings::{
    DYNAMIC_MODEL_AIRBORNE_4G, GLL_DELETE, GSA_DELETE, GST_ENABLE_UART1, GSV_DELETE, MEAS_RATE,
    QZSS_L1S_ENABLE, UART_BAUD, VTG_DELETE, gnss_setting,
};
