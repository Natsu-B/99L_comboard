pub const BUF_SIZE: usize = 2048;
pub const LORA_TRANSMIT_INTERVAL_MS: u64 = 500;
pub const LORA_RX_TX_GUARD_MS: u64 = 60;
// 実機110送信すべてでflush後1〜3 us（平均2.1 us）にAUX Lowを観測した。
// 15 msはLow開始を取りこぼさないための異常判定時間であり、送信後の固定待機ではない。
pub const LORA_AUX_LOW_OBSERVE_TIMEOUT_MS: u64 = 15;
pub const SD_LOG_INTERVAL_MS: u64 = 100;
pub const SD_FLUSH_INTERVAL_SECS: u64 = 1;
pub const CAN_TX_TIMEOUT_MS: u64 = 20;
pub const CAN_HEALTH_MONITOR_INTERVAL_MS: u64 = 100;
pub const CAN_CONSECUTIVE_ERROR_THRESHOLD: u8 = 3;
pub const LORA_AUX_TIMEOUT_MS: u64 = 1_000;
