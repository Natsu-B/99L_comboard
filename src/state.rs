use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32};

use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, mutex::Mutex, signal::Signal,
};

use crate::{
    can::{
        cache::CanCache, command::TransactionTracker, protocol::CanTxMessage,
        recovery::RecoverySession,
    },
    lora_uplink::UplinkCommand,
    payload::LoraFrame,
};

pub type GnssPacket = [u8; 90];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GnssCommand {
    TurnOn,
    TurnOff,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GnssReceiverState {
    Off,
    Starting,
    ReceiverDetected,
    ConfigurationFailed,
    NoFix,
    ValidFix,
    InvalidSample,
    Stale,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GnssTelemetry {
    pub state: GnssReceiverState,
    pub east: u16,
    pub north: u16,
    pub height: u16,
    pub last_receiver_at_ms: Option<u64>,
    pub last_fix_at_ms: Option<u64>,
}

impl GnssTelemetry {
    pub const fn new() -> Self {
        Self {
            state: GnssReceiverState::Off,
            east: 0x8000,
            north: 0x8000,
            height: 496,
            last_receiver_at_ms: None,
            last_fix_at_ms: None,
        }
    }
}

impl Default for GnssTelemetry {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CanTxRequest {
    pub message: CanTxMessage,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RawCanRecord {
    pub received_at_ms: u64,
    pub identifier: u16,
    pub data_length: u8,
    pub data: [u8; 8],
}

pub static CAN_CACHE: Mutex<CriticalSectionRawMutex, CanCache> = Mutex::new(CanCache::new());
pub static COMMAND_TRACKER: Mutex<CriticalSectionRawMutex, TransactionTracker> =
    Mutex::new(TransactionTracker::new());
pub static RECOVERY_SESSION: Mutex<CriticalSectionRawMutex, RecoverySession> =
    Mutex::new(RecoverySession::new());
pub static RECOVERY_BEACON_ACTIVE: AtomicBool = AtomicBool::new(false);
pub static RECOVERY_ENTER_SENT: AtomicBool = AtomicBool::new(false);
pub static GNSS_TELEMETRY: Mutex<CriticalSectionRawMutex, GnssTelemetry> =
    Mutex::new(GnssTelemetry::new());
pub static GNSS_CHANNEL: Channel<CriticalSectionRawMutex, GnssPacket, 5> = Channel::new();
pub static GNSS_CMD_CHANNEL: Channel<CriticalSectionRawMutex, GnssCommand, 2> = Channel::new();
pub static UPLINK_COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, UplinkCommand, 8> =
    Channel::new();
pub static IMMEDIATE_LORA_CHANNEL: Channel<CriticalSectionRawMutex, LoraFrame, 8> = Channel::new();
pub static RAW_CAN_LOG_CHANNEL: Channel<CriticalSectionRawMutex, RawCanRecord, 32> = Channel::new();
pub static CAN_TX_CHANNEL: Channel<CriticalSectionRawMutex, CanTxRequest, 8> = Channel::new();
pub static CAN_SAFETY_TX_SIGNAL: Signal<CriticalSectionRawMutex, CanTxRequest> = Signal::new();
pub static LOGGING_REQUESTED: AtomicBool = AtomicBool::new(false);
pub static LOGGING_ACTIVE: AtomicBool = AtomicBool::new(false);
pub static SD_HAS_ERROR: AtomicBool = AtomicBool::new(false);
pub static SD_WRITE_ERROR_COUNT: AtomicU32 = AtomicU32::new(0);
pub static SD_DROPPED_ROW_COUNT: AtomicU32 = AtomicU32::new(0);
pub static RAW_CAN_LOG_DROPPED_COUNT: AtomicU32 = AtomicU32::new(0);
pub static HAS_UNFLUSHED_DATA: AtomicBool = AtomicBool::new(false);
pub static SD_FLUSH_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static IS_CAN_ERROR: AtomicBool = AtomicBool::new(true);
pub static CAN_TEC: AtomicU8 = AtomicU8::new(0);
pub static CAN_REC: AtomicU8 = AtomicU8::new(0);
pub static CAN_HEALTH: AtomicU8 = AtomicU8::new(0);
pub static CAN_TX_ERROR_COUNT: AtomicU32 = AtomicU32::new(0);
pub static CAN_RX_ERROR_COUNT: AtomicU32 = AtomicU32::new(0);
pub static LORA_TX_ERROR_COUNT: AtomicU32 = AtomicU32::new(0);
pub static LORA_RX_ERROR_COUNT: AtomicU32 = AtomicU32::new(0);
pub static LORA_COMMAND_DROP_COUNT: AtomicU32 = AtomicU32::new(0);
pub static LORA_AUX_TIMEOUT_COUNT: AtomicU32 = AtomicU32::new(0);
pub static GNSS_SETTING_ERROR_COUNT: AtomicU32 = AtomicU32::new(0);
pub static GNSS_RX_ERROR_COUNT: AtomicU32 = AtomicU32::new(0);
pub static GNSS_CHANNEL_DROP_COUNT: AtomicU32 = AtomicU32::new(0);
