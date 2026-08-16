use core::sync::atomic::Ordering;

use embassy_futures::select::{Either, Either5, select, select5};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use esp_hal::{
    Async,
    gpio::Input,
    uart::{RxError, TxError, UartRx, UartTx},
};
use esp_println::println;

use crate::{
    can::{
        cache::{FRESHNESS_10_HZ_MS, FRESHNESS_25_HZ_MS, FRESHNESS_100_HZ_MS, Freshness},
        command::RegisterResult,
        protocol::{
            CanTxMessage, CommandPhase, CommandReason, GenericCommandRequest, MissionState,
            RecoveryControl, RecoveryOpcode, RecoverySource,
        },
        recovery::PendingRecovery,
    },
    constants::{
        LORA_AUX_LOW_OBSERVE_TIMEOUT_MS, LORA_AUX_TIMEOUT_MS, LORA_RX_TX_GUARD_MS,
        LORA_TRANSMIT_INTERVAL_MS,
    },
    lora_scheduler::{
        LoRaTxEnvelope, LoRaTxSource, advance_deadline, consume_periodic_selection,
        deferred_recovery_blocked, is_higher_priority, periodic_due_for_reselection,
        periodic_retry_after_nonperiodic_attempt, periodic_retry_after_recovery_us,
        preempted_periodic_missed_slots, recovery_allowed, recovery_displaced_regular_slot,
        recovery_next_eligible_at_us, select_periodic_deadline, should_clear_periodic_events,
        should_defer_preempted, update_recovery_fairness,
    },
    lora_uplink::{UplinkCommand, UplinkFrameBuffer},
    payload::{
        ApplicationPacket, CommandReceiveTelemetry, CommandResultPacket,
        ControlRollTelemetryV2Packet, DescentTelemetry, FlightTelemetry, LoraFrame,
        MissionLinkFallbackTelemetry, PacketHeader, RecoveryBeacon,
    },
    state::{
        CAN_CACHE, CAN_FALLBACK_HEALTH, CAN_SAFETY_TX_CHANNEL, CAN_TX_CHANNEL,
        COMMAND_RESULT_LORA_CHANNEL, COMMAND_TRACKER, CONTROL_ROLL_LORA_SIGNAL,
        CanTxRequest, EMERGENCY_RESULT_LORA_CHANNEL, GNSS_CMD_CHANNEL, GNSS_TELEMETRY,
        GROUND_TIME_REQUEST_LORA_CHANNEL, GnssCommand, GnssReceiverState, HAS_UNFLUSHED_DATA,
        IS_CAN_ERROR, LOGGING_ACTIVE, LOGGING_REQUESTED, LORA_AUX_TIMEOUT_COUNT,
        LORA_COMMAND_DROP_COUNT, LORA_PERIODIC_MISSED_SLOT_COUNT, LORA_RX_BYTE_COUNT,
        LORA_RX_ERROR_COUNT, LORA_RX_SUCCESS_COUNT, LORA_TX_ERROR_COUNT,
        LORA_TX_QUEUE_DROP_COUNT, LORA_TX_SUCCESS_COUNT, MISSION_LINK_FALLBACK_SEQUENCE,
        MISSION_STATUS_INVALID_AT_MS, RECOVERY_ASSEMBLER, RECOVERY_BEACON_ACTIVE,
        RECOVERY_LORA_CHANNEL, RECOVERY_SESSION, SD_FLUSH_SIGNAL, SD_HAS_ERROR,
        UPLINK_COMMAND_CHANNEL,
    },
};

#[cfg(feature = "lora-timing-debug")]
use crate::lora_timing::{
    AUX_LOW_DURATION, FLUSH_TO_AUX_LOW, FLUSH_TO_TX_COMPLETE, IDLE_GAP, INITIAL_AUX_WAIT,
    POST_GUARD_AUX_WAIT, QUEUE_WAIT, REQUEST_INTERVAL, REQUEST_TO_WRITE_START, RX_GUARD_WAIT,
    TX_COMPLETE_INTERVAL, TX_TOTAL, TimingCollector, TimingReport, TxTimingTrace, UART_FLUSH,
    UART_WRITE, WRITE_START_INTERVAL, reselect_prepared_trace,
};

static LORA_RX_ACTIVITY_SIGNAL: Signal<CriticalSectionRawMutex, Instant> = Signal::new();

#[cfg(feature = "lora-timing-debug")]
static LORA_TIMING_REPORT_SIGNAL: Signal<CriticalSectionRawMutex, TimingReport> = Signal::new();

#[cfg(feature = "lora-timing-debug")]
fn now_us() -> u64 {
    Instant::now().as_micros()
}

#[cfg(feature = "lora-timing-debug")]
fn print_timing_report(report: TimingReport) {
    let request = report.metrics[REQUEST_INTERVAL];
    let request_to_write = report.metrics[REQUEST_TO_WRITE_START];
    let queue = report.metrics[QUEUE_WAIT];
    let initial_aux = report.metrics[INITIAL_AUX_WAIT];
    let rx_guard = report.metrics[RX_GUARD_WAIT];
    let post_guard_aux = report.metrics[POST_GUARD_AUX_WAIT];
    let uart_write = report.metrics[UART_WRITE];
    let uart_flush = report.metrics[UART_FLUSH];
    let flush_to_low = report.metrics[FLUSH_TO_AUX_LOW];
    let aux_low = report.metrics[AUX_LOW_DURATION];
    let flush_to_complete = report.metrics[FLUSH_TO_TX_COMPLETE];
    let tx_total = report.metrics[TX_TOTAL];
    let write_interval = report.metrics[WRITE_START_INTERVAL];
    let complete_interval = report.metrics[TX_COMPLETE_INTERVAL];
    let idle = report.metrics[IDLE_GAP];
    println!(
        "LORA_TIMING samples={} src_emergency={} src_b0={} src_b1={} src_recovery={} src_periodic={} request_avg_us={} request_to_write_avg_us={} queue_avg_us={} initial_aux_avg_us={} rx_guard_avg_us={} post_guard_aux_avg_us={} uart_write_avg_us={} uart_flush_avg_us={} flush_to_aux_low_count={} flush_to_aux_low_avg_us={} aux_low_count={} aux_low_avg_us={} flush_to_complete_avg_us={} tx_total_min_us={} tx_total_avg_us={} tx_total_max_us={} write_interval_min_us={} write_interval_avg_us={} write_interval_max_us={} complete_interval_avg_us={} idle_avg_us={} emergency_queue_avg_us={} periodic_tx_avg_us={} b0_tx_avg_us={} b1_tx_avg_us={} recovery_tx_avg_us={} emergency_tx_avg_us={} aux_low_not_observed={} missed_slots={} invalid_timestamps={}",
        report.sample_count(),
        report.source_samples(LoRaTxSource::EmergencyResult),
        report.source_samples(LoRaTxSource::CommandResult),
        report.source_samples(LoRaTxSource::GroundTimeRequest),
        report.source_samples(LoRaTxSource::Recovery),
        report.source_samples(LoRaTxSource::Periodic),
        request.average,
        request_to_write.average,
        queue.average,
        initial_aux.average,
        rx_guard.average,
        post_guard_aux.average,
        uart_write.average,
        uart_flush.average,
        flush_to_low.count,
        flush_to_low.average,
        aux_low.count,
        aux_low.average,
        flush_to_complete.average,
        tx_total.min,
        tx_total.average,
        tx_total.max,
        write_interval.min,
        write_interval.average,
        write_interval.max,
        complete_interval.average,
        idle.average,
        report.emergency_queue_wait.average,
        report.source_tx_total(LoRaTxSource::Periodic).average,
        report.source_tx_total(LoRaTxSource::CommandResult).average,
        report
            .source_tx_total(LoRaTxSource::GroundTimeRequest)
            .average,
        report.source_tx_total(LoRaTxSource::Recovery).average,
        report
            .source_tx_total(LoRaTxSource::EmergencyResult)
            .average,
        report.aux_low_not_observed,
        report.periodic_missed_slots,
        report.invalid_timestamp_count,
    );
}

#[cfg(feature = "lora-timing-debug")]
#[embassy_executor::task]
pub async fn lora_timing_report_task() {
    loop {
        // 長い診断行はcore 0で出力し、core 1のCAN/LoRa ownerを停止させない。
        print_timing_report(LORA_TIMING_REPORT_SIGNAL.wait().await);
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
enum LocalCommand {
    StartLogging = b'l',
    StopLogging = b'm',
    GnssOn = b'g',
    GnssOff = b'h',
    Wake = b'w',
    DumpInternalFlash = b'f',
    DumpMissionSd = b's',
    StopDump = b'x',
}

impl LocalCommand {
    const fn decode(raw: u8) -> Option<Self> {
        match raw {
            b'l' => Some(Self::StartLogging),
            b'm' => Some(Self::StopLogging),
            b'g' => Some(Self::GnssOn),
            b'h' => Some(Self::GnssOff),
            b'w' => Some(Self::Wake),
            b'f' => Some(Self::DumpInternalFlash),
            b's' => Some(Self::DumpMissionSd),
            b'x' => Some(Self::StopDump),
            // 旧r=ForceRecoveryBeaconはMission所有権に反するためNotSupportedへ落とす。
            _ => None,
        }
    }
}

async fn write_all(tx: &mut UartTx<'static, Async>, mut bytes: &[u8]) -> Result<bool, TxError> {
    while !bytes.is_empty() {
        let written = tx.write_async(bytes).await?;
        if written == 0 {
            println!("LoRa UART write made no progress");
            return Ok(false);
        }
        bytes = &bytes[written..];
    }
    Ok(true)
}

async fn wait_for_aux_high(aux_pin: &mut Input<'static>) -> bool {
    if aux_pin.is_high() {
        return true;
    }
    if with_timeout(
        Duration::from_millis(LORA_AUX_TIMEOUT_MS),
        aux_pin.wait_for_high(),
    )
    .await
    .is_ok()
        && aux_pin.is_high()
    {
        return true;
    }
    LORA_AUX_TIMEOUT_COUNT.fetch_add(1, Ordering::Relaxed);
    println!("LoRa AUX timeout");
    false
}

async fn wait_for_aux_low(aux_pin: &mut Input<'static>) -> bool {
    if aux_pin.is_low() {
        return true;
    }
    if with_timeout(
        Duration::from_millis(LORA_AUX_LOW_OBSERVE_TIMEOUT_MS),
        aux_pin.wait_for_low(),
    )
    .await
    .is_ok()
        && aux_pin.is_low()
    {
        return true;
    }
    LORA_AUX_TIMEOUT_COUNT.fetch_add(1, Ordering::Relaxed);
    println!("LoRa AUX Low observation timeout after UART flush");
    false
}

async fn wait_for_rx_guard() {
    let Some(mut last_rx) = LORA_RX_ACTIVITY_SIGNAL.try_take() else {
        return;
    };
    let guard_duration = Duration::from_millis(LORA_RX_TX_GUARD_MS);
    loop {
        let deadline = last_rx + guard_duration;
        if deadline <= Instant::now() {
            let Some(updated_rx) = LORA_RX_ACTIVITY_SIGNAL.try_take() else {
                return;
            };
            last_rx = updated_rx;
            continue;
        }
        match select(LORA_RX_ACTIVITY_SIGNAL.wait(), Timer::at(deadline)).await {
            Either::First(updated_rx) => last_rx = updated_rx,
            Either::Second(()) => {
                let Some(updated_rx) = LORA_RX_ACTIVITY_SIGNAL.try_take() else {
                    return;
                };
                last_rx = updated_rx;
            }
        }
    }
}

async fn prepare_transmit(
    aux_pin: &mut Input<'static>,
    #[cfg(feature = "lora-timing-debug")] timing: &mut TxTimingTrace,
) -> bool {
    #[cfg(feature = "lora-timing-debug")]
    {
        timing.transmit_started_at_us = Some(now_us());
    }
    if !wait_for_aux_high(aux_pin).await {
        return false;
    }
    #[cfg(feature = "lora-timing-debug")]
    {
        timing.initial_aux_ready_at_us = Some(now_us());
    }
    wait_for_rx_guard().await;
    #[cfg(feature = "lora-timing-debug")]
    {
        timing.rx_guard_done_at_us = Some(now_us());
    }
    if !wait_for_aux_high(aux_pin).await {
        return false;
    }
    #[cfg(feature = "lora-timing-debug")]
    {
        timing.post_guard_aux_ready_at_us = Some(now_us());
    }
    true
}

async fn transmit_frame(
    tx: &mut UartTx<'static, Async>,
    aux_pin: &mut Input<'static>,
    frame: LoraFrame,
    #[cfg(feature = "lora-timing-debug")] timing: &mut TxTimingTrace,
) -> bool {
    #[cfg(feature = "lora-timing-debug")]
    {
        timing.uart_write_started_at_us = Some(now_us());
    }
    // UART write開始後からAUX Low→High完了までは物理送信中のためpreemptできない。
    match write_all(tx, frame.as_bytes()).await {
        Ok(true) => {}
        Ok(false) => {
            LORA_TX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
            return false;
        }
        Err(error) => {
            LORA_TX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
            println!("LoRa UART write error: {:?}", error);
            return false;
        }
    }
    #[cfg(feature = "lora-timing-debug")]
    {
        timing.uart_write_finished_at_us = Some(now_us());
    }
    if let Err(error) = tx.flush_async().await {
        LORA_TX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
        println!("LoRa UART flush error: {:?}", error);
        return false;
    }
    #[cfg(feature = "lora-timing-debug")]
    {
        timing.uart_flush_finished_at_us = Some(now_us());
    }
    if !wait_for_aux_low(aux_pin).await {
        #[cfg(feature = "lora-timing-debug")]
        {
            timing.aux_low_not_observed = true;
        }
        LORA_TX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
        return false;
    }
    #[cfg(feature = "lora-timing-debug")]
    {
        timing.aux_low_at_us = Some(now_us());
    }
    if wait_for_aux_high(aux_pin).await {
        #[cfg(feature = "lora-timing-debug")]
        {
            let completed_at_us = now_us();
            timing.aux_high_at_us = Some(completed_at_us);
            timing.completed_at_us = Some(completed_at_us);
        }
        LORA_TX_SUCCESS_COUNT.fetch_add(1, Ordering::Relaxed);
        true
    } else {
        LORA_TX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
        false
    }
}

fn stale_or<T: Copy>(
    latest: crate::can::cache::Latest<T>,
    now_ms: u64,
    maximum_age_ms: u64,
) -> Option<T> {
    if latest.freshness(now_ms, maximum_age_ms) == Freshness::Fresh {
        latest.value()
    } else {
        None
    }
}

fn age_tenths(now_ms: u64, received_at_ms: Option<u64>) -> u16 {
    let Some(received_at_ms) = received_at_ms else {
        return 0xffff;
    };
    if now_ms < received_at_ms {
        return 0xffff;
    }
    ((now_ms - received_at_ms) / 100).min(0xfffe) as u16
}

fn latest_mission_periodic_at(cache: &crate::can::cache::CanCache) -> Option<u64> {
    [
        cache.kinematics.received_at_ms(),
        cache.control.received_at_ms(),
        cache.mission_status.received_at_ms(),
        cache.power_time.received_at_ms(),
        cache.descent_core.received_at_ms(),
        cache.attitude_tilt.received_at_ms(),
        cache.lps.received_at_ms(),
        cache.airspeed.received_at_ms(),
        cache.control_roll_v2.received_at_ms(),
    ]
    .into_iter()
    .flatten()
    .max()
}

const fn gnss_state_code(state: GnssReceiverState) -> u8 {
    match state {
        GnssReceiverState::Off => 0,
        GnssReceiverState::Starting => 1,
        GnssReceiverState::ReceiverDetected => 2,
        GnssReceiverState::ConfigurationFailed => 3,
        GnssReceiverState::ReceiverError => 4,
        GnssReceiverState::NoFix => 5,
        GnssReceiverState::ValidFix => 6,
        GnssReceiverState::InvalidSample => 7,
        GnssReceiverState::Stale => 8,
    }
}

fn fallback_can_health() -> u8 {
    CAN_FALLBACK_HEALTH.load(Ordering::Relaxed)
}

const fn fallback_primary_loss_reason(
    can_health: u8,
    invalid_is_latest: bool,
    mission_ever: bool,
    mission_age: u16,
    periodic_ever: bool,
    any_periodic_age: u16,
) -> u8 {
    if can_health == 4 {
        3 // CAN_BUS_OFF
    } else if can_health == 5 {
        4 // CAN_RECOVERING
    } else if can_health == 6 {
        7 // UNKNOWN。controller errorはcan_healthで識別する。
    } else if invalid_is_latest {
        5 // MISSION_STATUS_INVALID
    } else if !mission_ever {
        0 // STARTUP_WAITING
    } else if mission_age != 0xffff && mission_age < 10 {
        // 300 ms以上1 s未満はageとstatus bit2でMISSION STATUS LATEを表す。
        7 // UNKNOWN
    } else if periodic_ever && any_periodic_age != 0xffff && any_periodic_age < 10 {
        1 // MISSION_STATUS_TIMEOUT
    } else {
        2 // NO_MISSION_TRAFFIC
    }
}

const fn fallback_can_status_flags(can_health: u8) -> u16 {
    let mut flags = 0;
    if can_health >= 1 && can_health <= 3 {
        flags |= 1 << 13;
    }
    if can_health >= 2 && can_health <= 6 {
        flags |= 1 << 14;
    }
    flags
}

fn mission_link_fallback_packet(
    cache: &crate::can::cache::CanCache,
    now_ms: u64,
    gnss: crate::state::GnssTelemetry,
) -> ApplicationPacket {
    mission_link_fallback_packet_with_health(cache, now_ms, gnss, fallback_can_health())
}

fn mission_link_fallback_packet_with_health(
    cache: &crate::can::cache::CanCache,
    now_ms: u64,
    gnss: crate::state::GnssTelemetry,
    can_health: u8,
) -> ApplicationPacket {
    let mission_received = cache.mission_status.received_at_ms();
    let any_periodic_received = latest_mission_periodic_at(cache);
    let power_received = cache.power_time.received_at_ms();
    let mission_age = age_tenths(now_ms, mission_received);
    let any_periodic_age = age_tenths(now_ms, any_periodic_received);
    let power_age = age_tenths(now_ms, power_received);
    let mission_ever = mission_received.is_some();
    let periodic_ever = any_periodic_received.is_some();
    let invalid_at = MISSION_STATUS_INVALID_AT_MS.load(Ordering::Relaxed) as u64;
    let invalid_is_latest = invalid_at != 0 && mission_received.is_none_or(|valid_at| invalid_at > valid_at);
    let primary_loss_reason = fallback_primary_loss_reason(
        can_health,
        invalid_is_latest,
        mission_ever,
        mission_age,
        periodic_ever,
        any_periodic_age,
    );
    let mut flags = 0u16;
    flags |= u16::from(mission_ever) << 0;
    flags |= u16::from(periodic_ever) << 1;
    flags |= u16::from(mission_age != 0xffff && mission_age < 10) << 2;
    flags |= u16::from(any_periodic_age != 0xffff && any_periodic_age < 10) << 3;
    flags |= u16::from(!SD_HAS_ERROR.load(Ordering::Relaxed)) << 4;
    flags |= u16::from(LOGGING_REQUESTED.load(Ordering::Relaxed)) << 5;
    flags |= u16::from(LOGGING_ACTIVE.load(Ordering::Relaxed)) << 6;
    flags |= u16::from(HAS_UNFLUSHED_DATA.load(Ordering::Relaxed)) << 7;
    flags |= u16::from(gnss.state != GnssReceiverState::Off) << 8;
    flags |= u16::from(gnss.state == GnssReceiverState::ValidFix) << 9;
    flags |= u16::from(gnss.state == GnssReceiverState::Stale) << 10;
    flags |= u16::from(power_received.is_some()) << 11;
    flags |= u16::from(mission_ever) << 12;
    flags |= fallback_can_status_flags(can_health);

    let last_state = cache.mission_status.value().map_or(0xff, |status| status.state as u8);
    let power = cache.power_time.value();
    ApplicationPacket::MissionLinkFallbackTelemetry(MissionLinkFallbackTelemetry {
        sequence: MISSION_LINK_FALLBACK_SEQUENCE.fetch_add(1, Ordering::Relaxed),
        primary_loss_reason,
        status_flags: flags,
        last_valid_mission_state: last_state,
        gnss_state: gnss_state_code(gnss.state),
        mission_status_age: mission_age,
        any_mission_periodic_age: any_periodic_age,
        power_time_age: power_age,
        east: gnss.east,
        north: gnss.north,
        height: gnss.height,
        logic_voltage: power.map_or(253, |value| value.logic_voltage),
        motor_voltage: power.map_or(253, |value| value.motor_voltage),
        can_health,
    })
}

struct PeriodicPacket {
    frame: LoraFrame,
    transmitted_events: u16,
    event_revision: u32,
    is_control_roll_v2: bool,
    control_roll_cycle_available: bool,
}

async fn periodic_packet(
    prefer_control_roll_v2: bool,
    control_roll_event_only: bool,
) -> Option<PeriodicPacket> {
    let now_ms = Instant::now().as_millis();
    let cache = *CAN_CACHE.lock().await;
    let (interval_events, event_revision) = cache.event_snapshot();
    let gnss = *GNSS_TELEMETRY.lock().await;
    let mission = stale_or(cache.mission_status, now_ms, FRESHNESS_10_HZ_MS);
    let state = mission.map_or(MissionState::Unknown, |value| value.state);
    let kinematics = stale_or(cache.kinematics, now_ms, FRESHNESS_100_HZ_MS);
    let control = stale_or(cache.control, now_ms, FRESHNESS_100_HZ_MS);
    let tilt = stale_or(cache.attitude_tilt, now_ms, FRESHNESS_10_HZ_MS);
    let lps = stale_or(cache.lps, now_ms, FRESHNESS_25_HZ_MS);
    let airspeed = stale_or(cache.airspeed, now_ms, FRESHNESS_100_HZ_MS);
    let power = stale_or(cache.power_time, now_ms, FRESHNESS_10_HZ_MS);
    let control_roll_v2 = stale_or(cache.control_roll_v2, now_ms, FRESHNESS_10_HZ_MS);
    let flight_state = matches!(
        state,
        MissionState::LiftoffDetection | MissionState::EngineBurn | MissionState::Control
    );
    let control_roll_cycle_available = flight_state && control_roll_v2.is_some();
    if prefer_control_roll_v2
        && control_roll_cycle_available
        && let Some(value) = control_roll_v2
    {
        let packet = ApplicationPacket::ControlRollTelemetryV2(ControlRollTelemetryV2Packet {
            control_roll_reference_unwrapped_raw: value.control_roll_reference_unwrapped_raw,
            roll_deviation_unwrapped_raw: value.roll_deviation_unwrapped_raw,
            flags: value.flags,
            reference_capture_event_sequence: value.reference_capture_event_sequence,
        });
        return finish_periodic_packet(packet, 0, event_revision).map(
            |(frame, transmitted_events, event_revision)| PeriodicPacket {
                frame,
                transmitted_events,
                event_revision,
                is_control_roll_v2: true,
                control_roll_cycle_available: true,
            },
        );
    }
    if control_roll_event_only {
        // event駆動経路はA7専用とし、A8を含む通常packetは500 ms周期でのみ生成する。
        return None;
    }

    // RecoveryBeaconはMissionから0x014を受けた場合だけlatchedされる。
    if RECOVERY_BEACON_ACTIVE.load(Ordering::Relaxed) {
        let packet = ApplicationPacket::RecoveryBeacon(RecoveryBeacon {
            logic_voltage: power.map_or(253, |value| value.logic_voltage),
            motor_voltage: power.map_or(253, |value| value.motor_voltage),
            east: gnss.east,
            north: gnss.north,
            height: gnss.height,
            elapsed: power.map_or(0xfffa, |value| value.recovery_elapsed),
        });
        return finish_periodic_packet(packet, 0, event_revision).map(
            |(frame, transmitted_events, event_revision)| PeriodicPacket {
                frame,
                transmitted_events,
                event_revision,
                is_control_roll_v2: false,
                control_roll_cycle_available: false,
            },
        );
    }

    if state == MissionState::Unknown {
        let packet = mission_link_fallback_packet(&cache, now_ms, gnss);
        return finish_periodic_packet(packet, 0, event_revision).map(
            |(frame, transmitted_events, event_revision)| PeriodicPacket {
                frame,
                transmitted_events,
                event_revision,
                is_control_roll_v2: false,
                control_roll_cycle_available: false,
            },
        );
    }

    let packet = match state {
        MissionState::CommandReceive => {
            let mission_status = mission.map_or(0, |value| value.status);
            let config = mission.map_or(0, |value| value.config);
            let mut status = map_command_receive_status(
                mission_status,
                config,
                power,
                lps.is_some_and(|value| value.pressure <= 2031 && value.temperature <= 200),
                airspeed.is_some_and(|value| value.airspeed <= 245),
                mission.is_some(),
            );
            if !SD_HAS_ERROR.load(Ordering::Relaxed) {
                status |= 1 << 11;
            } else {
                status &= !(1 << 11);
            }
            if !IS_CAN_ERROR.load(Ordering::Relaxed) {
                status |= 1 << 12;
            } else {
                status &= !(1 << 12);
            }
            ApplicationPacket::CommandReceive(CommandReceiveTelemetry {
                status,
                motor_profile: 0xff,
                tilt_magnitude: tilt.map_or(123, |value| value.magnitude),
                tilt_direction: tilt.map_or(0, |value| value.direction),
                fin_mode: mission.map_or(15, |value| value.fin_mode as u8),
                para_mode: mission.map_or(15, |value| value.para_mode as u8),
                fin_angle: kinematics.map_or(249, |value| value.fin_angle),
                para_angle: mission.map_or(247, |value| value.para_angle),
                pressure: lps.map_or(2039, |value| value.pressure),
                temperature: lps.map_or(247, |value| value.temperature),
                airspeed: airspeed.map_or(252, |value| value.airspeed),
                logic_voltage: power.map_or(253, |value| value.logic_voltage),
                motor_voltage: power.map_or(253, |value| value.motor_voltage),
                east: gnss.east,
                north: gnss.north,
                height: gnss.height,
                display_roll: kinematics.map_or(0x8004, |value| value.roll),
            })
        }
        MissionState::LiftoffDetection | MissionState::EngineBurn | MissionState::Control => {
            let header = match state {
                MissionState::LiftoffDetection => PacketHeader::LiftoffDetection,
                MissionState::EngineBurn => PacketHeader::EngineBurn,
                _ => PacketHeader::Control,
            };
            let mut status = mission.map_or(0, |value| value.status);
            status = inject_link_health(status, 7, 8);
            status |= map_flight_interval_events(interval_events);
            ApplicationPacket::Flight(FlightTelemetry {
                header,
                status,
                roll: kinematics.map_or(0x8004, |value| value.roll),
                roll_rate: kinematics.map_or(0x8004, |value| value.roll_rate),
                tilt_magnitude: tilt.map_or(123, |value| value.magnitude),
                tilt_direction: tilt.map_or(0, |value| value.direction),
                fin_angle: kinematics.map_or(249, |value| value.fin_angle),
                fin_rate: kinematics.map_or(0x8002, |value| value.fin_rate),
                pressure: lps.map_or(2039, |value| value.pressure),
                temperature: lps.map_or(247, |value| value.temperature),
                airspeed: airspeed.map_or(252, |value| value.airspeed),
                requested_torque: control.map_or(0x800, |value| value.requested_torque),
                elapsed: control.map_or(0xfa, |value| value.elapsed),
                east: gnss.east,
                north: gnss.north,
                height: gnss.height,
            })
        }
        MissionState::Descent => {
            let descent = stale_or(cache.descent_core, now_ms, FRESHNESS_100_HZ_MS);
            // 最新Vaultではbit0..3=ParachuteDeploymentFailureCode、bit4=persistence corrupt、bit5..12=reserved。
            let status = descent.map_or(0, |value| value.status) & 0x001f;
            ApplicationPacket::Descent(DescentTelemetry {
                status,
                pressure: lps.map_or(2039, |value| value.pressure),
                temperature: lps.map_or(247, |value| value.temperature),
                para_angle: descent.map_or(247, |value| value.para_angle),
                elapsed: power.map_or(0xfffa, |value| value.descent_elapsed),
                east: gnss.east,
                north: gnss.north,
                height: gnss.height,
            })
        }
        MissionState::Unknown => unreachable!(),
    };
    let transmitted_events = if matches!(
        state,
        MissionState::LiftoffDetection | MissionState::EngineBurn | MissionState::Control
    ) {
        interval_events & 0x001f
    } else {
        0
    };
    finish_periodic_packet(packet, transmitted_events, event_revision).map(
        |(frame, transmitted_events, event_revision)| PeriodicPacket {
            frame,
            transmitted_events,
            event_revision,
            is_control_roll_v2: false,
            control_roll_cycle_available,
        },
    )
}

fn finish_periodic_packet(
    packet: ApplicationPacket,
    interval_events: u16,
    event_revision: u32,
) -> Option<(LoraFrame, u16, u32)> {
    match packet.encode() {
        Ok(frame) => Some((frame, interval_events, event_revision)),
        Err(error) => {
            println!("periodic LoRa packet encode error: {:?}", error);
            None
        }
    }
}

fn inject_link_health(mut status: u16, sd_bit: u8, can_bit: u8) -> u16 {
    if !SD_HAS_ERROR.load(Ordering::Relaxed) {
        status |= 1 << sd_bit;
    } else {
        status &= !(1 << sd_bit);
    }
    if !IS_CAN_ERROR.load(Ordering::Relaxed) {
        status |= 1 << can_bit;
    } else {
        status &= !(1 << can_bit);
    }
    status
}

fn map_command_receive_status(
    mission_status: u16,
    config: u8,
    power: Option<crate::can::protocol::PowerTimeTelemetry>,
    lps_fresh: bool,
    airspeed_fresh: bool,
    mission_status_fresh: bool,
) -> u32 {
    let mut status = 0u32;
    status |= u32::from(mission_status & (1 << 2) != 0);
    status |= u32::from(lps_fresh) << 1;
    status |= u32::from(airspeed_fresh) << 2;
    status |= u32::from(mission_status_fresh && mission_status & (1 << 10) == 0) << 3;
    status |= u32::from(mission_status & (1 << 3) != 0) << 4;
    status |= u32::from(config & (1 << 0) != 0) << 5;
    status |= u32::from(config & (1 << 1) != 0) << 6;
    status |= u32::from(config & (1 << 2) != 0) << 7;
    if let Some(power) = power {
        status |= u32::from(power.logic_voltage <= 240) << 8;
        status |= u32::from(power.motor_voltage <= 240) << 9;
        status |= u32::from(power.flags & (1 << 2) != 0) << 10;
        status |= u32::from(power.flags & (1 << 0) != 0) << 13;
        // PowerTime bit5/6は最新Vaultでreserved。Flash backup bitへ転用しない。
    }
    status |= u32::from(config & (1 << 3) != 0) << 16;
    status |= u32::from(config & (1 << 4) != 0) << 18;
    status |= u32::from(config & (1 << 5) != 0) << 21;
    status |= u32::from(config & (1 << 6) != 0) << 22;
    status
}

fn map_flight_interval_events(events: u16) -> u16 {
    let mut status = 0u16;
    status |= u16::from(events & (1 << 0) != 0) << 9;
    status |= u16::from(events & (1 << 1) != 0) << 10;
    status |= u16::from(events & (1 << 2) != 0) << 11;
    status |= u16::from(events & (1 << 3) != 0) << 12;
    status |= u16::from(events & (1 << 4) != 0) << 14;
    status
}

const fn periodic_interval_ms(recovery_active: bool, control_roll_cycle_active: bool) -> u64 {
    if recovery_active {
        10_000
    } else if control_roll_cycle_active {
        LORA_TRANSMIT_INTERVAL_MS / 2
    } else {
        LORA_TRANSMIT_INTERVAL_MS
    }
}

const RECOVERY_LOG_INTERVAL_MS: u64 = 200;

fn add_periodic_missed_slots(value: u32) {
    if value == 0 {
        return;
    }
    let _ = LORA_PERIODIC_MISSED_SLOT_COUNT.fetch_update(
        Ordering::Relaxed,
        Ordering::Relaxed,
        |current| Some(current.saturating_add(value)),
    );
}

fn skip_overdue_periodic_deadlines(next_tx_at: &mut Instant, interval: Duration) -> Option<u32> {
    let advance = advance_deadline(
        next_tx_at.as_micros(),
        Instant::now().as_micros(),
        interval.as_micros(),
    )?;
    *next_tx_at = Instant::from_micros(advance.next_deadline_us);
    add_periodic_missed_slots(advance.missed_slots);
    Some(advance.missed_slots)
}

fn take_higher_priority_envelope(
    selected_source: LoRaTxSource,
    recovery_sent_since_periodic: bool,
    next_recovery_at: Instant,
    periodic_deadline_valid: bool,
    next_periodic_at: Instant,
    pending_recovery: &mut Option<LoRaTxEnvelope>,
) -> Option<LoRaTxEnvelope> {
    if is_higher_priority(LoRaTxSource::EmergencyResult, selected_source)
        && let Ok(envelope) = EMERGENCY_RESULT_LORA_CHANNEL.try_receive()
    {
        return Some(envelope);
    }
    if is_higher_priority(LoRaTxSource::CommandResult, selected_source)
        && let Ok(envelope) = COMMAND_RESULT_LORA_CHANNEL.try_receive()
    {
        return Some(envelope);
    }
    if is_higher_priority(LoRaTxSource::GroundTimeRequest, selected_source)
        && let Ok(envelope) = GROUND_TIME_REQUEST_LORA_CHANNEL.try_receive()
    {
        return Some(envelope);
    }
    let now = Instant::now();
    let periodic_due = periodic_due_for_reselection(
        selected_source,
        periodic_deadline_valid,
        now >= next_periodic_at,
    );
    if is_higher_priority(LoRaTxSource::Recovery, selected_source)
        && recovery_allowed(recovery_sent_since_periodic, periodic_due)
        && now >= next_recovery_at
    {
        if let Some(envelope) = pending_recovery.take() {
            return Some(envelope);
        }
        if let Ok(envelope) = RECOVERY_LORA_CHANNEL.try_receive() {
            return Some(envelope);
        }
    }
    None
}

fn queue_command_result(packet: CommandResultPacket) {
    let Ok(frame) = ApplicationPacket::CommandResult(packet).encode() else {
        println!("CommandResult encode error");
        return;
    };
    if COMMAND_RESULT_LORA_CHANNEL
        .try_send(LoRaTxEnvelope::queued(
            frame,
            LoRaTxSource::CommandResult,
            Instant::now().as_micros(),
        ))
        .is_err()
    {
        LORA_TX_QUEUE_DROP_COUNT.fetch_add(1, Ordering::Relaxed);
        println!("CommandResult LoRa queue overflow");
    }
}

fn queue_result(transaction_id: u8, command: u8, phase: CommandPhase, reason: CommandReason) {
    queue_command_result(CommandResultPacket {
        transaction_id,
        command,
        phase,
        reason,
        detail: 0,
    });
}

async fn process_generic(request: GenericCommandRequest) {
    match COMMAND_TRACKER.lock().await.register(request) {
        RegisterResult::Forward => {
            CAN_TX_CHANNEL
                .send(CanTxRequest {
                    message: CanTxMessage::GenericCommandRequest(request),
                })
                .await;
        }
        RegisterResult::DuplicatePending => {}
        RegisterResult::Replay(result) => {
            queue_command_result(CommandResultPacket {
                transaction_id: result.transaction_id,
                command: result.command,
                phase: result.phase,
                reason: result.reason,
                detail: result.detail,
            });
        }
        RegisterResult::ProtocolError => {
            queue_result(
                request.transaction_id,
                request.command,
                CommandPhase::Rejected,
                CommandReason::ProtocolError,
            );
        }
        RegisterResult::Busy => {
            queue_result(
                request.transaction_id,
                request.command,
                CommandPhase::Rejected,
                CommandReason::Busy,
            );
        }
    }
}

async fn process_local(transaction_id: u8, command: u8, args: [u8; 6]) {
    let Some(command_value) = LocalCommand::decode(command) else {
        queue_result(
            transaction_id,
            command,
            CommandPhase::Rejected,
            CommandReason::NotSupported,
        );
        return;
    };
    let recovery_dump = matches!(
        command_value,
        LocalCommand::DumpInternalFlash | LocalCommand::DumpMissionSd
    );
    if !recovery_dump && args.iter().any(|value| *value != 0) {
        queue_result(
            transaction_id,
            command,
            CommandPhase::Rejected,
            CommandReason::InvalidArgument,
        );
        return;
    }
    match command_value {
        LocalCommand::StartLogging => LOGGING_REQUESTED.store(true, Ordering::Relaxed),
        LocalCommand::StopLogging => {
            LOGGING_REQUESTED.store(false, Ordering::Relaxed);
            SD_FLUSH_SIGNAL.signal(());
        }
        LocalCommand::GnssOn => GNSS_CMD_CHANNEL.send(GnssCommand::TurnOn).await,
        LocalCommand::GnssOff => GNSS_CMD_CHANNEL.send(GnssCommand::TurnOff).await,
        LocalCommand::Wake
        | LocalCommand::DumpInternalFlash
        | LocalCommand::DumpMissionSd
        | LocalCommand::StopDump => {
            let opcode = match command_value {
                LocalCommand::Wake => RecoveryOpcode::Wake,
                LocalCommand::DumpInternalFlash | LocalCommand::DumpMissionSd => {
                    RecoveryOpcode::StartLogDump
                }
                LocalCommand::StopDump => RecoveryOpcode::StopLogDump,
                _ => unreachable!(),
            };
            let source = if command_value == LocalCommand::DumpMissionSd {
                RecoverySource::MissionSdLatestFlight
            } else if command_value == LocalCommand::StopDump {
                RECOVERY_SESSION
                    .lock()
                    .await
                    .active_source()
                    .unwrap_or(RecoverySource::InternalFlash)
            } else {
                RecoverySource::InternalFlash
            };
            let offset = u32::from_le_bytes([args[0], args[1], args[2], 0]);
            let length = u32::from_le_bytes([args[3], args[4], args[5], 0]);
            let pending = PendingRecovery {
                transaction_id,
                command,
                opcode,
                source,
            };
            let interrupted = if command_value == LocalCommand::StopDump {
                RECOVERY_SESSION.lock().await.interrupt_with_stop(pending)
            } else if RECOVERY_SESSION.lock().await.start(pending) {
                None
            } else {
                queue_result(
                    transaction_id,
                    command,
                    CommandPhase::Rejected,
                    CommandReason::Busy,
                );
                return;
            };
            if let Some(result) = interrupted {
                queue_result(
                    result.transaction_id,
                    result.command,
                    result.phase,
                    result.reason,
                );
            }
            if command_value == LocalCommand::StopDump {
                let _ = RECOVERY_ASSEMBLER.lock().await.abort();
            }
            if opcode == RecoveryOpcode::StartLogDump {
                RECOVERY_ASSEMBLER
                    .lock()
                    .await
                    .start(transaction_id, source, offset, length);
            }
            CAN_TX_CHANNEL
                .send(CanTxRequest {
                    message: CanTxMessage::RecoveryControl(RecoveryControl {
                        opcode,
                        source,
                        transfer_id: transaction_id,
                        offset,
                        length,
                    }),
                })
                .await;
            queue_result(
                transaction_id,
                command,
                CommandPhase::Accepted,
                CommandReason::None,
            );
            return;
        }
    }
    queue_result(
        transaction_id,
        command,
        CommandPhase::Completed,
        CommandReason::None,
    );
}

#[embassy_executor::task]
pub async fn command_process_task() {
    loop {
        match UPLINK_COMMAND_CHANNEL.receive().await {
            UplinkCommand::MissionGeneric(request) => process_generic(request).await,
            UplinkCommand::ActuatorEmergency { .. }
            | UplinkCommand::LiftoffDetectionEmergency { .. } => {}
            UplinkCommand::ComBoardLocal {
                transaction_id,
                command,
                args,
            } => process_local(transaction_id, command, args).await,
            UplinkCommand::GroundTimeResponse {
                request_id,
                source,
                unix_seconds,
                milliseconds,
            } => {
                CAN_TX_CHANNEL
                    .send(CanTxRequest {
                        message: CanTxMessage::TimeResponse {
                            request_id,
                            source,
                            unix_seconds,
                            milliseconds,
                        },
                    })
                    .await;
            }
        }
    }
}

#[embassy_executor::task]
pub async fn lora_rx_task(mut rx: UartRx<'static, Async>) {
    let mut rx_buf = [0u8; 32];
    let mut uplink_frame = UplinkFrameBuffer::new();
    loop {
        match rx.read_async(&mut rx_buf).await {
            Ok(length) => {
                LORA_RX_BYTE_COUNT.fetch_add(length as u32, Ordering::Relaxed);
                if length != 0 {
                    LORA_RX_ACTIVITY_SIGNAL.signal(Instant::now());
                }
                for byte in &rx_buf[..length] {
                    if let Some(result) = uplink_frame.push(*byte) {
                        LORA_RX_ACTIVITY_SIGNAL.signal(Instant::now());
                        match result {
                            Ok(command) => {
                                LORA_RX_SUCCESS_COUNT.fetch_add(1, Ordering::Relaxed);
                                match command {
                                    UplinkCommand::ActuatorEmergency { transaction_id } => {
                                        CAN_SAFETY_TX_CHANNEL
                                            .send(CanTxRequest {
                                                message: CanTxMessage::ActuatorEmergencyStop {
                                                    transaction_id,
                                                },
                                            })
                                            .await;
                                    }
                                    UplinkCommand::LiftoffDetectionEmergency { transaction_id } => {
                                        CAN_SAFETY_TX_CHANNEL
                                            .send(CanTxRequest {
                                                message: CanTxMessage::LiftoffDetectionEmergencyStop {
                                                    transaction_id,
                                                },
                                            })
                                            .await;
                                    }
                                    command => {
                                        if UPLINK_COMMAND_CHANNEL.try_send(command).is_err() {
                                            LORA_COMMAND_DROP_COUNT.fetch_add(1, Ordering::Relaxed);
                                        }
                                    }
                                }
                            }
                            Err(error) => {
                                LORA_RX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
                                println!("invalid LoRa uplink: {:?}", error);
                            }
                        }
                    }
                }
            }
            Err(error) => {
                LORA_RX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
                uplink_frame.reset();
                match error {
                    RxError::FifoOverflowed => println!("LoRa UART RX FIFO overflowed"),
                    _ => println!("LoRa UART receive error: {:?}", error),
                }
            }
        }
    }
}

#[embassy_executor::task]
pub async fn lora_tx_task(mut tx: UartTx<'static, Async>, mut aux_pin: Input<'static>) {
    let mut control_roll_cycle_active = false;
    let mut prefer_control_roll_v2 = false;
    let mut interval = Duration::from_millis(periodic_interval_ms(false, false));
    let (mut next_tx_at, mut periodic_deadline_valid) = match Instant::now().checked_add(interval) {
        Some(deadline) => (deadline, true),
        None => (Instant::MAX, false),
    };
    let mut next_recovery_at = Instant::now();
    let mut pending_recovery = None;
    let mut deferred_envelope = None;
    let mut periodic_retry_at = None;
    let mut recovery_sent_since_periodic = false;
    #[cfg(feature = "lora-timing-debug")]
    let mut timing_collector = TimingCollector::new();
    loop {
        let periodic_selection = select_periodic_deadline(
            next_tx_at.as_micros(),
            periodic_deadline_valid,
            periodic_retry_at.map(|retry_at: Instant| retry_at.as_micros()),
        );
        let scheduled_periodic_at =
            Instant::try_from_micros(periodic_selection.deadline_us).unwrap_or(Instant::MAX);
        let periodic_schedule_valid = if periodic_selection.retry_selected {
            periodic_retry_at.is_some()
        } else {
            periodic_deadline_valid
        };
        let periodic_due = periodic_schedule_valid && Instant::now() >= scheduled_periodic_at;
        let deferred_blocked = deferred_envelope.is_some_and(|envelope: LoRaTxEnvelope| {
            deferred_recovery_blocked(envelope.source, recovery_sent_since_periodic, periodic_due)
        });
        let deferred_candidate = if deferred_blocked {
            None
        } else {
            deferred_envelope.take()
        };
        let mut regular_periodic_selected = false;
        let mut envelope = if let Some(deferred) = deferred_candidate {
            deferred
        } else {
            let had_pending_recovery = pending_recovery.is_some();
            let recovery_ready = async {
                if !recovery_allowed(recovery_sent_since_periodic, periodic_due) {
                    core::future::pending::<LoRaTxEnvelope>().await;
                }
                if let Some(frame) = pending_recovery {
                    Timer::at(next_recovery_at).await;
                    frame
                } else {
                    RECOVERY_LORA_CHANNEL.receive().await
                }
            };
            let periodic_ready = async move {
                if !periodic_schedule_valid {
                    // 表現可能な未来時刻がない場合はready-loopせず、periodicだけを停止する。
                    core::future::pending::<()>().await;
                }
                Timer::at(scheduled_periodic_at).await;
            };
            let control_roll_or_periodic_ready =
                select(periodic_ready, CONTROL_ROLL_LORA_SIGNAL.wait());
            // select5は左からpollするため、この並びが送信優先度になる。
            match select5(
                EMERGENCY_RESULT_LORA_CHANNEL.receive(),
                COMMAND_RESULT_LORA_CHANNEL.receive(),
                GROUND_TIME_REQUEST_LORA_CHANNEL.receive(),
                recovery_ready,
                control_roll_or_periodic_ready,
            )
            .await
            {
                Either5::First(envelope) | Either5::Second(envelope) | Either5::Third(envelope) => {
                    envelope
                }
                Either5::Fourth(envelope) => {
                    let now = Instant::now();
                    let recovery_ready = had_pending_recovery || now >= next_recovery_at;
                    let periodic_due = periodic_schedule_valid && now >= scheduled_periodic_at;
                    if recovery_ready
                        && recovery_allowed(recovery_sent_since_periodic, periodic_due)
                    {
                        pending_recovery = None;
                        envelope
                    } else {
                        pending_recovery = Some(envelope);
                        continue;
                    }
                }
                Either5::Fifth(Either::First(())) => {
                    interval = Duration::from_millis(periodic_interval_ms(
                        RECOVERY_BEACON_ACTIVE.load(Ordering::Relaxed),
                        control_roll_cycle_active,
                    ));
                    regular_periodic_selected = !periodic_selection.retry_selected;
                    let consumption = consume_periodic_selection(
                        next_tx_at.as_micros(),
                        periodic_selection.retry_selected,
                        interval.as_micros(),
                    );
                    // regular/retryどちらのPeriodic試行でもone-shot retryは消費する。
                    periodic_retry_at = consumption.retry_at_us.and_then(Instant::try_from_micros);
                    if regular_periodic_selected {
                        match consumption
                            .next_regular_deadline_us
                            .and_then(Instant::try_from_micros)
                        {
                            Some(deadline) => next_tx_at = deadline,
                            None => periodic_deadline_valid = false,
                        }
                    }
                    let requested_at_us = Instant::now().as_micros();
                    let Some(frame) = periodic_packet(prefer_control_roll_v2, false).await else {
                        recovery_sent_since_periodic = update_recovery_fairness(
                            recovery_sent_since_periodic,
                            LoRaTxSource::Periodic,
                        );
                        if periodic_deadline_valid
                            && skip_overdue_periodic_deadlines(&mut next_tx_at, interval).is_none()
                        {
                            periodic_deadline_valid = false;
                        }
                        continue;
                    };
                    control_roll_cycle_active = frame.control_roll_cycle_available;
                    prefer_control_roll_v2 =
                        frame.control_roll_cycle_available && !frame.is_control_roll_v2;
                    LoRaTxEnvelope::periodic(
                        frame.frame,
                        requested_at_us,
                        frame.transmitted_events,
                        frame.event_revision,
                    )
                }
                Either5::Fifth(Either::Second(())) => {
                    let requested_at_us = Instant::now().as_micros();
                    let Some(frame) = periodic_packet(true, true).await else {
                        continue;
                    };
                    LoRaTxEnvelope::periodic(
                        frame.frame,
                        requested_at_us,
                        frame.transmitted_events,
                        frame.event_revision,
                    )
                }
            }
        };
        #[cfg(feature = "lora-timing-debug")]
        let mut timing = TxTimingTrace::new(envelope.requested_at_us, envelope.source);
        let mut preempted_missed_slots = 0u32;
        let mut recovery_displaced_regular = false;
        let prepared = prepare_transmit(
            &mut aux_pin,
            #[cfg(feature = "lora-timing-debug")]
            &mut timing,
        )
        .await;
        if prepared
            && let Some(higher_priority) = take_higher_priority_envelope(
                envelope.source,
                recovery_sent_since_periodic,
                next_recovery_at,
                periodic_schedule_valid,
                scheduled_periodic_at,
                &mut pending_recovery,
            )
        {
            if should_defer_preempted(envelope.source) {
                // queueへ戻さず1件だけlocal保持し、再投入失敗で失わない。
                deferred_envelope = Some(envelope);
            }
            // periodicはslotを消費済みなので保持せず、event flagを残して次deadlineで再生成する。
            recovery_displaced_regular = recovery_displaced_regular_slot(
                envelope.source,
                higher_priority.source,
                regular_periodic_selected,
            );
            preempted_missed_slots = if regular_periodic_selected {
                preempted_periodic_missed_slots(envelope.source)
            } else {
                0
            };
            add_periodic_missed_slots(preempted_missed_slots);
            envelope = higher_priority;
            #[cfg(feature = "lora-timing-debug")]
            {
                timing = reselect_prepared_trace(timing, envelope.requested_at_us, envelope.source);
            }
        }
        let transmitted = if prepared {
            transmit_frame(
                &mut tx,
                &mut aux_pin,
                envelope.frame,
                #[cfg(feature = "lora-timing-debug")]
                &mut timing,
            )
            .await
        } else {
            false
        };
        let attempt_completed_at_us = Instant::now().as_micros();
        periodic_retry_at = periodic_retry_after_nonperiodic_attempt(
            periodic_retry_at.map(|retry_at: Instant| retry_at.as_micros()),
            attempt_completed_at_us,
            envelope.source,
        )
        .and_then(Instant::try_from_micros);
        let recovery_completed_at_us =
            (envelope.source == LoRaTxSource::Recovery).then_some(attempt_completed_at_us);
        if let Some(completed_at_us) = recovery_completed_at_us {
            next_recovery_at = recovery_next_eligible_at_us(
                completed_at_us,
                Duration::from_millis(RECOVERY_LOG_INTERVAL_MS).as_micros(),
            )
            .and_then(Instant::try_from_micros)
            .unwrap_or(Instant::MAX);
        }
        recovery_sent_since_periodic =
            update_recovery_fairness(recovery_sent_since_periodic, envelope.source);
        let overdue_missed_slots = if periodic_deadline_valid {
            match skip_overdue_periodic_deadlines(&mut next_tx_at, interval) {
                Some(missed_slots) => missed_slots,
                None => {
                    periodic_deadline_valid = false;
                    0
                }
            }
        } else {
            0
        };
        let recovery_displaced_periodic = recovery_displaced_regular
            || recovery_completed_at_us.is_some_and(|_| overdue_missed_slots != 0);
        if recovery_displaced_periodic && let Some(completed_at_us) = recovery_completed_at_us {
            periodic_retry_at = periodic_retry_after_recovery_us(
                completed_at_us,
                Duration::from_millis(RECOVERY_LOG_INTERVAL_MS).as_micros(),
                true,
            )
            .and_then(Instant::try_from_micros);
        }
        let _missed_slots = preempted_missed_slots.saturating_add(overdue_missed_slots);
        #[cfg(feature = "lora-timing-debug")]
        {
            timing.periodic_missed_slots = _missed_slots;
            if let Some(report) = timing_collector.record(timing) {
                LORA_TIMING_REPORT_SIGNAL.signal(report);
            }
        }
        if should_clear_periodic_events(envelope.source, transmitted, envelope.interval_events) {
            CAN_CACHE
                .lock()
                .await
                .clear_event_flags(envelope.interval_events, envelope.event_revision);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::can::cache::CanCache;

    #[test]
    fn recovery_beacon_uses_ten_second_interval() {
        assert_eq!(periodic_interval_ms(false, false), 500);
        assert_eq!(periodic_interval_ms(false, true), 250);
        assert_eq!(periodic_interval_ms(true, true), 10_000);
    }

    #[test]
    fn link_health_bits_are_injected_without_changing_other_bits() {
        SD_HAS_ERROR.store(false, Ordering::Relaxed);
        IS_CAN_ERROR.store(false, Ordering::Relaxed);
        assert_eq!(inject_link_health(1, 7, 8), 0x0181);
    }

    #[test]
    fn fallback_age_uses_tenth_seconds_and_saturates() {
        assert_eq!(age_tenths(1_300, Some(1_000)), 3);
        assert_eq!(age_tenths(1_000, None), 0xffff);
        assert_eq!(age_tenths(1, Some(2)), 0xffff);
        assert_eq!(age_tenths(u64::MAX, Some(0)), 0xfffe);
    }

    #[test]
    fn missing_mission_status_is_startup_fallback() {
        let cache = CanCache::new();
        let gnss = crate::state::GnssTelemetry::new();
        MISSION_STATUS_INVALID_AT_MS.store(0, Ordering::Relaxed);
        let packet = mission_link_fallback_packet_with_health(&cache, 1_000, gnss, 1);
        let ApplicationPacket::MissionLinkFallbackTelemetry(value) = packet else {
            panic!("expected fallback telemetry");
        };
        assert_eq!(value.primary_loss_reason, 0);
        assert_eq!(value.last_valid_mission_state, 0xff);
        assert_eq!(value.mission_status_age, 0xffff);
        assert_ne!(value.status_flags & (1 << 13), 0);
        assert_eq!(value.status_flags & (1 << 14), 0);
    }

    #[test]
    fn fallback_reports_recovering_and_controller_error() {
        let cache = CanCache::new();
        let gnss = crate::state::GnssTelemetry::new();
        MISSION_STATUS_INVALID_AT_MS.store(0, Ordering::Relaxed);

        let ApplicationPacket::MissionLinkFallbackTelemetry(recovering) =
            mission_link_fallback_packet_with_health(&cache, 1_000, gnss, 5)
        else {
            panic!("expected fallback telemetry");
        };
        assert_eq!(recovering.primary_loss_reason, 4);
        assert_eq!(recovering.can_health, 5);
        assert_eq!(recovering.status_flags & (1 << 13), 0);
        assert_ne!(recovering.status_flags & (1 << 14), 0);

        let ApplicationPacket::MissionLinkFallbackTelemetry(controller_error) =
            mission_link_fallback_packet_with_health(&cache, 1_000, gnss, 6)
        else {
            panic!("expected fallback telemetry");
        };
        assert_eq!(controller_error.primary_loss_reason, 7);
        assert_eq!(controller_error.can_health, 6);
        assert_eq!(controller_error.status_flags & (1 << 13), 0);
        assert_ne!(controller_error.status_flags & (1 << 14), 0);
    }

    #[test]
    fn legacy_local_recovery_command_is_not_supported() {
        assert_eq!(LocalCommand::decode(b'r'), None);
    }
}
