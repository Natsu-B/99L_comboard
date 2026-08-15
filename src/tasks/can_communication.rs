use core::sync::atomic::Ordering;

use embassy_futures::select::{Either, Either3, select, select3};
use embassy_time::{Duration, Instant, Ticker};
use embedded_can::{Frame, Id};
use esp_hal::{
    Async,
    twai::{self, EspTwaiError, EspTwaiFrame},
};
use esp_println::println;

use crate::{
    can::{
        cache::{CacheUpdate, FRESHNESS_10_HZ_MS, observed_sequence, sequence_gap},
        health::{CanHealth, classify_can_health},
        protocol::{
            CanRxMessage, CanTxMessage, CommandResult, ControlRollTelemetryV2, RecoveryControl,
            RecoveryOpcode, RecoverySource, RecoveryStatusCode, emergency_failure_result,
            is_emergency_result_command, prioritize_untracked_emergency_result,
        },
        tx::{CanTxError, transmit_message_with_timeout},
    },
    constants::{
        CAN_CONSECUTIVE_ERROR_THRESHOLD, CAN_HEALTH_MONITOR_INTERVAL_MS, CAN_TX_TIMEOUT_MS,
    },
    lora_scheduler::{
        GroundTimeQueueAction, LoRaTxEnvelope, LoRaTxSource, ground_time_queue_action,
    },
    payload::{ApplicationPacket, CommandResultPacket, RecoveryLogPacket},
    state::{
        CAN_CACHE, CAN_HEALTH, CAN_REC, CAN_RX_ERROR_COUNT, CAN_RX_SUCCESS_COUNT,
        CAN_SAFETY_TX_CHANNEL, CAN_TEC, CAN_TX_CHANNEL, CAN_TX_ERROR_COUNT, CAN_TX_SUCCESS_COUNT,
        COMMAND_RESULT_LORA_CHANNEL, COMMAND_TRACKER, CONTROL_ROLL_LORA_SIGNAL,
        EMERGENCY_RESULT_LORA_CHANNEL, GNSS_CMD_CHANNEL, GROUND_TIME_REQUEST_LORA_CHANNEL,
        GnssCommand, IS_CAN_ERROR, LOGGING_REQUESTED, LORA_EMERGENCY_RESULT_DROP_COUNT,
        LORA_GROUND_TIME_REQUEST_DROP_COUNT, LORA_GROUND_TIME_REQUEST_DUPLICATE_COUNT,
        LORA_TX_QUEUE_DROP_COUNT, MISSION_STATUS_INVALID_AT_MS, RAW_CAN_LOG_CHANNEL,
        RAW_CAN_LOG_DROPPED_COUNT, RECOVERY_ASSEMBLER, RECOVERY_BEACON_ACTIVE,
        RECOVERY_ENTER_SENT, RECOVERY_LORA_CHANNEL, RECOVERY_SESSION, RawCanRecord,
    },
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CanRuntimeState {
    AwaitingTraffic,
    Normal,
    BusRecovering,
    TxStateUnknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CanRuntimeEvent {
    TransmitSucceeded,
    ReceiveSucceeded,
    BusOff,
    TimedOutUnknownState,
}

const OBSERVED_CAN_IDS: [u16; 14] = [
    0x011, 0x012, 0x020, 0x100, 0x101, 0x102, 0x103, 0x104, 0x105, 0x106, 0x107, 0x108, 0x109,
    0x10a,
];

struct CanObservedStats {
    counts: [u32; OBSERVED_CAN_IDS.len()],
    sequence_gaps: [u32; OBSERVED_CAN_IDS.len()],
    last_sequences: [Option<u8>; OBSERVED_CAN_IDS.len()],
}

impl CanObservedStats {
    const fn new() -> Self {
        Self {
            counts: [0; OBSERVED_CAN_IDS.len()],
            sequence_gaps: [0; OBSERVED_CAN_IDS.len()],
            last_sequences: [None; OBSERVED_CAN_IDS.len()],
        }
    }

    fn observe(&mut self, identifier: u16, data: &[u8]) {
        let Some(index) = OBSERVED_CAN_IDS.iter().position(|id| *id == identifier) else {
            return;
        };
        self.counts[index] = self.counts[index].saturating_add(1);
        let sequence = observed_sequence(identifier, data);
        if let Some(sequence) = sequence {
            if sequence_gap(self.last_sequences[index], sequence) {
                self.sequence_gaps[index] = self.sequence_gaps[index].saturating_add(1);
            }
            self.last_sequences[index] = Some(sequence);
        }
    }

    fn print(&self) {
        println!(
            "CANID c011={} c012={} c020={} g020={} c100={} g100={} c101={} g101={} c102={} g102={} c103={} g103={} c104={} g104={} c105={} c106={} g106={} c107={} g107={} c108={} g108={} c109={} g109={} c10a={} g10a={}",
            self.counts[0],
            self.counts[1],
            self.counts[2],
            self.sequence_gaps[2],
            self.counts[3],
            self.sequence_gaps[3],
            self.counts[4],
            self.sequence_gaps[4],
            self.counts[5],
            self.sequence_gaps[5],
            self.counts[6],
            self.sequence_gaps[6],
            self.counts[7],
            self.sequence_gaps[7],
            self.counts[8],
            self.counts[9],
            self.sequence_gaps[9],
            self.counts[10],
            self.sequence_gaps[10],
            self.counts[11],
            self.sequence_gaps[11],
            self.counts[12],
            self.sequence_gaps[12],
            self.counts[13],
            self.sequence_gaps[13],
        );
    }
}

const fn transition_runtime(
    state: CanRuntimeState,
    event: CanRuntimeEvent,
) -> (CanRuntimeState, bool) {
    match (state, event) {
        (
            CanRuntimeState::AwaitingTraffic,
            CanRuntimeEvent::TransmitSucceeded | CanRuntimeEvent::ReceiveSucceeded,
        ) => (CanRuntimeState::Normal, false),
        (CanRuntimeState::AwaitingTraffic | CanRuntimeState::Normal, CanRuntimeEvent::BusOff) => {
            (CanRuntimeState::BusRecovering, true)
        }
        (
            CanRuntimeState::AwaitingTraffic | CanRuntimeState::Normal,
            CanRuntimeEvent::TimedOutUnknownState,
        ) => (CanRuntimeState::TxStateUnknown, true),
        (CanRuntimeState::TxStateUnknown, CanRuntimeEvent::BusOff) => {
            (CanRuntimeState::BusRecovering, false)
        }
        (CanRuntimeState::BusRecovering, CanRuntimeEvent::TimedOutUnknownState) => {
            (CanRuntimeState::TxStateUnknown, false)
        }
        (
            CanRuntimeState::BusRecovering | CanRuntimeState::TxStateUnknown,
            CanRuntimeEvent::TransmitSucceeded,
        )
        | (CanRuntimeState::BusRecovering, CanRuntimeEvent::ReceiveSucceeded) => {
            (CanRuntimeState::Normal, false)
        }
        (state, _) => (state, false),
    }
}

fn enter_recovering(
    can: twai::Twai<'static, Async>,
    state: CanRuntimeState,
    event: CanRuntimeEvent,
) -> (twai::Twai<'static, Async>, CanRuntimeState, bool) {
    let (next_state, restart_required) = transition_runtime(state, event);
    let can = if restart_required {
        can.stop().start()
    } else {
        can
    };
    (can, next_state, restart_required)
}

fn publish_health(can: &twai::Twai<'static, Async>) -> CanHealth {
    let tec = can.transmit_error_count();
    let rec = can.receive_error_count();
    let health = classify_can_health(tec, rec, can.is_bus_off());
    CAN_TEC.store(tec, Ordering::Relaxed);
    CAN_REC.store(rec, Ordering::Relaxed);
    CAN_HEALTH.store(health as u8, Ordering::Relaxed);
    health
}

fn publish_error_state(
    state: CanRuntimeState,
    health: CanHealth,
    consecutive_tx_errors: u8,
    consecutive_rx_errors: u8,
    mission_status_fresh: bool,
) {
    IS_CAN_ERROR.store(
        state != CanRuntimeState::Normal
            || health != CanHealth::Active
            || consecutive_tx_errors >= CAN_CONSECUTIVE_ERROR_THRESHOLD
            || consecutive_rx_errors >= CAN_CONSECUTIVE_ERROR_THRESHOLD
            || !mission_status_fresh,
        Ordering::Relaxed,
    );
}

fn queue_command_result(packet: CommandResultPacket) {
    let Ok(frame) = ApplicationPacket::CommandResult(packet).encode() else {
        println!("CommandResult encode error");
        return;
    };
    let envelope = LoRaTxEnvelope::queued(
        frame,
        LoRaTxSource::CommandResult,
        Instant::now().as_micros(),
    );
    if COMMAND_RESULT_LORA_CHANNEL.try_send(envelope).is_err() {
        LORA_TX_QUEUE_DROP_COUNT.fetch_add(1, Ordering::Relaxed);
        println!("CommandResult LoRa queue overflow");
    }
}

fn queue_ground_time_request(request_id: u8, previous_request_id: &mut Option<u8>) {
    let action = ground_time_queue_action(
        *previous_request_id,
        request_id,
        GROUND_TIME_REQUEST_LORA_CHANNEL.is_full(),
    );
    if action == GroundTimeQueueAction::IgnoreDuplicate {
        LORA_GROUND_TIME_REQUEST_DUPLICATE_COUNT.fetch_add(1, Ordering::Relaxed);
        return;
    }
    let Ok(frame) = ApplicationPacket::GroundTimeRequest { request_id }.encode() else {
        println!("GroundTimeRequest encode error");
        return;
    };
    // この区間は単一CAN owner上でawaitしないため、oldest削除と再投入の間にtask切替しない。
    if action == GroundTimeQueueAction::ReplaceOldest
        && GROUND_TIME_REQUEST_LORA_CHANNEL.try_receive().is_ok()
    {
        LORA_TX_QUEUE_DROP_COUNT.fetch_add(1, Ordering::Relaxed);
        LORA_GROUND_TIME_REQUEST_DROP_COUNT.fetch_add(1, Ordering::Relaxed);
        println!("GroundTimeRequest LoRa queue full: oldest dropped");
    }
    let envelope = LoRaTxEnvelope::queued(
        frame,
        LoRaTxSource::GroundTimeRequest,
        Instant::now().as_micros(),
    );
    if GROUND_TIME_REQUEST_LORA_CHANNEL.try_send(envelope).is_err() {
        LORA_TX_QUEUE_DROP_COUNT.fetch_add(1, Ordering::Relaxed);
        LORA_GROUND_TIME_REQUEST_DROP_COUNT.fetch_add(1, Ordering::Relaxed);
        println!("GroundTimeRequest LoRa queue overflow: newest dropped");
        return;
    }
    *previous_request_id = Some(request_id);
}

fn queue_emergency_result(result: CommandResult) {
    let packet = CommandResultPacket {
        transaction_id: result.transaction_id,
        command: result.command,
        phase: result.phase,
        reason: result.reason,
        detail: result.detail,
    };
    let Ok(frame) = ApplicationPacket::CommandResult(packet).encode() else {
        println!("Emergency result encode error");
        return;
    };
    // CAN ownerはLoRa待ちをせず、専用queueへ委譲する。
    let envelope = LoRaTxEnvelope::queued(
        frame,
        LoRaTxSource::EmergencyResult,
        Instant::now().as_micros(),
    );
    if EMERGENCY_RESULT_LORA_CHANNEL.try_send(envelope).is_err() {
        LORA_TX_QUEUE_DROP_COUNT.fetch_add(1, Ordering::Relaxed);
        LORA_EMERGENCY_RESULT_DROP_COUNT.fetch_add(1, Ordering::Relaxed);
        println!("Emergency result LoRa queue overflow");
    }
}

fn queue_recovery_chunk(chunk: crate::can::recovery::RecoveryChunk) -> bool {
    let packet = ApplicationPacket::RecoveryLogData(RecoveryLogPacket {
        transfer_id: chunk.transfer_id,
        source: chunk.source == RecoverySource::MissionSdLatestFlight,
        end_of_file: chunk.end_of_file,
        offset: chunk.offset,
        data_length: chunk.data_length,
        data: chunk.data,
    });
    match packet.encode() {
        Ok(frame)
            if RECOVERY_LORA_CHANNEL
                .try_send(LoRaTxEnvelope::queued(
                    frame,
                    LoRaTxSource::Recovery,
                    Instant::now().as_micros(),
                ))
                .is_ok() =>
        {
            true
        }
        Ok(_) => {
            LORA_TX_QUEUE_DROP_COUNT.fetch_add(1, Ordering::Relaxed);
            println!("Recovery LoRa queue overflow: transfer stopped");
            false
        }
        Err(error) => {
            println!("Recovery LoRa encode error: {:?}", error);
            false
        }
    }
}

async fn fail_recovery_transfer(reason: crate::can::protocol::CommandReason, stop_mission: bool) {
    let resume = RECOVERY_ASSEMBLER.lock().await.abort();
    if stop_mission && let Some(resume) = resume {
        let _ = CAN_SAFETY_TX_CHANNEL.try_send(crate::state::CanTxRequest {
            message: CanTxMessage::RecoveryControl(RecoveryControl {
                opcode: RecoveryOpcode::StopLogDump,
                source: resume.source,
                transfer_id: resume.transfer_id,
                offset: resume.offset,
                length: 0,
            }),
        });
    }
    let detail = resume.map_or(0, |value| value.offset);
    if let Some(result) = RECOVERY_SESSION.lock().await.fail(reason, detail) {
        queue_command_result(CommandResultPacket {
            transaction_id: result.transaction_id,
            command: result.command,
            phase: result.phase,
            reason: result.reason,
            detail: result.detail,
        });
    }
}

async fn apply_received_message(
    message: CanRxMessage,
    received_at_ms: u64,
    previous_time_request_id: &mut Option<u8>,
) {
    let mut cache = CAN_CACHE.lock().await;
    let signal_control_roll = match message {
        CanRxMessage::ControlRollV2(current) => {
            let capture_event = current.flags
                & ControlRollTelemetryV2::REFERENCE_CAPTURED_SINCE_PREVIOUS_FRAME
                != 0;
            capture_event
                || cache.control_roll_v2.value().is_some_and(|previous| {
                    previous.status_signature() != current.status_signature()
                })
        }
        _ => false,
    };
    let update = cache.update(message, received_at_ms);
    drop(cache);
    if signal_control_roll {
        CONTROL_ROLL_LORA_SIGNAL.signal(());
    }
    match update {
        CacheUpdate::CommandResult(result) => {
            let matched = COMMAND_TRACKER.lock().await.apply_result(result);
            if !matched && !is_emergency_result_command(result.command) {
                println!("pending requestに一致しないCommandResult");
            }
            if prioritize_untracked_emergency_result(matched, result.command) {
                queue_emergency_result(result);
            } else {
                queue_command_result(CommandResultPacket {
                    transaction_id: result.transaction_id,
                    command: result.command,
                    phase: result.phase,
                    reason: result.reason,
                    detail: result.detail,
                });
            }
        }
        CacheUpdate::TimeRequest { request_id } => {
            queue_ground_time_request(request_id, previous_time_request_id);
        }
        CacheUpdate::MissionEvent { event, new_flags } => {
            println!(
                "MissionEvent seq={} flags=0x{:04x} new=0x{:04x}",
                event.sequence, event.flags, new_flags
            );
        }
        CacheUpdate::DuplicateMissionEvent => {}
        CacheUpdate::RecoveryStatus(status) => {
            RECOVERY_ASSEMBLER
                .lock()
                .await
                .set_total_size(status.transfer_id, status.total_size);
            if status.status == RecoveryStatusCode::Complete {
                let final_chunk = { RECOVERY_ASSEMBLER.lock().await.finish(status.transfer_id) };
                if let Some(chunk) = final_chunk
                    && !queue_recovery_chunk(chunk)
                {
                    fail_recovery_transfer(
                        crate::can::protocol::CommandReason::InternalError,
                        true,
                    )
                    .await;
                    return;
                }
            }
            if let Some(result) = RECOVERY_SESSION.lock().await.apply_status(status) {
                queue_command_result(CommandResultPacket {
                    transaction_id: result.transaction_id,
                    command: result.command,
                    phase: result.phase,
                    reason: result.reason,
                    detail: result.detail,
                });
            }
        }
        CacheUpdate::RecoveryLogData(fragment) => {
            let assembly = { RECOVERY_ASSEMBLER.lock().await.push(fragment) };
            match assembly {
                Ok(Some(chunk)) if !queue_recovery_chunk(chunk) => {
                    fail_recovery_transfer(
                        crate::can::protocol::CommandReason::InternalError,
                        true,
                    )
                    .await;
                }
                Ok(Some(_)) => {}
                Ok(None) => {}
                Err(error) => {
                    println!("Recovery log assembly error: {:?}", error);
                    fail_recovery_transfer(
                        crate::can::protocol::CommandReason::ProtocolError,
                        true,
                    )
                    .await;
                }
            }
        }
        CacheUpdate::Telemetry => {}
    }
}

fn handle_recovery_mode_command(data: &[u8]) -> bool {
    // Vault 04: ID 0x014 / DLC 3 / mode=1 / reason=0..3のみ有効。
    if data.len() != 3 || data[1] != 1 || data[2] > 3 {
        return false;
    }
    // sequenceはu8 wrap可。同じEnterRecoveryBeaconの再受信はidempotent。
    RECOVERY_BEACON_ACTIVE.store(true, Ordering::Relaxed);
    RECOVERY_ENTER_SENT.store(true, Ordering::Relaxed);
    true
}

async fn handle_received_frame(frame: EspTwaiFrame, previous_time_request_id: &mut Option<u8>) {
    let identifier = match frame.id() {
        Id::Standard(id) => id.as_raw(),
        Id::Extended(id) => {
            CAN_RX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
            println!("extended CAN frame rejected: id=0x{:08x}", id.as_raw());
            return;
        }
    };
    let received_at_ms = Instant::now().as_millis();
    let mut raw = RawCanRecord {
        received_at_ms,
        identifier,
        data_length: frame.data().len() as u8,
        data: [0; 8],
    };
    raw.data[..frame.data().len()].copy_from_slice(frame.data());
    if RAW_CAN_LOG_CHANNEL.try_send(raw).is_err() {
        RAW_CAN_LOG_DROPPED_COUNT.fetch_add(1, Ordering::Relaxed);
    }

    if identifier == 0x014 {
        if !handle_recovery_mode_command(frame.data()) {
            CAN_RX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
            println!("invalid RecoveryModeCommand");
        }
        return;
    }

    match CanRxMessage::decode_standard(identifier, frame.data()) {
        Ok(message) => {
            if identifier == 0x102 {
                MISSION_STATUS_INVALID_AT_MS.store(0, Ordering::Relaxed);
            }
            apply_received_message(message, received_at_ms, previous_time_request_id).await
        }
        Err(error) => {
            if identifier == 0x102 {
                let observed = received_at_ms.min(u32::MAX as u64) as u32;
                MISSION_STATUS_INVALID_AT_MS.store(observed.max(1), Ordering::Relaxed);
            }
            CAN_RX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
            println!("invalid CAN frame: {:?}", error);
        }
    }
}

#[embassy_executor::task]
pub async fn can_communication_task(mut can: twai::Twai<'static, Async>) {
    let mut runtime_state = CanRuntimeState::AwaitingTraffic;
    let mut health_ticker = Ticker::every(Duration::from_millis(CAN_HEALTH_MONITOR_INTERVAL_MS));
    let tx_timeout = Duration::from_millis(CAN_TX_TIMEOUT_MS);
    let mut consecutive_tx_errors = 0u8;
    let mut consecutive_rx_errors = 0u8;
    let mut observed = CanObservedStats::new();
    let mut health_ticks = 0u8;
    let mut previous_time_request_id = None;

    loop {
        match select3(
            health_ticker.next(),
            select(CAN_SAFETY_TX_CHANNEL.receive(), CAN_TX_CHANNEL.receive()),
            can.receive_async(),
        )
        .await
        {
            Either3::Second(request) => {
                let mut request = match request {
                    Either::First(request) | Either::Second(request) => request,
                };
                loop {
                    match transmit_message_with_timeout(&mut can, request.message, tx_timeout).await
                    {
                        Ok(()) => {
                            CAN_TX_SUCCESS_COUNT.fetch_add(1, Ordering::Relaxed);
                            consecutive_tx_errors = 0;
                            runtime_state = transition_runtime(
                                runtime_state,
                                CanRuntimeEvent::TransmitSucceeded,
                            )
                            .0;
                            if let CanTxMessage::GenericCommandRequest(generic) = request.message
                                && generic.command == 0x01
                            {
                                LOGGING_REQUESTED.store(true, Ordering::Relaxed);
                                let _ = GNSS_CMD_CHANNEL.try_send(GnssCommand::TurnOn);
                            }
                        }
                        Err(error) => {
                            CAN_TX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
                            consecutive_tx_errors = consecutive_tx_errors.saturating_add(1);
                            println!("CAN transmit error: {:?}", error);
                            if let Some(result) = emergency_failure_result(request.message) {
                                queue_emergency_result(result);
                            } else {
                                match request.message {
                                    crate::can::protocol::CanTxMessage::GenericCommandRequest(
                                        generic,
                                    ) => {
                                        let result = crate::can::protocol::CommandResult {
                                            transaction_id: generic.transaction_id,
                                            command: generic.command,
                                            phase: crate::can::protocol::CommandPhase::Failed,
                                            reason: crate::can::protocol::CommandReason::Timeout,
                                            detail: 0,
                                        };
                                        if COMMAND_TRACKER.lock().await.apply_result(result) {
                                            queue_command_result(CommandResultPacket {
                                                transaction_id: result.transaction_id,
                                                command: result.command,
                                                phase: result.phase,
                                                reason: result.reason,
                                                detail: result.detail,
                                            });
                                        }
                                    }
                                    crate::can::protocol::CanTxMessage::RecoveryControl(
                                        control,
                                    ) => {
                                        if let Some(result) =
                                            RECOVERY_SESSION.lock().await.fail_matching(
                                                control.transfer_id,
                                                crate::can::protocol::CommandReason::Timeout,
                                                0,
                                            )
                                        {
                                            let _ = RECOVERY_ASSEMBLER.lock().await.abort();
                                            queue_command_result(CommandResultPacket {
                                                transaction_id: result.transaction_id,
                                                command: result.command,
                                                phase: result.phase,
                                                reason: result.reason,
                                                detail: result.detail,
                                            });
                                        }
                                    }
                                    _ => {}
                                }
                            }
                            if matches!(
                                error,
                                CanTxError::BusOff | CanTxError::TimedOutUnknownState
                            ) {
                                let event = if error == CanTxError::BusOff {
                                    CanRuntimeEvent::BusOff
                                } else {
                                    CanRuntimeEvent::TimedOutUnknownState
                                };
                                let recovery = enter_recovering(can, runtime_state, event);
                                can = recovery.0;
                                runtime_state = recovery.1;
                            }
                        }
                    }
                    let Ok(next_safety) = CAN_SAFETY_TX_CHANNEL.try_receive() else {
                        break;
                    };
                    request = next_safety;
                }
            }
            Either3::Third(result) => match result {
                Ok(frame) => {
                    if let Id::Standard(id) = frame.id() {
                        observed.observe(id.as_raw(), frame.data());
                    }
                    CAN_RX_SUCCESS_COUNT.fetch_add(1, Ordering::Relaxed);
                    consecutive_rx_errors = 0;
                    runtime_state =
                        transition_runtime(runtime_state, CanRuntimeEvent::ReceiveSucceeded).0;
                    handle_received_frame(frame, &mut previous_time_request_id).await;
                }
                Err(error) => {
                    CAN_RX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
                    consecutive_rx_errors = consecutive_rx_errors.saturating_add(1);
                    println!("CAN receive error: {:?}", error);
                    if error == EspTwaiError::BusOff {
                        let recovery =
                            enter_recovering(can, runtime_state, CanRuntimeEvent::BusOff);
                        can = recovery.0;
                        runtime_state = recovery.1;
                    }
                }
            },
            Either3::First(_) => {
                health_ticks = health_ticks.saturating_add(1);
                if health_ticks == 100 {
                    health_ticks = 0;
                    observed.print();
                }
                let health = publish_health(&can);
                if health == CanHealth::BusOff {
                    let recovery = enter_recovering(can, runtime_state, CanRuntimeEvent::BusOff);
                    can = recovery.0;
                    runtime_state = recovery.1;
                }
            }
        }
        let health = publish_health(&can);
        let mission_status_fresh = CAN_CACHE
            .lock()
            .await
            .mission_status
            .freshness(Instant::now().as_millis(), FRESHNESS_10_HZ_MS)
            == crate::can::cache::Freshness::Fresh;
        publish_error_state(
            runtime_state,
            health,
            consecutive_tx_errors,
            consecutive_rx_errors,
            mission_status_fresh,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recovery_mode_command_requires_vault_layout() {
        RECOVERY_BEACON_ACTIVE.store(false, Ordering::Relaxed);
        assert!(handle_recovery_mode_command(&[0x10, 1, 0]));
        assert!(RECOVERY_BEACON_ACTIVE.load(Ordering::Relaxed));
        assert!(!handle_recovery_mode_command(&[0x10, 0, 0]));
        assert!(!handle_recovery_mode_command(&[0x10, 1, 4]));
        assert!(!handle_recovery_mode_command(&[0x10, 1]));
    }
}
