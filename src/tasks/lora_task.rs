use core::sync::atomic::Ordering;

use embassy_futures::select::{Either, Either3, select, select3};
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
    constants::{LORA_AUX_TIMEOUT_MS, LORA_RX_TX_GUARD_MS, LORA_TRANSMIT_INTERVAL_MS},
    lora_uplink::{UplinkCommand, UplinkFrameBuffer},
    payload::{
        ApplicationPacket, CommandReceiveTelemetry, CommandResultPacket, DescentTelemetry,
        FlightTelemetry, LoraFrame, PacketHeader, RecoveryBeacon,
    },
    state::{
        CAN_CACHE, CAN_SAFETY_TX_CHANNEL, CAN_TX_CHANNEL, COMMAND_TRACKER, CanTxRequest,
        GNSS_CMD_CHANNEL, GNSS_TELEMETRY, GnssCommand, IMMEDIATE_LORA_CHANNEL, IS_CAN_ERROR,
        LOGGING_REQUESTED, LORA_AUX_TIMEOUT_COUNT, LORA_COMMAND_DROP_COUNT, LORA_RX_ERROR_COUNT,
        LORA_TX_ERROR_COUNT, RECOVERY_ASSEMBLER, RECOVERY_BEACON_ACTIVE, RECOVERY_ENTER_SENT,
        RECOVERY_LORA_CHANNEL, RECOVERY_SESSION, SD_FLUSH_SIGNAL, SD_HAS_ERROR,
        UPLINK_COMMAND_CHANNEL,
    },
};

static LORA_RX_ACTIVITY_SIGNAL: Signal<CriticalSectionRawMutex, Instant> = Signal::new();

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
enum LocalCommand {
    StartLogging = b'l',
    StopLogging = b'm',
    GnssOn = b'g',
    GnssOff = b'h',
    EnterRecovery = b'r',
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
            b'r' => Some(Self::EnterRecovery),
            b'w' => Some(Self::Wake),
            b'f' => Some(Self::DumpInternalFlash),
            b's' => Some(Self::DumpMissionSd),
            b'x' => Some(Self::StopDump),
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

async fn transmit_frame(
    tx: &mut UartTx<'static, Async>,
    aux_pin: &mut Input<'static>,
    frame: LoraFrame,
) -> bool {
    if !wait_for_aux_high(aux_pin).await {
        return false;
    }
    wait_for_rx_guard().await;
    if !wait_for_aux_high(aux_pin).await {
        return false;
    }
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
    if let Err(error) = tx.flush_async().await {
        LORA_TX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
        println!("LoRa UART flush error: {:?}", error);
        return false;
    }
    wait_for_aux_high(aux_pin).await
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

async fn periodic_packet() -> Option<(LoraFrame, u16, u32)> {
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

    if !RECOVERY_BEACON_ACTIVE.load(Ordering::Relaxed)
        && state == MissionState::Descent
        && cache.mission_event.value().is_some_and(|event| {
            event.state == MissionState::Descent && event.elapsed < 0xfff0 && event.elapsed >= 1_200
        })
    {
        RECOVERY_BEACON_ACTIVE.store(true, Ordering::Relaxed);
    }
    if RECOVERY_BEACON_ACTIVE.load(Ordering::Relaxed) {
        if !RECOVERY_ENTER_SENT.swap(true, Ordering::Relaxed) {
            CAN_TX_CHANNEL
                .send(CanTxRequest {
                    message: CanTxMessage::RecoveryControl(RecoveryControl {
                        opcode: RecoveryOpcode::EnterRecovery,
                        source: RecoverySource::InternalFlash,
                        transfer_id: 0xff,
                        offset: 0,
                        length: 0,
                    }),
                })
                .await;
        }
        let packet = ApplicationPacket::RecoveryBeacon(RecoveryBeacon {
            logic_voltage: power.map_or(253, |value| value.logic_voltage),
            motor_voltage: power.map_or(253, |value| value.motor_voltage),
            east: gnss.east,
            north: gnss.north,
            height: gnss.height,
            elapsed: power.map_or(0xfffa, |value| value.recovery_elapsed),
        });
        return finish_periodic_packet(packet, 0, event_revision);
    }

    let packet = match state {
        MissionState::CommandReceive | MissionState::Unknown => {
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
            let mut status = descent.map_or(0, |value| value.status);
            status = inject_link_health(status, 5, 6) & 0x1fff;
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
    };
    let transmitted_events = if matches!(
        state,
        MissionState::LiftoffDetection | MissionState::EngineBurn | MissionState::Control
    ) {
        interval_events & 0x001f
    } else {
        0
    };
    finish_periodic_packet(packet, transmitted_events, event_revision)
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
        status |= u32::from(power.flags & (1 << 6) != 0) << 19;
        status |= u32::from(power.flags & (1 << 5) != 0) << 20;
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

const fn periodic_interval_ms(recovery_active: bool) -> u64 {
    if recovery_active {
        10_000
    } else {
        LORA_TRANSMIT_INTERVAL_MS
    }
}

const RECOVERY_LOG_INTERVAL_MS: u64 = 200;

async fn queue_result(transaction_id: u8, command: u8, phase: CommandPhase, reason: CommandReason) {
    if let Ok(frame) = ApplicationPacket::CommandResult(CommandResultPacket {
        transaction_id,
        command,
        phase,
        reason,
        detail: 0,
    })
    .encode()
    {
        let _ = IMMEDIATE_LORA_CHANNEL.try_send(frame);
    }
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
            if let Ok(frame) = ApplicationPacket::CommandResult(CommandResultPacket {
                transaction_id: result.transaction_id,
                command: result.command,
                phase: result.phase,
                reason: result.reason,
                detail: result.detail,
            })
            .encode()
            {
                let _ = IMMEDIATE_LORA_CHANNEL.try_send(frame);
            }
        }
        RegisterResult::ProtocolError => {
            queue_result(
                request.transaction_id,
                request.command,
                CommandPhase::Rejected,
                CommandReason::ProtocolError,
            )
            .await;
        }
        RegisterResult::Busy => {
            queue_result(
                request.transaction_id,
                request.command,
                CommandPhase::Rejected,
                CommandReason::Busy,
            )
            .await;
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
        )
        .await;
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
        )
        .await;
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
        LocalCommand::EnterRecovery
        | LocalCommand::Wake
        | LocalCommand::DumpInternalFlash
        | LocalCommand::DumpMissionSd
        | LocalCommand::StopDump => {
            let opcode = match command_value {
                LocalCommand::EnterRecovery => RecoveryOpcode::EnterRecovery,
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
                )
                .await;
                return;
            };
            if let Some(result) = interrupted {
                queue_result(
                    result.transaction_id,
                    result.command,
                    result.phase,
                    result.reason,
                )
                .await;
            }
            if command_value == LocalCommand::StopDump {
                let _ = RECOVERY_ASSEMBLER.lock().await.abort();
            }
            if command_value == LocalCommand::EnterRecovery {
                RECOVERY_BEACON_ACTIVE.store(true, Ordering::Relaxed);
                RECOVERY_ENTER_SENT.store(true, Ordering::Relaxed);
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
            )
            .await;
            return;
        }
    }
    queue_result(
        transaction_id,
        command,
        CommandPhase::Completed,
        CommandReason::None,
    )
    .await;
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
                if length != 0 {
                    LORA_RX_ACTIVITY_SIGNAL.signal(Instant::now());
                }
                for byte in &rx_buf[..length] {
                    if let Some(result) = uplink_frame.push(*byte) {
                        LORA_RX_ACTIVITY_SIGNAL.signal(Instant::now());
                        match result {
                            Ok(command) => match command {
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
                                            message: CanTxMessage::LiftoffEmergencyStop {
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
                            },
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
    let mut interval = Duration::from_millis(periodic_interval_ms(false));
    let mut next_tx_at = Instant::now() + interval;
    let mut next_recovery_at = Instant::now();
    let mut pending_recovery = None;
    loop {
        let had_pending_recovery = pending_recovery.is_some();
        let recovery_ready = async {
            if let Some(frame) = pending_recovery {
                Timer::at(next_recovery_at).await;
                frame
            } else {
                RECOVERY_LORA_CHANNEL.receive().await
            }
        };
        let (frame, interval_events, event_revision) = match select3(
            IMMEDIATE_LORA_CHANNEL.receive(),
            Timer::at(next_tx_at),
            recovery_ready,
        )
        .await
        {
            Either3::First(frame) => (frame, 0, 0),
            Either3::Second(()) => {
                interval = Duration::from_millis(periodic_interval_ms(
                    RECOVERY_BEACON_ACTIVE.load(Ordering::Relaxed),
                ));
                let scheduled_next = next_tx_at + interval;
                let now = Instant::now();
                next_tx_at = if scheduled_next <= now {
                    now + interval
                } else {
                    scheduled_next
                };
                let Some(frame) = periodic_packet().await else {
                    continue;
                };
                frame
            }
            Either3::Third(frame) if had_pending_recovery || Instant::now() >= next_recovery_at => {
                pending_recovery = None;
                next_recovery_at = Instant::now() + Duration::from_millis(RECOVERY_LOG_INTERVAL_MS);
                (frame, 0, 0)
            }
            Either3::Third(frame) => {
                pending_recovery = Some(frame);
                continue;
            }
        };
        if transmit_frame(&mut tx, &mut aux_pin, frame).await && interval_events != 0 {
            CAN_CACHE
                .lock()
                .await
                .clear_event_flags(interval_events, event_revision);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recovery_beacon_uses_ten_second_interval() {
        assert_eq!(periodic_interval_ms(false), 500);
        assert_eq!(periodic_interval_ms(true), 10_000);
    }

    #[test]
    fn link_health_bits_are_injected_without_changing_other_bits() {
        SD_HAS_ERROR.store(false, Ordering::Relaxed);
        IS_CAN_ERROR.store(false, Ordering::Relaxed);
        assert_eq!(inject_link_health(1, 7, 8), 0x0181);
    }
}
