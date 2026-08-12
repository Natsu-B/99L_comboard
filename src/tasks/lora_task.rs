use core::sync::atomic::Ordering;

use embassy_futures::select::{Either, select};
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
        CAN_CACHE, CAN_SAFETY_TX_SIGNAL, CAN_TX_CHANNEL, COMMAND_TRACKER, CanTxRequest,
        GNSS_CMD_CHANNEL, GNSS_TELEMETRY, GnssCommand, IMMEDIATE_LORA_CHANNEL, IS_CAN_ERROR,
        LOGGING_REQUESTED, LORA_AUX_TIMEOUT_COUNT, LORA_COMMAND_DROP_COUNT, LORA_RX_ERROR_COUNT,
        LORA_TX_ERROR_COUNT, RECOVERY_BEACON_ACTIVE, RECOVERY_ENTER_SENT, RECOVERY_SESSION,
        SD_FLUSH_SIGNAL, SD_HAS_ERROR, UPLINK_COMMAND_CHANNEL,
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
) {
    if !wait_for_aux_high(aux_pin).await {
        return;
    }
    wait_for_rx_guard().await;
    if !wait_for_aux_high(aux_pin).await {
        return;
    }
    match write_all(tx, frame.as_bytes()).await {
        Ok(true) => {}
        Ok(false) => {
            LORA_TX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
            return;
        }
        Err(error) => {
            LORA_TX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
            println!("LoRa UART write error: {:?}", error);
            return;
        }
    }
    if let Err(error) = tx.flush_async().await {
        LORA_TX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
        println!("LoRa UART flush error: {:?}", error);
        return;
    }
    let _ = wait_for_aux_high(aux_pin).await;
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

async fn periodic_packet() -> Option<LoraFrame> {
    let now_ms = Instant::now().as_millis();
    let cache = *CAN_CACHE.lock().await;
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
        && power
            .is_some_and(|value| value.descent_elapsed < 0xfff0 && value.descent_elapsed >= 1_200)
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
        return ApplicationPacket::RecoveryBeacon(RecoveryBeacon {
            logic_voltage: power.map_or(253, |value| value.logic_voltage),
            motor_voltage: power.map_or(253, |value| value.motor_voltage),
            east: gnss.east,
            north: gnss.north,
            height: gnss.height,
            elapsed: power.map_or(0xfffa, |value| value.recovery_elapsed),
        })
        .encode()
        .ok();
    }

    let packet = match state {
        MissionState::CommandReceive | MissionState::Unknown => {
            let mut status = mission.map_or(0, |value| {
                u32::from(value.status) | (u32::from(value.config) << 16)
            });
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
                tilt_magnitude: tilt.map_or(121, |value| value.magnitude),
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
            ApplicationPacket::Flight(FlightTelemetry {
                header,
                status: mission.map_or(0, |value| value.status),
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
            let descent = stale_or(cache.descent_core, now_ms, FRESHNESS_10_HZ_MS);
            ApplicationPacket::Descent(DescentTelemetry {
                status: descent.map_or(0, |value| value.status),
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
    packet.encode().ok()
}

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
            if !RECOVERY_SESSION.lock().await.start(pending) {
                queue_result(
                    transaction_id,
                    command,
                    CommandPhase::Rejected,
                    CommandReason::Busy,
                )
                .await;
                return;
            }
            if command_value == LocalCommand::EnterRecovery {
                RECOVERY_BEACON_ACTIVE.store(true, Ordering::Relaxed);
                RECOVERY_ENTER_SENT.store(true, Ordering::Relaxed);
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
            UplinkCommand::ActuatorEmergency { transaction_id } => {
                CAN_SAFETY_TX_SIGNAL.signal(CanTxRequest {
                    message: CanTxMessage::ActuatorEmergencyStop { transaction_id },
                });
            }
            UplinkCommand::LiftoffDetectionEmergency { transaction_id } => {
                CAN_SAFETY_TX_SIGNAL.signal(CanTxRequest {
                    message: CanTxMessage::LiftoffEmergencyStop { transaction_id },
                });
            }
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
                for byte in &rx_buf[..length] {
                    if let Some(result) = uplink_frame.push(*byte) {
                        LORA_RX_ACTIVITY_SIGNAL.signal(Instant::now());
                        match result {
                            Ok(command) => {
                                if UPLINK_COMMAND_CHANNEL.try_send(command).is_err() {
                                    LORA_COMMAND_DROP_COUNT.fetch_add(1, Ordering::Relaxed);
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
    let interval = Duration::from_millis(LORA_TRANSMIT_INTERVAL_MS);
    let mut next_tx_at = Instant::now() + interval;
    loop {
        let frame = match select(IMMEDIATE_LORA_CHANNEL.receive(), Timer::at(next_tx_at)).await {
            Either::First(frame) => frame,
            Either::Second(()) => {
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
        };
        transmit_frame(&mut tx, &mut aux_pin, frame).await;
    }
}
