use core::sync::atomic::Ordering;

use embassy_time::Instant;
use esp_println::println;

use crate::{
    can::{
        command::RegisterResult,
        protocol::{
            CanTxMessage, CommandPhase, CommandReason, GenericCommandRequest, RecoveryControl,
            RecoveryOpcode, RecoverySource,
        },
        recovery::PendingRecovery,
    },
    constants::LOCAL_EXIT_RECOVERY_COMMAND,
    lora_scheduler::{LoRaTxEnvelope, LoRaTxSource},
    lora_uplink::UplinkCommand,
    payload::{ApplicationPacket, CommandResultPacket},
    state::{
        CAN_TX_CHANNEL, COMMAND_RESULT_LORA_CHANNEL, COMMAND_TRACKER, GNSS_CMD_CHANNEL,
        GnssCommand, LOGGING_REQUESTED, LORA_TX_QUEUE_DROP_COUNT, RECOVERY_ASSEMBLER,
        RECOVERY_BEACON_ACTIVE, RECOVERY_SESSION, SD_FLUSH_SIGNAL, CanTxRequest,
        UPLINK_COMMAND_CHANNEL,
    },
};

#[path = "lora_task_base.rs"]
mod base;

pub use base::{lora_rx_task, lora_tx_task};
#[cfg(feature = "lora-timing-debug")]
pub use base::lora_timing_report_task;

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
    ExitRecovery = LOCAL_EXIT_RECOVERY_COMMAND,
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
            LOCAL_EXIT_RECOVERY_COMMAND => Some(Self::ExitRecovery),
            // 旧r=ForceRecoveryBeaconはMission所有権に反するためNotSupportedへ落とす。
            _ => None,
        }
    }
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
        | LocalCommand::StopDump
        | LocalCommand::ExitRecovery => {
            if command_value == LocalCommand::ExitRecovery
                && !RECOVERY_BEACON_ACTIVE.load(Ordering::Relaxed)
            {
                queue_result(
                    transaction_id,
                    command,
                    CommandPhase::Rejected,
                    CommandReason::InvalidState,
                );
                return;
            }

            let opcode = match command_value {
                LocalCommand::Wake => RecoveryOpcode::Wake,
                LocalCommand::DumpInternalFlash | LocalCommand::DumpMissionSd => {
                    RecoveryOpcode::StartLogDump
                }
                LocalCommand::StopDump => RecoveryOpcode::StopLogDump,
                LocalCommand::ExitRecovery => RecoveryOpcode::ExitRecovery,
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recovery_exit_local_command_is_unique() {
        assert_eq!(LocalCommand::decode(b'r'), None);
        assert_eq!(
            LocalCommand::decode(LOCAL_EXIT_RECOVERY_COMMAND),
            Some(LocalCommand::ExitRecovery)
        );
        assert_eq!(LOCAL_EXIT_RECOVERY_COMMAND, b'e');
    }
}
