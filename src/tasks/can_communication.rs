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
        cache::{CacheUpdate, FRESHNESS_10_HZ_MS},
        health::{CanHealth, classify_can_health},
        protocol::CanRxMessage,
        tx::{CanTxError, transmit_message_with_timeout},
    },
    constants::{
        CAN_CONSECUTIVE_ERROR_THRESHOLD, CAN_HEALTH_MONITOR_INTERVAL_MS, CAN_TX_TIMEOUT_MS,
    },
    payload::{ApplicationPacket, CommandResultPacket},
    state::{
        CAN_CACHE, CAN_HEALTH, CAN_REC, CAN_RX_ERROR_COUNT, CAN_SAFETY_TX_SIGNAL, CAN_TEC,
        CAN_TX_CHANNEL, CAN_TX_ERROR_COUNT, COMMAND_TRACKER, IMMEDIATE_LORA_CHANNEL, IS_CAN_ERROR,
        RAW_CAN_LOG_CHANNEL, RAW_CAN_LOG_DROPPED_COUNT, RawCanRecord,
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
) {
    IS_CAN_ERROR.store(
        state != CanRuntimeState::Normal
            || health != CanHealth::Active
            || consecutive_tx_errors >= CAN_CONSECUTIVE_ERROR_THRESHOLD
            || consecutive_rx_errors >= CAN_CONSECUTIVE_ERROR_THRESHOLD,
        Ordering::Relaxed,
    );
}

async fn queue_application(packet: ApplicationPacket) {
    match packet.encode() {
        Ok(frame) => {
            if IMMEDIATE_LORA_CHANNEL.try_send(frame).is_err() {
                println!("即時LoRa packet queue overflow");
            }
        }
        Err(error) => println!("LoRa packet encode error: {:?}", error),
    }
}

async fn apply_received_message(message: CanRxMessage, received_at_ms: u64) {
    let update = CAN_CACHE.lock().await.update(message, received_at_ms);
    match update {
        CacheUpdate::CommandResult(result) => {
            if !COMMAND_TRACKER.lock().await.apply_result(result) {
                println!("pending requestに一致しないCommandResult");
            }
            queue_application(ApplicationPacket::CommandResult(CommandResultPacket {
                transaction_id: result.transaction_id,
                command: result.command,
                phase: result.phase,
                reason: result.reason,
                detail: result.detail,
            }))
            .await;
        }
        CacheUpdate::TimeRequest { request_id } => {
            queue_application(ApplicationPacket::GroundTimeRequest { request_id }).await;
        }
        CacheUpdate::MissionEvent { event, new_flags } => {
            println!(
                "MissionEvent seq={} flags=0x{:04x} new=0x{:04x}",
                event.sequence, event.flags, new_flags
            );
        }
        CacheUpdate::DuplicateMissionEvent => {}
        CacheUpdate::RecoveryStatus(status) => println!("RecoveryStatus: {:?}", status),
        CacheUpdate::RecoveryLogData(_) | CacheUpdate::Telemetry => {}
    }
}

async fn handle_received_frame(frame: EspTwaiFrame) {
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

    match CanRxMessage::decode_standard(identifier, frame.data()) {
        Ok(message) => apply_received_message(message, received_at_ms).await,
        Err(error) => {
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

    loop {
        match select3(
            health_ticker.next(),
            select(CAN_SAFETY_TX_SIGNAL.wait(), CAN_TX_CHANNEL.receive()),
            can.receive_async(),
        )
        .await
        {
            Either3::Second(request) => {
                let request = match request {
                    Either::First(request) | Either::Second(request) => request,
                };
                match transmit_message_with_timeout(&mut can, request.message, tx_timeout).await {
                    Ok(()) => {
                        consecutive_tx_errors = 0;
                        runtime_state =
                            transition_runtime(runtime_state, CanRuntimeEvent::TransmitSucceeded).0;
                    }
                    Err(error) => {
                        CAN_TX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
                        consecutive_tx_errors = consecutive_tx_errors.saturating_add(1);
                        println!("CAN transmit error: {:?}", error);
                        if matches!(error, CanTxError::BusOff | CanTxError::TimedOutUnknownState) {
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
            }
            Either3::Third(result) => match result {
                Ok(frame) => {
                    consecutive_rx_errors = 0;
                    runtime_state =
                        transition_runtime(runtime_state, CanRuntimeEvent::ReceiveSucceeded).0;
                    handle_received_frame(frame).await;
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
                let health = publish_health(&can);
                if health == CanHealth::BusOff {
                    let recovery = enter_recovering(can, runtime_state, CanRuntimeEvent::BusOff);
                    can = recovery.0;
                    runtime_state = recovery.1;
                }
                let now_ms = Instant::now().as_millis();
                if CAN_CACHE
                    .lock()
                    .await
                    .mission_status
                    .freshness(now_ms, FRESHNESS_10_HZ_MS)
                    != crate::can::cache::Freshness::Fresh
                {
                    IS_CAN_ERROR.store(true, Ordering::Relaxed);
                }
            }
        }
        let health = publish_health(&can);
        publish_error_state(
            runtime_state,
            health,
            consecutive_tx_errors,
            consecutive_rx_errors,
        );
    }
}
