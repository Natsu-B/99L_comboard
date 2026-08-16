use core::sync::atomic::Ordering;

use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Ticker, Timer};
use esp_hal::{
    Async,
    gpio::Output,
    uart::{Config as UartConfig, Uart},
};
use esp_println::println;

use crate::{
    can::protocol::{CanTxMessage, TimeSource},
    gnss::{
        FixQuality, gnss_setting, parse_gga, parse_rmc_datetime,
        telemetry::{
            GNSS_COORDINATE_INVALID, GNSS_COORDINATE_NO_FIX, GNSS_COORDINATE_RECEIVER_ERROR,
            GNSS_COORDINATE_STALE, GNSS_COORDINATE_UNAVAILABLE, GNSS_HEIGHT_INVALID,
            GNSS_HEIGHT_NO_FIX, GNSS_HEIGHT_RECEIVER_ERROR, GNSS_HEIGHT_STALE,
            GNSS_HEIGHT_UNAVAILABLE, encode_position,
        },
    },
    state::{
        CAN_CACHE, CAN_TX_CHANNEL, CanTxRequest, GNSS_CHANNEL, GNSS_CHANNEL_DROP_COUNT,
        GNSS_CMD_CHANNEL, GNSS_RX_ERROR_COUNT, GNSS_SETTING_ERROR_COUNT, GNSS_TELEMETRY,
        GNSS_TIME_MILLISECONDS, GNSS_TIME_UNIX_SECONDS, GNSS_TIME_UPDATED_AT_MS, GNSS_TIME_VALID,
        GnssCommand, GnssReceiverState,
    },
};

const GNSS_STALE_TIMEOUT_MS: u64 = 3_000;
const GNSS_TIME_RESPONSE_POLL_MS: u64 = 100;

async fn set_receiver_state(state: GnssReceiverState) {
    let mut telemetry = GNSS_TELEMETRY.lock().await;
    telemetry.state = state;
    if state == GnssReceiverState::Starting {
        telemetry.started_at_ms = Some(Instant::now().as_millis());
    }
    if state == GnssReceiverState::ConfigurationFailed || state == GnssReceiverState::ReceiverError
    {
        telemetry.east = GNSS_COORDINATE_RECEIVER_ERROR;
        telemetry.north = GNSS_COORDINATE_RECEIVER_ERROR;
        telemetry.height = GNSS_HEIGHT_RECEIVER_ERROR;
    }
}

#[embassy_executor::task]
pub async fn gnss_manager_task(mut uart: Uart<'static, Async>, mut gnss_en: Output<'static>) {
    let mut read_buf = [0u8; 90];
    let mut line_buf = [0u8; 90];
    let mut line_length = 0;
    let mut discard_line = false;
    let mut is_on = false;

    loop {
        match select(uart.read_async(&mut read_buf), GNSS_CMD_CHANNEL.receive()).await {
            Either::First(Ok(bytes_read)) => {
                if bytes_read == 0 {
                    continue;
                }
                for letter in &read_buf[..bytes_read] {
                    if *letter == b'$' {
                        line_length = 0;
                        discard_line = false;
                        line_buf[line_length] = *letter;
                        line_length += 1;
                        continue;
                    }
                    if *letter == b'\r' {
                        continue;
                    }
                    if *letter == b'\n' {
                        if !discard_line && line_length > 0 && line_buf[0] == b'$' {
                            let now_ms = Instant::now().as_millis();
                            {
                                let mut telemetry = GNSS_TELEMETRY.lock().await;
                                telemetry.last_receiver_at_ms = Some(now_ms);
                                if matches!(
                                    telemetry.state,
                                    GnssReceiverState::Starting | GnssReceiverState::ReceiverError
                                ) {
                                    telemetry.state = GnssReceiverState::ReceiverDetected;
                                }
                            }
                            let mut send_buf = [0u8; 90];
                            send_buf[..line_length].copy_from_slice(&line_buf[..line_length]);
                            if GNSS_CHANNEL.try_send(send_buf).is_err() {
                                GNSS_CHANNEL_DROP_COUNT.fetch_add(1, Ordering::Relaxed);
                            }
                        }
                        line_length = 0;
                        discard_line = false;
                        continue;
                    }
                    if line_length < line_buf.len() {
                        line_buf[line_length] = *letter;
                        line_length += 1;
                    } else {
                        discard_line = true;
                    }
                }
            }
            Either::First(Err(error)) => {
                GNSS_RX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
                println!("GNSS UART receive error: {:?}", error);
            }
            Either::Second(command) => match command {
                GnssCommand::TurnOn => {
                    if is_on {
                        continue;
                    }
                    GNSS_TIME_VALID.store(false, Ordering::Release);
                    set_receiver_state(GnssReceiverState::Starting).await;
                    gnss_en.set_high();
                    let config_9600 = UartConfig::default().with_baudrate(9_600);
                    if uart.apply_config(&config_9600).is_err() {
                        GNSS_SETTING_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
                        set_receiver_state(GnssReceiverState::ConfigurationFailed).await;
                        gnss_en.set_low();
                        continue;
                    }
                    // receiver起動時のNMEAを待機中も読み、UART FIFO overflowを防ぐ。
                    let startup_deadline = Instant::now() + Duration::from_millis(500);
                    loop {
                        match select(uart.read_async(&mut read_buf), Timer::at(startup_deadline))
                            .await
                        {
                            Either::First(Ok(_)) => {}
                            Either::First(Err(error)) => {
                                GNSS_RX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
                                println!("GNSS startup UART receive error: {:?}", error);
                            }
                            Either::Second(()) => break,
                        }
                    }
                    line_length = 0;
                    discard_line = false;
                    match gnss_setting(&mut uart).await {
                        Ok(report) => println!(
                            "GNSS configuration ACK count={}, final baud ACK unverified={}",
                            report.acknowledged_commands, report.final_baud_ack_unverified
                        ),
                        Err(error) => {
                            GNSS_SETTING_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
                            GNSS_TIME_VALID.store(false, Ordering::Release);
                            println!("GNSS setting failed: {:?}", error);
                            set_receiver_state(GnssReceiverState::ConfigurationFailed).await;
                            gnss_en.set_low();
                            continue;
                        }
                    }
                    Timer::after(Duration::from_millis(50)).await;
                    if uart
                        .apply_config(&UartConfig::default().with_baudrate(115_200))
                        .is_err()
                    {
                        GNSS_SETTING_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
                        GNSS_TIME_VALID.store(false, Ordering::Release);
                        set_receiver_state(GnssReceiverState::ConfigurationFailed).await;
                        gnss_en.set_low();
                        continue;
                    }
                    is_on = true;
                }
                GnssCommand::TurnOff => {
                    gnss_en.set_low();
                    is_on = false;
                    GNSS_TIME_VALID.store(false, Ordering::Release);
                    {
                        let mut telemetry = GNSS_TELEMETRY.lock().await;
                        telemetry.state = GnssReceiverState::Off;
                        telemetry.east = GNSS_COORDINATE_UNAVAILABLE;
                        telemetry.north = GNSS_COORDINATE_UNAVAILABLE;
                        telemetry.height = GNSS_HEIGHT_UNAVAILABLE;
                    }
                    // 再ON後にOFF前のsentenceを再利用しない。
                    while GNSS_CHANNEL.try_receive().is_ok() {}
                }
            },
        }
    }
}

fn is_sentence(sentence: &[u8], suffix: &[u8; 3]) -> bool {
    sentence
        .iter()
        .position(|value| *value == b',')
        .is_some_and(|comma| comma >= 4 && &sentence[comma - 3..comma] == suffix)
}

fn is_gga(sentence: &[u8]) -> bool {
    is_sentence(sentence, b"GGA")
}

fn is_rmc(sentence: &[u8]) -> bool {
    is_sentence(sentence, b"RMC")
}

#[embassy_executor::task]
pub async fn parse_gnss_task() {
    let mut stale_ticker = Ticker::every(Duration::from_secs(1));
    loop {
        match select(GNSS_CHANNEL.receive(), stale_ticker.next()).await {
            Either::First(sentence) => {
                if matches!(
                    GNSS_TELEMETRY.lock().await.state,
                    GnssReceiverState::Off
                        | GnssReceiverState::Starting
                        | GnssReceiverState::ConfigurationFailed
                ) {
                    continue;
                }

                if is_rmc(&sentence) {
                    if let Ok(time) = parse_rmc_datetime(sentence.as_slice()) {
                        let now_ms = Instant::now().as_millis();
                        GNSS_TIME_UNIX_SECONDS.store(time.unix_seconds, Ordering::Relaxed);
                        GNSS_TIME_MILLISECONDS.store(time.milliseconds, Ordering::Relaxed);
                        GNSS_TIME_UPDATED_AT_MS.store(now_ms as u32, Ordering::Relaxed);
                        GNSS_TIME_VALID.store(true, Ordering::Release);
                    }
                    continue;
                }
                if !is_gga(&sentence) {
                    continue;
                }
                let now_ms = Instant::now().as_millis();
                let parsed = parse_gga(sentence.as_slice());
                let mut telemetry = GNSS_TELEMETRY.lock().await;
                // parse中にTurnOffされた場合も、古いGGAでOffを上書きしない。
                if matches!(
                    telemetry.state,
                    GnssReceiverState::Off
                        | GnssReceiverState::Starting
                        | GnssReceiverState::ConfigurationFailed
                ) {
                    continue;
                }
                match parsed {
                    Ok(data)
                        if data.fix_quality != FixQuality::Invalid
                            && data.latitude.is_some()
                            && data.longitude.is_some()
                            && data.altitude.is_some() =>
                    {
                        let encoded = encode_position(
                            data.latitude.unwrap(),
                            data.longitude.unwrap(),
                            data.altitude.unwrap(),
                        );
                        telemetry.state = GnssReceiverState::ValidFix;
                        telemetry.east = encoded.east;
                        telemetry.north = encoded.north;
                        telemetry.height = encoded.height;
                        telemetry.last_fix_at_ms = Some(now_ms);
                    }
                    Ok(_) => {
                        telemetry.state = GnssReceiverState::NoFix;
                        telemetry.east = GNSS_COORDINATE_NO_FIX;
                        telemetry.north = GNSS_COORDINATE_NO_FIX;
                        telemetry.height = GNSS_HEIGHT_NO_FIX;
                    }
                    Err(_) => {
                        telemetry.state = GnssReceiverState::InvalidSample;
                        telemetry.east = GNSS_COORDINATE_INVALID;
                        telemetry.north = GNSS_COORDINATE_INVALID;
                        telemetry.height = GNSS_HEIGHT_INVALID;
                    }
                }
            }
            Either::Second(_) => {
                let now_ms = Instant::now().as_millis();
                let mut telemetry = GNSS_TELEMETRY.lock().await;
                match telemetry.state {
                    GnssReceiverState::Starting
                        if telemetry.started_at_ms.is_none_or(|seen| {
                            now_ms.saturating_sub(seen) >= GNSS_STALE_TIMEOUT_MS
                        }) =>
                    {
                        telemetry.state = GnssReceiverState::ReceiverError;
                        telemetry.east = GNSS_COORDINATE_RECEIVER_ERROR;
                        telemetry.north = GNSS_COORDINATE_RECEIVER_ERROR;
                        telemetry.height = GNSS_HEIGHT_RECEIVER_ERROR;
                    }
                    GnssReceiverState::ReceiverDetected
                    | GnssReceiverState::NoFix
                    | GnssReceiverState::InvalidSample
                        if telemetry.last_receiver_at_ms.is_some_and(|seen| {
                            now_ms.saturating_sub(seen) >= GNSS_STALE_TIMEOUT_MS
                        }) =>
                    {
                        telemetry.state = GnssReceiverState::Stale;
                        telemetry.east = GNSS_COORDINATE_STALE;
                        telemetry.north = GNSS_COORDINATE_STALE;
                        telemetry.height = GNSS_HEIGHT_STALE;
                    }
                    GnssReceiverState::ValidFix
                        if telemetry.last_fix_at_ms.is_some_and(|fix| {
                            now_ms.saturating_sub(fix) >= GNSS_STALE_TIMEOUT_MS
                        }) =>
                    {
                        telemetry.state = GnssReceiverState::Stale;
                        telemetry.east = GNSS_COORDINATE_STALE;
                        telemetry.north = GNSS_COORDINATE_STALE;
                        telemetry.height = GNSS_HEIGHT_STALE;
                    }
                    _ => {}
                }
                if GNSS_TIME_VALID.load(Ordering::Acquire) {
                    let updated = u64::from(GNSS_TIME_UPDATED_AT_MS.load(Ordering::Relaxed));
                    if now_ms.saturating_sub(updated) >= GNSS_STALE_TIMEOUT_MS {
                        GNSS_TIME_VALID.store(false, Ordering::Release);
                    }
                }
            }
        }
    }
}

#[embassy_executor::task]
pub async fn gnss_time_response_task() {
    let mut ticker = Ticker::every(Duration::from_millis(GNSS_TIME_RESPONSE_POLL_MS));
    let mut last_response: Option<(u8, u64)> = None;
    loop {
        ticker.next().await;
        let request = {
            let cache = CAN_CACHE.lock().await;
            cache
                .time_request
                .value()
                .zip(cache.time_request.received_at_ms())
        };
        let Some((request_id, request_received_at_ms)) = request else {
            continue;
        };
        if last_response == Some((request_id, request_received_at_ms)) {
            continue;
        }
        if !GNSS_TIME_VALID.load(Ordering::Acquire) {
            continue;
        }

        let now_ms = Instant::now().as_millis();
        let updated_at_ms = u64::from(GNSS_TIME_UPDATED_AT_MS.load(Ordering::Relaxed));
        let age_ms = now_ms.saturating_sub(updated_at_ms);
        if age_ms > GNSS_STALE_TIMEOUT_MS {
            GNSS_TIME_VALID.store(false, Ordering::Release);
            continue;
        }
        let base_seconds = GNSS_TIME_UNIX_SECONDS.load(Ordering::Relaxed);
        let base_milliseconds = u64::from(GNSS_TIME_MILLISECONDS.load(Ordering::Relaxed));
        let total_milliseconds = base_milliseconds + age_ms;
        let Some(unix_seconds) = base_seconds.checked_add((total_milliseconds / 1_000) as u32)
        else {
            GNSS_TIME_VALID.store(false, Ordering::Release);
            continue;
        };
        let milliseconds = (total_milliseconds % 1_000) as u16;
        let response = CanTxRequest {
            message: CanTxMessage::TimeResponse {
                request_id,
                source: TimeSource::Gnss,
                unix_seconds,
                milliseconds,
            },
        };
        if CAN_TX_CHANNEL.try_send(response).is_ok() {
            last_response = Some((request_id, request_received_at_ms));
        }
    }
}
