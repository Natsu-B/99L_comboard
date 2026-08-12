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
    gnss::{
        FixQuality, gnss_setting, parse_gga,
        telemetry::{
            GNSS_COORDINATE_INVALID, GNSS_COORDINATE_NO_FIX, GNSS_COORDINATE_RECEIVER_ERROR,
            GNSS_COORDINATE_STALE, GNSS_COORDINATE_UNAVAILABLE, GNSS_HEIGHT_INVALID,
            GNSS_HEIGHT_NO_FIX, GNSS_HEIGHT_RECEIVER_ERROR, GNSS_HEIGHT_STALE,
            GNSS_HEIGHT_UNAVAILABLE, encode_position,
        },
    },
    state::{
        GNSS_CHANNEL, GNSS_CHANNEL_DROP_COUNT, GNSS_CMD_CHANNEL, GNSS_RX_ERROR_COUNT,
        GNSS_SETTING_ERROR_COUNT, GNSS_TELEMETRY, GnssCommand, GnssReceiverState,
    },
};

const GNSS_STALE_TIMEOUT_MS: u64 = 3_000;

async fn set_receiver_state(state: GnssReceiverState) {
    GNSS_TELEMETRY.lock().await.state = state;
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
                                if !matches!(telemetry.state, GnssReceiverState::ValidFix) {
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
                    set_receiver_state(GnssReceiverState::Starting).await;
                    gnss_en.set_high();
                    let config_9600 = UartConfig::default().with_baudrate(9_600);
                    if uart.apply_config(&config_9600).is_err() {
                        GNSS_SETTING_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
                        set_receiver_state(GnssReceiverState::ConfigurationFailed).await;
                        gnss_en.set_low();
                        continue;
                    }
                    Timer::after(Duration::from_millis(500)).await;
                    if let Err(error) = gnss_setting(&mut uart).await {
                        GNSS_SETTING_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
                        println!("GNSS setting failed: {:?}", error);
                        set_receiver_state(GnssReceiverState::ConfigurationFailed).await;
                        gnss_en.set_low();
                        continue;
                    }
                    Timer::after(Duration::from_millis(50)).await;
                    if uart
                        .apply_config(&UartConfig::default().with_baudrate(115_200))
                        .is_err()
                    {
                        GNSS_SETTING_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
                        set_receiver_state(GnssReceiverState::ConfigurationFailed).await;
                        gnss_en.set_low();
                        continue;
                    }
                    is_on = true;
                }
                GnssCommand::TurnOff => {
                    gnss_en.set_low();
                    is_on = false;
                    let mut telemetry = GNSS_TELEMETRY.lock().await;
                    telemetry.state = GnssReceiverState::Off;
                    telemetry.east = GNSS_COORDINATE_UNAVAILABLE;
                    telemetry.north = GNSS_COORDINATE_UNAVAILABLE;
                    telemetry.height = GNSS_HEIGHT_UNAVAILABLE;
                }
            },
        }
    }
}

fn is_gga(sentence: &[u8]) -> bool {
    sentence.windows(3).any(|window| window == b"GGA")
}

#[embassy_executor::task]
pub async fn parse_gnss_task() {
    let mut stale_ticker = Ticker::every(Duration::from_secs(1));
    loop {
        match select(GNSS_CHANNEL.receive(), stale_ticker.next()).await {
            Either::First(sentence) => {
                if !is_gga(&sentence) {
                    continue;
                }
                let now_ms = Instant::now().as_millis();
                match parse_gga(sentence.as_slice()) {
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
                        let mut telemetry = GNSS_TELEMETRY.lock().await;
                        telemetry.state = GnssReceiverState::ValidFix;
                        telemetry.east = encoded.east;
                        telemetry.north = encoded.north;
                        telemetry.height = encoded.height;
                        telemetry.last_fix_at_ms = Some(now_ms);
                    }
                    Ok(_) => {
                        let mut telemetry = GNSS_TELEMETRY.lock().await;
                        telemetry.state = GnssReceiverState::NoFix;
                        telemetry.east = GNSS_COORDINATE_NO_FIX;
                        telemetry.north = GNSS_COORDINATE_NO_FIX;
                        telemetry.height = GNSS_HEIGHT_NO_FIX;
                    }
                    Err(_) => {
                        let mut telemetry = GNSS_TELEMETRY.lock().await;
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
                        if telemetry.last_receiver_at_ms.is_none_or(|seen| {
                            now_ms.saturating_sub(seen) >= GNSS_STALE_TIMEOUT_MS
                        }) =>
                    {
                        telemetry.east = GNSS_COORDINATE_RECEIVER_ERROR;
                        telemetry.north = GNSS_COORDINATE_RECEIVER_ERROR;
                        telemetry.height = GNSS_HEIGHT_RECEIVER_ERROR;
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
            }
        }
    }
}
