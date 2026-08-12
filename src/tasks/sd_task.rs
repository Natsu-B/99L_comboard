use core::{fmt::Write, sync::atomic::Ordering};

use embassy_futures::select::{Either3, select3};
use embassy_time::{Delay, Duration, Ticker};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::{Mode, SdCard, TimeSource, Timestamp, VolumeIdx, VolumeManager};
use esp_hal::{Blocking, gpio::Output, rtc_cntl::Rtc, spi::master::Spi};
use heapless::String;

use crate::{
    constants::{BUF_SIZE, SD_FLUSH_INTERVAL_SECS},
    state::{
        HAS_UNFLUSHED_DATA, LOGGING_ACTIVE, LOGGING_REQUESTED, RAW_CAN_LOG_CHANNEL,
        SD_DROPPED_ROW_COUNT, SD_FLUSH_SIGNAL, SD_HAS_ERROR, SD_WRITE_ERROR_COUNT,
    },
};

type SdSpiDevice = ExclusiveDevice<Spi<'static, Blocking>, Output<'static>, Delay>;
type SdBlockDevice = SdCard<SdSpiDevice, Delay>;
pub type SdVolumeManager = VolumeManager<SdBlockDevice, SdTimeSource>;

pub struct SdTimeSource {
    timer: Rtc<'static>,
}

impl SdTimeSource {
    pub fn new(timer: Rtc<'static>) -> Self {
        Self { timer }
    }

    fn current_time(&self) -> u64 {
        self.timer.current_time_us()
    }
}

static TZ: jiff::tz::TimeZone = jiff::tz::get!("UTC");

impl TimeSource for SdTimeSource {
    fn get_timestamp(&self) -> Timestamp {
        let now_us = self.current_time();
        let now = jiff::Timestamp::from_microsecond(now_us as i64)
            .unwrap_or_else(|_| jiff::Timestamp::from_second(0).unwrap())
            .to_zoned(TZ.clone());
        Timestamp {
            year_since_1970: (now.year() - 1970).unsigned_abs() as u8,
            zero_indexed_month: now.month().wrapping_sub(1) as u8,
            zero_indexed_day: now.day().wrapping_sub(1) as u8,
            hours: now.hour() as u8,
            minutes: now.minute() as u8,
            seconds: now.second() as u8,
        }
    }
}

fn mark_sd_error(sd_logging_led: &mut Output<'static>) {
    SD_HAS_ERROR.store(true, Ordering::Relaxed);
    LOGGING_ACTIVE.store(false, Ordering::Relaxed);
    sd_logging_led.set_low();
}

#[embassy_executor::task]
pub async fn sd_write_task(
    volume_mgr: &'static mut SdVolumeManager,
    mut sd_logging_led: Output<'static>,
) {
    sd_logging_led.set_low();
    LOGGING_ACTIVE.store(false, Ordering::Relaxed);
    SD_HAS_ERROR.store(false, Ordering::Relaxed);

    let volume0 = match volume_mgr.open_volume(VolumeIdx(0)) {
        Ok(volume) => volume,
        Err(error) => {
            esp_println::println!("SD open_volume error: {:?}", error);
            mark_sd_error(&mut sd_logging_led);
            return;
        }
    };
    let root_dir = match volume0.open_root_dir() {
        Ok(directory) => directory,
        Err(error) => {
            esp_println::println!("SD open_root_dir error: {:?}", error);
            mark_sd_error(&mut sd_logging_led);
            return;
        }
    };
    let can_file = match root_dir.open_file_in_dir("CAN.CSV", Mode::ReadWriteCreateOrAppend) {
        Ok(file) => file,
        Err(error) => {
            esp_println::println!("SD CAN.CSV open error: {:?}", error);
            mark_sd_error(&mut sd_logging_led);
            return;
        }
    };
    if can_file.length() == 0 {
        if can_file
            .write(b"Time_ms,Id,Dlc,D0,D1,D2,D3,D4,D5,D6,D7\n")
            .and_then(|_| can_file.flush())
            .is_err()
        {
            mark_sd_error(&mut sd_logging_led);
            SD_WRITE_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
            return;
        }
    } else if can_file.seek_from_end(0).is_err() {
        mark_sd_error(&mut sd_logging_led);
        return;
    }

    let mut buffer = [0u8; BUF_SIZE];
    let mut cursor = 0usize;
    let mut flush_ticker = Ticker::every(Duration::from_secs(SD_FLUSH_INTERVAL_SECS));
    loop {
        let flush_requested = match select3(
            RAW_CAN_LOG_CHANNEL.receive(),
            flush_ticker.next(),
            SD_FLUSH_SIGNAL.wait(),
        )
        .await
        {
            Either3::First(record) => {
                let logging = LOGGING_REQUESTED.load(Ordering::Relaxed)
                    && !SD_HAS_ERROR.load(Ordering::Relaxed);
                LOGGING_ACTIVE.store(logging, Ordering::Relaxed);
                if logging {
                    sd_logging_led.set_high();
                    let mut line: String<128> = String::new();
                    if writeln!(
                        line,
                        "{},{:03X},{},{:02X},{:02X},{:02X},{:02X},{:02X},{:02X},{:02X},{:02X}",
                        record.received_at_ms,
                        record.identifier,
                        record.data_length,
                        record.data[0],
                        record.data[1],
                        record.data[2],
                        record.data[3],
                        record.data[4],
                        record.data[5],
                        record.data[6],
                        record.data[7],
                    )
                    .is_err()
                    {
                        SD_DROPPED_ROW_COUNT.fetch_add(1, Ordering::Relaxed);
                    } else {
                        let bytes = line.as_bytes();
                        if cursor + bytes.len() > buffer.len() {
                            if can_file.write(&buffer[..cursor]).is_err() {
                                mark_sd_error(&mut sd_logging_led);
                                SD_WRITE_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
                            }
                            cursor = 0;
                        }
                        if cursor + bytes.len() <= buffer.len() {
                            buffer[cursor..cursor + bytes.len()].copy_from_slice(bytes);
                            cursor += bytes.len();
                        } else {
                            SD_DROPPED_ROW_COUNT.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                } else {
                    sd_logging_led.set_low();
                }
                false
            }
            Either3::Second(_) | Either3::Third(_) => true,
        };

        if flush_requested || !LOGGING_REQUESTED.load(Ordering::Relaxed) {
            if cursor > 0 {
                if can_file.write(&buffer[..cursor]).is_err() {
                    mark_sd_error(&mut sd_logging_led);
                    SD_WRITE_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
                } else {
                    cursor = 0;
                }
            }
            if can_file.flush().is_err() {
                mark_sd_error(&mut sd_logging_led);
                SD_WRITE_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
            }
        }
        HAS_UNFLUSHED_DATA.store(cursor > 0, Ordering::Relaxed);
    }
}
