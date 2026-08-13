#![no_std]
#![no_main]

use c99l_comboard::tasks::SdTimeSource;
use embassy_time::Timer;
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::{Mode, SdCard, VolumeIdx, VolumeManager};
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    gpio::{Level, Output, OutputConfig},
    interrupt::software::SoftwareInterruptControl,
    rtc_cntl::Rtc,
    spi::{self, master::Spi},
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

const HEADER: &[u8] = b"Time_ms,Id,Dlc,D0,D1,D2,D3,D4,D5,D6,D7";
const LINE_CAPACITY: usize = 128;
const EXPECTED_IDS: [u16; 13] = [
    0x011, 0x012, 0x020, 0x100, 0x101, 0x102, 0x103, 0x104, 0x105, 0x106, 0x107, 0x108, 0x109,
];

#[derive(Clone, Copy)]
enum RowError {
    Fields,
    Time,
    Id,
    Dlc,
    Data,
}

struct SegmentStats {
    index: u64,
    rows: u64,
    first_time_ms: Option<u64>,
    last_time_ms: Option<u64>,
    id_counts: [u64; EXPECTED_IDS.len()],
    other_id_rows: u64,
}

impl SegmentStats {
    const fn new() -> Self {
        Self {
            index: 0,
            rows: 0,
            first_time_ms: None,
            last_time_ms: None,
            id_counts: [0; EXPECTED_IDS.len()],
            other_id_rows: 0,
        }
    }

    fn reset(&mut self, index: u64) {
        *self = Self::new();
        self.index = index;
    }

    fn add(&mut self, time_ms: u64, id_index: Option<usize>) {
        if self.first_time_ms.is_none() {
            self.first_time_ms = Some(time_ms);
        }
        self.last_time_ms = Some(time_ms);
        self.rows += 1;
        if let Some(index) = id_index {
            self.id_counts[index] += 1;
        } else {
            self.other_id_rows += 1;
        }
    }
}

struct Stats {
    header_ok: bool,
    rows: u64,
    valid_rows: u64,
    malformed_rows: u64,
    overlong_malformed_lines: u64,
    bad_fields: u64,
    bad_time: u64,
    bad_id: u64,
    bad_dlc: u64,
    bad_data: u64,
    time_decreases: u64,
    segments: u64,
    previous_time_ms: Option<u64>,
    id_counts: [u64; EXPECTED_IDS.len()],
    other_id_rows: u64,
    last_segment: SegmentStats,
}

impl Stats {
    const fn new() -> Self {
        Self {
            header_ok: false,
            rows: 0,
            valid_rows: 0,
            malformed_rows: 0,
            overlong_malformed_lines: 0,
            bad_fields: 0,
            bad_time: 0,
            bad_id: 0,
            bad_dlc: 0,
            bad_data: 0,
            time_decreases: 0,
            segments: 0,
            previous_time_ms: None,
            id_counts: [0; EXPECTED_IDS.len()],
            other_id_rows: 0,
            last_segment: SegmentStats::new(),
        }
    }

    fn malformed(&mut self, error: RowError) {
        self.malformed_rows += 1;
        match error {
            RowError::Fields => self.bad_fields += 1,
            RowError::Time => self.bad_time += 1,
            RowError::Id => self.bad_id += 1,
            RowError::Dlc => self.bad_dlc += 1,
            RowError::Data => self.bad_data += 1,
        }
    }

    fn add(&mut self, time_ms: u64, id: u16) {
        if self
            .previous_time_ms
            .is_some_and(|previous| time_ms < previous)
        {
            self.time_decreases += 1;
            self.segments += 1;
            self.last_segment.reset(self.segments);
        } else if self.segments == 0 {
            self.segments = 1;
            self.last_segment.reset(1);
        }

        let id_index = EXPECTED_IDS.iter().position(|expected| *expected == id);
        if let Some(index) = id_index {
            self.id_counts[index] += 1;
        } else {
            self.other_id_rows += 1;
        }
        self.last_segment.add(time_ms, id_index);
        self.previous_time_ms = Some(time_ms);
        self.valid_rows += 1;
    }
}

fn parse_row(line: &[u8]) -> Result<(u64, u16), RowError> {
    let text = core::str::from_utf8(line).map_err(|_| RowError::Fields)?;
    let mut fields = text.split(',');

    let time_ms = fields
        .next()
        .ok_or(RowError::Fields)?
        .parse::<u64>()
        .map_err(|_| RowError::Time)?;
    let id_text = fields.next().ok_or(RowError::Fields)?;
    if id_text.len() != 3 {
        return Err(RowError::Id);
    }
    let id = u16::from_str_radix(id_text, 16).map_err(|_| RowError::Id)?;
    if id > 0x7ff {
        return Err(RowError::Id);
    }

    let dlc = fields
        .next()
        .ok_or(RowError::Fields)?
        .parse::<u8>()
        .map_err(|_| RowError::Dlc)?;
    if dlc > 8 {
        return Err(RowError::Dlc);
    }

    for _ in 0..8 {
        let byte = fields.next().ok_or(RowError::Fields)?;
        if byte.len() != 2 || u8::from_str_radix(byte, 16).is_err() {
            return Err(RowError::Data);
        }
    }
    if fields.next().is_some() {
        return Err(RowError::Fields);
    }

    Ok((time_ms, id))
}

fn process_line(stats: &mut Stats, first_line: bool, line: &[u8]) {
    let line = line.strip_suffix(b"\r").unwrap_or(line);
    if first_line {
        stats.header_ok = line == HEADER;
        return;
    }

    stats.rows += 1;
    match parse_row(line) {
        Ok((time_ms, id)) => stats.add(time_ms, id),
        Err(error) => stats.malformed(error),
    }
}

fn process_overlong(stats: &mut Stats, first_line: bool) {
    stats.overlong_malformed_lines += 1;
    if first_line {
        stats.header_ok = false;
    } else {
        stats.rows += 1;
        stats.malformed(RowError::Fields);
    }
}

fn print_stats(stats: &Stats) {
    println!(
        "CSV header={} rows={} valid={} malformed={} overlong_malformed={} bad_fields={} bad_time={} bad_id={} bad_dlc={} bad_data={}",
        stats.header_ok,
        stats.rows,
        stats.valid_rows,
        stats.malformed_rows,
        stats.overlong_malformed_lines,
        stats.bad_fields,
        stats.bad_time,
        stats.bad_id,
        stats.bad_dlc,
        stats.bad_data,
    );
    println!(
        "CSV segments={} time_decreases={} other_id_rows={}",
        stats.segments, stats.time_decreases, stats.other_id_rows,
    );
    println!(
        "CSV last_segment={} rows={} first_time_ms={:?} last_time_ms={:?} other_id_rows={}",
        stats.last_segment.index,
        stats.last_segment.rows,
        stats.last_segment.first_time_ms,
        stats.last_segment.last_time_ms,
        stats.last_segment.other_id_rows,
    );
    for (index, id) in EXPECTED_IDS.iter().enumerate() {
        println!(
            "CSV id=0x{:03X} total={} last_segment={}",
            id, stats.id_counts[index], stats.last_segment.id_counts[index],
        );
    }
}

#[esp_rtos::main]
async fn main(_spawner: embassy_executor::Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let mut gnss_en = Output::new(peripherals.GPIO13, Level::Low, OutputConfig::default());
    let mut m0 = Output::new(peripherals.GPIO9, Level::Low, OutputConfig::default());
    let mut m1 = Output::new(peripherals.GPIO10, Level::Low, OutputConfig::default());
    gnss_en.set_low();
    m0.set_low();
    m1.set_low();
    println!("safe outputs: GNSS_EN=LOW E220_M0=LOW E220_M1=LOW");

    let mut spi_bus = Spi::new(
        peripherals.SPI2,
        spi::master::Config::default()
            .with_frequency(Rate::from_khz(400))
            .with_mode(spi::Mode::_0),
    )
    .unwrap()
    .with_sck(peripherals.GPIO41)
    .with_mosi(peripherals.GPIO42)
    .with_miso(peripherals.GPIO40);
    let sd_cs = Output::new(peripherals.GPIO2, Level::High, OutputConfig::default());
    spi_bus.write(&[0xff; 10]).unwrap();

    let spi_device = ExclusiveDevice::new(spi_bus, sd_cs, embassy_time::Delay).unwrap();
    let sdcard = SdCard::new(spi_device, embassy_time::Delay);
    let mut card_ok = true;
    match sdcard.num_bytes() {
        Ok(size) => println!("SD card size={} bytes", size),
        Err(error) => {
            println!("SD initialization/size error: {:?}", error);
            card_ok = false;
        }
    }
    let fast_config = spi::master::Config::default()
        .with_frequency(Rate::from_mhz(1))
        .with_mode(spi::Mode::_0);
    if let Err(error) = sdcard.spi(|device| device.bus_mut().apply_config(&fast_config)) {
        println!("SD SPI frequency change error: {:?}", error);
        card_ok = false;
    }

    let rtc = Rtc::new(peripherals.LPWR);
    let volume_manager = VolumeManager::new(sdcard, SdTimeSource::new(rtc));
    let mut verify_ok = false;

    match volume_manager.open_volume(VolumeIdx(0)) {
        Ok(volume) => match volume.open_root_dir() {
            Ok(root) => match root.open_file_in_dir("CAN.CSV", Mode::ReadOnly) {
                Ok(file) => {
                    println!(
                        "CAN.CSV exists=true size={} bytes mode=ReadOnly",
                        file.length()
                    );
                    let mut stats = Stats::new();
                    let mut read_buffer = [0u8; 64];
                    let mut line = [0u8; LINE_CAPACITY];
                    let mut line_length = 0usize;
                    let mut overlong = false;
                    let mut first_line = true;
                    let mut read_ok = true;

                    loop {
                        let count = match file.read(&mut read_buffer) {
                            Ok(count) => count,
                            Err(error) => {
                                println!("CAN.CSV read error: {:?}", error);
                                read_ok = false;
                                break;
                            }
                        };
                        if count == 0 {
                            break;
                        }
                        for &byte in &read_buffer[..count] {
                            if byte == b'\n' {
                                if overlong {
                                    process_overlong(&mut stats, first_line);
                                } else {
                                    process_line(&mut stats, first_line, &line[..line_length]);
                                }
                                first_line = false;
                                line_length = 0;
                                overlong = false;
                            } else if !overlong {
                                if line_length < line.len() {
                                    line[line_length] = byte;
                                    line_length += 1;
                                } else {
                                    overlong = true;
                                }
                            }
                        }
                    }
                    if overlong {
                        process_overlong(&mut stats, first_line);
                    } else if line_length != 0 {
                        process_line(&mut stats, first_line, &line[..line_length]);
                    }

                    print_stats(&stats);
                    const REQUIRED_ACTIVE_IDS: [u16; 6] =
                        [0x100, 0x102, 0x103, 0x107, 0x108, 0x109];
                    let required_ids_present = REQUIRED_ACTIVE_IDS.iter().all(|required| {
                        EXPECTED_IDS
                            .iter()
                            .position(|id| id == required)
                            .is_some_and(|index| stats.last_segment.id_counts[index] != 0)
                    });
                    verify_ok = card_ok
                        && read_ok
                        && stats.header_ok
                        && stats.rows != 0
                        && stats.valid_rows == stats.rows
                        && stats.overlong_malformed_lines == 0
                        && stats.last_segment.rows != 0
                        && required_ids_present;
                    if let Err(error) = file.close() {
                        println!("CAN.CSV close error: {:?}", error);
                        verify_ok = false;
                    }
                }
                Err(error) => println!("CAN.CSV exists=false open error: {:?}", error),
            },
            Err(error) => println!("SD root directory open error: {:?}", error),
        },
        Err(error) => println!("SD volume open error: {:?}", error),
    }

    println!("SD_LOG_VERIFY {}", if verify_ok { "PASS" } else { "FAIL" });
    loop {
        Timer::after_secs(1).await;
    }
}
