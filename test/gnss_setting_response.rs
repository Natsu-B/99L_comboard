#![no_std]
#![no_main]

#[path = "../src/gnss/settings.rs"]
mod settings;

use core::fmt::Write as _;

use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::{
    Async,
    clock::CpuClock,
    gpio::{Input, InputConfig, Level, Output, OutputConfig},
    interrupt::software::SoftwareInterruptControl,
    timer::timg::TimerGroup,
    uart::{Config as UartConfig, DataBits, Parity, RxError, StopBits, Uart},
};
use esp_println::println;
use heapless::String;
use settings::{
    DYNAMIC_MODEL_AIRBORNE_4G, GLL_DELETE, GSA_DELETE, GST_ENABLE_UART1, GSV_DELETE, MEAS_RATE,
    QZSS_L1S_ENABLE, UART_BAUD, VTG_DELETE,
};

esp_bootloader_esp_idf::esp_app_desc!();

type GnssUart = Uart<'static, Async>;

const INITIAL_BAUD: u32 = 9_600;
const CONFIGURED_BAUD: u32 = 115_200;
const STARTUP_RX_MS: u64 = 500;
const COMMAND_RX_MS: u64 = 750;
const BAUD_SWITCH_DELAY_MS: u64 = 50;
const FINAL_RX_SECS: u64 = 3;
const RX_BUFFER_SIZE: usize = 128;

struct NamedCommand {
    name: &'static str,
    bytes: &'static [u8],
}

const COMMANDS: &[NamedCommand] = &[
    NamedCommand {
        name: "GLL_DELETE",
        bytes: GLL_DELETE,
    },
    NamedCommand {
        name: "GSA_DELETE",
        bytes: GSA_DELETE,
    },
    NamedCommand {
        name: "GSV_DELETE",
        bytes: GSV_DELETE,
    },
    NamedCommand {
        name: "VTG_DELETE",
        bytes: VTG_DELETE,
    },
    NamedCommand {
        name: "MEAS_RATE",
        bytes: MEAS_RATE,
    },
    NamedCommand {
        name: "QZSS_L1S_ENABLE",
        bytes: QZSS_L1S_ENABLE,
    },
    NamedCommand {
        name: "DYNAMIC_MODEL_AIRBORNE_4G",
        bytes: DYNAMIC_MODEL_AIRBORNE_4G,
    },
    NamedCommand {
        name: "GST_ENABLE_UART1",
        bytes: GST_ENABLE_UART1,
    },
    NamedCommand {
        name: "UART_BAUD",
        bytes: UART_BAUD,
    },
];

fn print_hex(bytes: &[u8]) {
    let mut text = String::<512>::new();
    for (index, byte) in bytes.iter().enumerate() {
        if index != 0 {
            let _ = text.push(' ');
        }
        let _ = write!(text, "{:02X}", byte);
    }
    println!("{}", text.as_str());
}

fn print_ascii(bytes: &[u8]) {
    let mut text = String::<RX_BUFFER_SIZE>::new();
    for &byte in bytes {
        let character = if byte.is_ascii_graphic() || byte == b' ' {
            byte as char
        } else {
            '.'
        };
        let _ = text.push(character);
    }
    println!("{}", text.as_str());
}

fn print_rx_chunk(bytes: &[u8]) {
    println!("RX chunk: {} bytes", bytes.len());
    println!("RX HEX:");
    print_hex(bytes);
    println!("RX ASCII:");
    print_ascii(bytes);
}

fn print_rx_error(error: RxError) {
    match error {
        RxError::FifoOverflowed => println!("GNSS UART FIFO overflow"),
        _ => println!("GNSS UART read error: {:?}", error),
    }
}

async fn receive_for(uart: &mut GnssUart, duration: Duration) {
    let deadline = Instant::now() + duration;
    let mut received_any = false;
    let mut rx_buf = [0u8; RX_BUFFER_SIZE];

    loop {
        if Instant::now() >= deadline {
            break;
        }

        match select(uart.read_async(&mut rx_buf), Timer::at(deadline)).await {
            Either::First(Ok(0)) => {}
            Either::First(Ok(length)) => {
                received_any = true;
                print_rx_chunk(&rx_buf[..length]);
                if length == rx_buf.len() {
                    println!("RX buffer full; continuing with the next chunk");
                }
            }
            Either::First(Err(error)) => print_rx_error(error),
            Either::Second(_) => break,
        }
    }

    if !received_any {
        println!("GNSS UART receive timeout");
        println!("RX: no data within timeout");
    }
    println!("RX wait finished");
}

fn drain_buffered_rx(uart: &mut GnssUart) {
    let mut rx_buf = [0u8; RX_BUFFER_SIZE];
    let mut drained = 0usize;

    loop {
        match uart.read_buffered(&mut rx_buf) {
            Ok(0) => break,
            Ok(length) => drained = drained.saturating_add(length),
            Err(error) => {
                print_rx_error(error);
                break;
            }
        }
    }

    println!("RX buffer drained: {} bytes", drained);
}

async fn write_all(uart: &mut GnssUart, mut bytes: &[u8]) -> bool {
    while !bytes.is_empty() {
        match uart.write_async(bytes).await {
            Ok(0) => {
                println!("GNSS UART write made no progress");
                return false;
            }
            Ok(written) => bytes = &bytes[written..],
            Err(error) => {
                println!("GNSS UART write error: {:?}", error);
                return false;
            }
        }
    }

    true
}

fn uart_config(baudrate: u32) -> UartConfig {
    UartConfig::default()
        .with_baudrate(baudrate)
        .with_data_bits(DataBits::_8)
        .with_parity(Parity::None)
        .with_stop_bits(StopBits::_1)
}

#[esp_rtos::main]
async fn main(_spawner: embassy_executor::Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let gnss_tx = Output::new(peripherals.GPIO14, Level::Low, OutputConfig::default());
    let gnss_rx = Input::new(peripherals.GPIO21, InputConfig::default());
    let mut gnss_en = Output::new(peripherals.GPIO13, Level::Low, OutputConfig::default());

    let mut uart = match Uart::new(peripherals.UART1, uart_config(INITIAL_BAUD)) {
        Ok(uart) => uart.with_rx(gnss_rx).with_tx(gnss_tx).into_async(),
        Err(error) => {
            println!("GNSS UART init error: {:?}", error);
            loop {
                Timer::after_secs(1).await;
            }
        }
    };

    println!("=== GNSS setting test ===");
    println!(
        "UART1 TX=GPIO14 RX=GPIO21 EN=GPIO13 initial_baud={} data=8 parity=None stop=1",
        INITIAL_BAUD
    );

    gnss_en.set_high();
    let mut active_baud = INITIAL_BAUD;
    println!("Startup RX ({} ms at {} baud)", STARTUP_RX_MS, INITIAL_BAUD);
    receive_for(&mut uart, Duration::from_millis(STARTUP_RX_MS)).await;
    drain_buffered_rx(&mut uart);

    for (index, command) in COMMANDS.iter().enumerate() {
        println!("COMMAND {}: {}", index + 1, command.name);
        println!("TX length: {}", command.bytes.len());
        println!("TX HEX:");
        print_hex(command.bytes);

        if !write_all(&mut uart, command.bytes).await {
            println!("TX failed; continuing with the next command");
            continue;
        }
        if let Err(error) = uart.flush_async().await {
            println!("GNSS UART flush error: {:?}", error);
            continue;
        }
        println!("TX write and flush complete");

        if command.name == "UART_BAUD" {
            Timer::after_millis(BAUD_SWITCH_DELAY_MS).await;

            let mut old_baud_rx = [0u8; RX_BUFFER_SIZE];
            match uart.read_buffered(&mut old_baud_rx) {
                Ok(0) => println!("RX at old baud: no buffered data"),
                Ok(length) => {
                    println!("RX captured at old baud before switch:");
                    print_rx_chunk(&old_baud_rx[..length]);
                }
                Err(error) => print_rx_error(error),
            }

            if let Err(error) = uart.apply_config(&uart_config(CONFIGURED_BAUD)) {
                println!(
                    "GNSS UART config error ({} baud): {:?}",
                    CONFIGURED_BAUD, error
                );
                continue;
            }
            active_baud = CONFIGURED_BAUD;
            println!("ESP32 UART switched to {} baud", CONFIGURED_BAUD);
        }

        receive_for(&mut uart, Duration::from_millis(COMMAND_RX_MS)).await;
    }

    println!(
        "Final RX ({} seconds at {} baud)",
        FINAL_RX_SECS, active_baud
    );
    receive_for(&mut uart, Duration::from_secs(FINAL_RX_SECS)).await;
    println!("=== GNSS setting test finished ===");

    loop {
        Timer::after_secs(1).await;
    }
}
