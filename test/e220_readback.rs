#![no_std]
#![no_main]

use core::fmt::Write as _;

use embassy_futures::select::{Either, Either3, select, select3};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use esp_backtrace as _;
use esp_hal::{
    Async,
    clock::CpuClock,
    gpio::{Input, InputConfig, Level, Output, OutputConfig},
    interrupt::software::SoftwareInterruptControl,
    timer::timg::TimerGroup,
    uart::{Config as UartConfig, DataBits, Parity, StopBits, Uart},
};
use esp_println::println;
use heapless::String;

esp_bootloader_esp_idf::esp_app_desc!();

type E220Uart = Uart<'static, Async>;

const READ_COMMAND: &[u8] = &[0xC1, 0x00, 0x08];
const EXPECTED_RESPONSE: &[u8] = &[
    0xC1, 0x00, 0x08, 0x00, 0x00, 0xEC, 0x81, 0x04, 0xC3, 0x00, 0x00,
];
const AUX_TIMEOUT: Duration = Duration::from_secs(1);
const RESPONSE_TIMEOUT: Duration = Duration::from_secs(2);
const RESPONSE_QUIET: Duration = Duration::from_millis(200);
const RESPONSE_CAPACITY: usize = 64;

async fn wait_for_aux_high(aux: &mut Input<'static>) -> bool {
    aux.is_high() || (with_timeout(AUX_TIMEOUT, aux.wait_for_high()).await.is_ok() && aux.is_high())
}

fn drain_buffered_rx(uart: &mut E220Uart) -> bool {
    let mut buffer = [0u8; RESPONSE_CAPACITY];
    let mut drained = 0usize;
    loop {
        match uart.read_buffered(&mut buffer) {
            Ok(0) => break,
            Ok(count) => drained = drained.saturating_add(count),
            Err(error) => {
                println!("E220 RX drain error: {:?}", error);
                return false;
            }
        }
    }
    println!("E220 RX drained: {} bytes", drained);
    true
}

async fn write_read_command(uart: &mut E220Uart) -> bool {
    let mut remaining = READ_COMMAND;
    while !remaining.is_empty() {
        match uart.write_async(remaining).await {
            Ok(0) => {
                println!("E220 TX made no progress");
                return false;
            }
            Ok(count) => remaining = &remaining[count..],
            Err(error) => {
                println!("E220 TX error: {:?}", error);
                return false;
            }
        }
    }
    match uart.flush_async().await {
        Ok(()) => true,
        Err(error) => {
            println!("E220 TX flush error: {:?}", error);
            false
        }
    }
}

async fn receive_response(uart: &mut E220Uart) -> Option<([u8; RESPONSE_CAPACITY], usize)> {
    let deadline = Instant::now() + RESPONSE_TIMEOUT;
    let mut response = [0u8; RESPONSE_CAPACITY];
    let mut count = 0usize;

    match select(uart.read_async(&mut response), Timer::at(deadline)).await {
        Either::First(Ok(0)) => {}
        Either::First(Ok(received)) => count = received,
        Either::First(Err(error)) => {
            println!("E220 RX error: {:?}", error);
            return None;
        }
        Either::Second(()) => return Some((response, 0)),
    }

    while count < response.len() && Instant::now() < deadline {
        match select3(
            uart.read_async(&mut response[count..]),
            Timer::at(deadline),
            Timer::after(RESPONSE_QUIET),
        )
        .await
        {
            Either3::First(Ok(0)) => {}
            Either3::First(Ok(received)) => count += received,
            Either3::First(Err(error)) => {
                println!("E220 RX error: {:?}", error);
                return None;
            }
            Either3::Second(()) | Either3::Third(()) => break,
        }
    }

    Some((response, count))
}

fn print_response(bytes: &[u8]) {
    let mut hex = String::<{ RESPONSE_CAPACITY * 3 }>::new();
    for (index, byte) in bytes.iter().enumerate() {
        if index != 0 {
            let _ = hex.push(' ');
        }
        let _ = write!(hex, "{:02X}", byte);
    }
    println!("E220 response count: {}", bytes.len());
    println!("E220 response HEX: {}", hex.as_str());
}

async fn readback_session(uart: &mut E220Uart, aux: &mut Input<'static>) -> bool {
    if !wait_for_aux_high(aux).await {
        println!("E220 AUX timeout entering configuration mode");
        return false;
    }
    Timer::after_millis(40).await;

    if !drain_buffered_rx(uart) {
        return false;
    }

    println!("E220 TX HEX: C1 00 08");
    if !write_read_command(uart).await {
        return false;
    }
    if !wait_for_aux_high(aux).await {
        println!("E220 AUX timeout after read command");
        return false;
    }

    let Some((response, count)) = receive_response(uart).await else {
        return false;
    };
    print_response(&response[..count]);
    if response[..count] != EXPECTED_RESPONSE[..] {
        println!(
            "E220 readback: invalid response expected_length={} actual={}",
            EXPECTED_RESPONSE.len(),
            count
        );
        return false;
    }
    true
}

async fn receive_probe(uart: &mut E220Uart) {
    if let Err(error) = uart.apply_config(&UartConfig::default().with_baudrate(115_200)) {
        println!("E220 receive probe UART config error: {:?}", error);
        return;
    }
    println!("E220 receive probe: communication mode baud=115200 duration=15s");
    let deadline = Instant::now() + Duration::from_secs(15);
    let mut bytes = 0usize;
    let mut buffer = [0u8; 32];
    while Instant::now() < deadline {
        match select(uart.read_async(&mut buffer), Timer::at(deadline)).await {
            Either::First(Ok(count)) => {
                bytes += count;
                print_response(&buffer[..count]);
            }
            Either::First(Err(error)) => {
                println!("E220 receive probe RX error: {:?}", error);
                break;
            }
            Either::Second(()) => break,
        }
    }
    println!("E220 receive probe bytes={}", bytes);
}

fn uart_config() -> UartConfig {
    UartConfig::default()
        .with_baudrate(9_600)
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

    let lora_tx = Output::new(peripherals.GPIO11, Level::Low, OutputConfig::default());
    let lora_rx = Input::new(peripherals.GPIO12, InputConfig::default());
    let mut aux = Input::new(peripherals.GPIO8, InputConfig::default());
    let mut m0 = Output::new(peripherals.GPIO9, Level::Low, OutputConfig::default());
    let mut m1 = Output::new(peripherals.GPIO10, Level::Low, OutputConfig::default());
    let _gnss_en = Output::new(peripherals.GPIO13, Level::Low, OutputConfig::default());

    let mut uart = match Uart::new(peripherals.UART2, uart_config()) {
        Ok(uart) => uart.with_rx(lora_rx).with_tx(lora_tx).into_async(),
        Err(error) => {
            println!("E220 UART init error: {:?}", error);
            loop {
                Timer::after_secs(1).await;
            }
        }
    };

    println!("=== E220 read-only register readback ===");
    println!("UART2 TX=GPIO11 RX=GPIO12 AUX=GPIO8 M0=GPIO9 M1=GPIO10 baud=9600");
    println!("GNSS_EN GPIO13=LOW");
    m0.set_high();
    m1.set_high();
    let passed = readback_session(&mut uart, &mut aux).await;

    // 設定モードのまま専用診断を終了しない。以後は書込みも再試行もしない。
    m0.set_low();
    m1.set_low();
    Timer::after_millis(40).await;
    println!("E220 M0/M1 restored LOW");
    println!(
        "E220 readback result: {}",
        if passed { "PASS" } else { "FAIL" }
    );
    receive_probe(&mut uart).await;
    println!("=== E220 readback finished ===");

    loop {
        Timer::after_secs(1).await;
    }
}
