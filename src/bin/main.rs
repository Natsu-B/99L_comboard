#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::sync::atomic::Ordering;

use c99l_comboard::{
    state::{
        CAN_HEALTH, CAN_REC, CAN_RX_ERROR_COUNT, CAN_RX_SUCCESS_COUNT, CAN_TEC, CAN_TX_ERROR_COUNT,
        CAN_TX_SUCCESS_COUNT, GNSS_CHANNEL_DROP_COUNT, GNSS_RX_ERROR_COUNT,
        GNSS_SETTING_ERROR_COUNT, GNSS_TELEMETRY, HAS_UNFLUSHED_DATA, IS_CAN_ERROR, LOGGING_ACTIVE,
        LOGGING_REQUESTED, LORA_AUX_TIMEOUT_COUNT, LORA_COMMAND_DROP_COUNT, LORA_RX_BYTE_COUNT,
        LORA_RX_ERROR_COUNT, LORA_RX_SUCCESS_COUNT, LORA_TX_ERROR_COUNT, LORA_TX_QUEUE_DROP_COUNT,
        LORA_TX_SUCCESS_COUNT, RAW_CAN_LOG_DROPPED_COUNT, SD_DROPPED_ROW_COUNT, SD_HAS_ERROR,
        SD_WRITE_ERROR_COUNT,
    },
    tasks::{
        SdTimeSource, SdVolumeManager, can_communication_task, command_process_task,
        gnss_manager_task, lora_rx_task, lora_tx_task, parse_gnss_task, sd_write_task,
    },
};
use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::{SdCard, VolumeManager};
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    gpio::{Input, InputConfig, Level, Output, OutputConfig},
    interrupt::software::SoftwareInterruptControl,
    rtc_cntl::Rtc,
    spi::{self, master::Spi},
    system::Stack,
    time::Rate,
    timer::timg::TimerGroup,
    twai::{self, BaudRate, TwaiMode, filter::SingleStandardFilter},
    uart::{Config as UartConfig, DataBits, Parity, StopBits, Uart},
};
use esp_println::println;
use esp_rtos::embassy::Executor;
use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "the second-core stack is intentionally initialized during startup"
)]
#[esp_rtos::main]
async fn main(spawner0: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();

    println!("boot");
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    static APP_CORE_STACK: StaticCell<Stack<16384>> = StaticCell::new();
    let app_core_stack = APP_CORE_STACK.init_with(Stack::new);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let can_tx = Output::new(peripherals.GPIO7, Level::Low, OutputConfig::default());
    let can_rx = Input::new(peripherals.GPIO16, InputConfig::default());
    let mut led1 = Output::new(peripherals.GPIO6, Level::Low, OutputConfig::default());
    let sd_logging_led = Output::new(peripherals.GPIO15, Level::Low, OutputConfig::default());
    let lora_tx = Output::new(peripherals.GPIO11, Level::Low, OutputConfig::default());
    let mut m0 = Output::new(peripherals.GPIO9, Level::Low, OutputConfig::default());
    let mut m1 = Output::new(peripherals.GPIO10, Level::Low, OutputConfig::default());
    let gnss_tx = Output::new(peripherals.GPIO14, Level::Low, OutputConfig::default());
    let gnss_en = Output::new(peripherals.GPIO13, Level::Low, OutputConfig::default());
    let aux_pin = Input::new(peripherals.GPIO8, InputConfig::default());
    let lora_rx = Input::new(peripherals.GPIO12, InputConfig::default());
    let gnss_rx = Input::new(peripherals.GPIO21, InputConfig::default());

    m0.set_low();
    m1.set_low();
    // --- SPI ---
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
    // SDカードをSPI modeに入れるため、CS High のまま 74 clock 以上送る
    // 0xFF 10 bytes = 80 clocks
    spi_bus.write(&[0xFF; 10]).unwrap();

    // --- init sd ---
    let spi_dev = ExclusiveDevice::new(spi_bus, sd_cs, Delay).unwrap();
    let rtc = Rtc::new(peripherals.LPWR);
    let sd_timer = SdTimeSource::new(rtc);
    let sdcard = SdCard::new(spi_dev, Delay);
    match sdcard.num_bytes() {
        Ok(sd_size) => {
            println!("SD card initialized");
            println!("SD card size: {} bytes", sd_size);

            let fast_config = spi::master::Config::default()
                .with_frequency(Rate::from_mhz(1))
                .with_mode(spi::Mode::_0);
            match sdcard.spi(|device| device.bus_mut().apply_config(&fast_config)) {
                Ok(()) => println!("SD SPI frequency set to 1 MHz"),
                Err(e) => println!("SD SPI frequency change error: {:?}", e),
            }
        }
        Err(e) => println!("SD card initialization/size error: {:?}", e),
    }
    static VOLUME_MGR: StaticCell<SdVolumeManager> = StaticCell::new();
    let volume_mgr = VOLUME_MGR.init(VolumeManager::new(sdcard, sd_timer));

    let uart_config1 = UartConfig::default()
        .with_baudrate(9600)
        .with_data_bits(DataBits::_8)
        .with_parity(Parity::None)
        .with_stop_bits(StopBits::_1);
    let mut uart1 = Uart::new(peripherals.UART1, uart_config1)
        .unwrap()
        .with_rx(gnss_rx)
        .with_tx(gnss_tx)
        .into_async();

    let uart_config1_fast = UartConfig::default()
        .with_baudrate(115_200)
        .with_data_bits(DataBits::_8)
        .with_parity(Parity::None)
        .with_stop_bits(StopBits::_1);
    if let Err(e) = uart1.apply_config(&uart_config1_fast) {
        esp_println::println!("UART config error (115200baud rate): {:?}", e);
    }

    spawner0.spawn(gnss_manager_task(uart1, gnss_en).unwrap());
    // Keep blocking SD traffic off the core reserved for LoRa and optional CAN tasks.
    spawner0.spawn(sd_write_task(volume_mgr, sd_logging_led).unwrap());

    esp_rtos::start_second_core(
        peripherals.CPU_CTRL,
        sw_int.software_interrupt1,
        app_core_stack,
        move || {
            static EXECUTOR: StaticCell<Executor> = StaticCell::new();
            let executor = EXECUTOR.init(Executor::new());

            executor.run(|spawner| {
                let mut can_config = twai::TwaiConfiguration::new(
                    peripherals.TWAI0,
                    can_rx,
                    can_tx,
                    BaudRate::B125K,
                    TwaiMode::Normal,
                )
                .into_async();
                can_config.set_filter(const {
                    SingleStandardFilter::new(b"0xxxxxxxxxx", b"x", [b"xxxxxxxx", b"xxxxxxxx"])
                });
                let can = can_config.start();

                let uart_config2 = UartConfig::default()
                    .with_baudrate(115200)
                    .with_data_bits(DataBits::_8)
                    .with_parity(Parity::None)
                    .with_stop_bits(StopBits::_1);
                let uart2 = Uart::new(peripherals.UART2, uart_config2)
                    .unwrap()
                    .with_rx(lora_rx)
                    .with_tx(lora_tx)
                    .into_async();
                let (lora_uart_rx, lora_uart_tx) = uart2.split();

                spawner.spawn(can_communication_task(can).unwrap());
                spawner.spawn(command_process_task().unwrap());
                spawner.spawn(parse_gnss_task().unwrap());
                spawner.spawn(lora_rx_task(lora_uart_rx).unwrap());
                spawner.spawn(lora_tx_task(lora_uart_tx, aux_pin).unwrap());
            });
        },
    );

    let mut hwstat_ticks = 0u8;
    loop {
        if IS_CAN_ERROR.load(Ordering::Relaxed) {
            led1.toggle();
        } else {
            led1.set_low();
        }

        hwstat_ticks += 1;
        if hwstat_ticks == 100 {
            hwstat_ticks = 0;
            let gnss = *GNSS_TELEMETRY.lock().await;
            println!(
                "HWSTAT can_err={} can_health={} tec={} rec={} can_tx_ok={} can_rx_ok={} can_tx_err={} can_rx_err={} lora_tx_ok={} lora_rx_ok={} lora_rx_bytes={} lora_tx_err={} lora_rx_err={} lora_aux_timeout={} lora_tx_qdrop={} lora_cmd_drop={} sd_req={} sd_active={} sd_error={} sd_dirty={} sd_write_err={} sd_row_drop={} can_log_drop={} gnss={:?} east={} north={} height={} gnss_setting_err={} gnss_rx_err={} gnss_drop={}",
                IS_CAN_ERROR.load(Ordering::Relaxed),
                CAN_HEALTH.load(Ordering::Relaxed),
                CAN_TEC.load(Ordering::Relaxed),
                CAN_REC.load(Ordering::Relaxed),
                CAN_TX_SUCCESS_COUNT.load(Ordering::Relaxed),
                CAN_RX_SUCCESS_COUNT.load(Ordering::Relaxed),
                CAN_TX_ERROR_COUNT.load(Ordering::Relaxed),
                CAN_RX_ERROR_COUNT.load(Ordering::Relaxed),
                LORA_TX_SUCCESS_COUNT.load(Ordering::Relaxed),
                LORA_RX_SUCCESS_COUNT.load(Ordering::Relaxed),
                LORA_RX_BYTE_COUNT.load(Ordering::Relaxed),
                LORA_TX_ERROR_COUNT.load(Ordering::Relaxed),
                LORA_RX_ERROR_COUNT.load(Ordering::Relaxed),
                LORA_AUX_TIMEOUT_COUNT.load(Ordering::Relaxed),
                LORA_TX_QUEUE_DROP_COUNT.load(Ordering::Relaxed),
                LORA_COMMAND_DROP_COUNT.load(Ordering::Relaxed),
                LOGGING_REQUESTED.load(Ordering::Relaxed),
                LOGGING_ACTIVE.load(Ordering::Relaxed),
                SD_HAS_ERROR.load(Ordering::Relaxed),
                HAS_UNFLUSHED_DATA.load(Ordering::Relaxed),
                SD_WRITE_ERROR_COUNT.load(Ordering::Relaxed),
                SD_DROPPED_ROW_COUNT.load(Ordering::Relaxed),
                RAW_CAN_LOG_DROPPED_COUNT.load(Ordering::Relaxed),
                gnss.state,
                gnss.east,
                gnss.north,
                gnss.height,
                GNSS_SETTING_ERROR_COUNT.load(Ordering::Relaxed),
                GNSS_RX_ERROR_COUNT.load(Ordering::Relaxed),
                GNSS_CHANNEL_DROP_COUNT.load(Ordering::Relaxed),
            );
        }

        Timer::after(Duration::from_millis(100)).await;
    }
}
