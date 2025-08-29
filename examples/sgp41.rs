#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::AnyPin;
use esp_hal::i2c;
use esp_hal::rmt::{ChannelCreator, Rmt};
use esp_hal::time::Rate;
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::Async;
use log::info;
use static_cell::StaticCell;

use s3zero_hal::rgb::Rgb;
use s3zero_hal::sgp41;
use s3zero_hal::ws2812_rmt_single::{Ws2812Async, RMT_FREQ_MHZ};

static I2C_BUS: StaticCell<Mutex<NoopRawMutex, i2c::master::I2c<Async>>> = StaticCell::new();

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[embassy_executor::task]
async fn led_generic(chan: ChannelCreator<Async, 0>, gpio: AnyPin<'static>) {
    let mut ws2812 = Ws2812Async::new(chan, gpio).expect("Error initialising Ws2812");
    loop {
        for c in [Rgb::new(10, 0, 0), Rgb::new(0, 10, 0), Rgb::new(0, 0, 10)] {
            // log::info!("{c:?}");
            ws2812.set(c).await.expect("Error setting Ws2812");
            Timer::after(Duration::from_secs(1)).await;
        }
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.5.0

    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    let delay = Delay;

    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    info!("Embassy initialized!");

    // Initialize RMT peripheral
    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(RMT_FREQ_MHZ))
        .expect("Error initialising RMT")
        .into_async();

    spawner
        .spawn(led_generic(rmt.channel0, peripherals.GPIO21.into()))
        .expect("Error spawning led task");

    // Initialise I2C
    let i2c_config = i2c::master::Config::default().with_frequency(Rate::from_khz(100));
    let mut i2c = i2c::master::I2c::new(peripherals.I2C0, i2c_config)
        .expect("Error initailising I2C")
        .with_scl(peripherals.GPIO4)
        .with_sda(peripherals.GPIO5)
        .into_async();

    // I2C Bus Scan
    for addr in 0..=127 {
        if let Ok(_) = i2c.write_async(addr, &[0]).await {
            info!("Found I2C device at address: 0x{:02x}", addr);
        }
    }

    // Create shared I2C bus
    let i2c_bus = I2C_BUS.init(Mutex::new(i2c));

    let mut sgp41 = sgp41::Sgp41::new(I2cDevice::new(i2c_bus), delay.clone());

    let sn = sgp41.get_serial_number().await.unwrap();
    log::info!("Serial: {sn:012x}");

    match sgp41.execute_conditioning().await {
        Ok(v) => log::info!("SGP41 Conditioning: {v}"),
        Err(e) => log::error!("SGP41 Conditioning: {e:?}"),
    }

    match sgp41.execute_self_test().await {
        Ok(st) => log::info!("SGP41 Self Test: {st:?}"),
        Err(e) => log::error!("SGP41 Self Test: {e:?}"),
    }

    loop {
        match sgp41.measure_raw_signals(None, None).await {
            Ok(m) => log::info!("SGP41: {m:?}"),
            Err(e) => log::error!("{e:?}"),
        }
        Timer::after(Duration::from_secs(1)).await;
    }
}
