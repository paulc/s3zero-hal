#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::rmt::Rmt;
use esp_hal::time::Rate;
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::timer::timg::TimerGroup;
use esp_hal_rmt_onewire::{OneWire, Search};
use log::info;

use s3zero_hal::ds18b20::{check_onewire_crc, Ds18b20};
use s3zero_hal::rgb::Rgb;
use s3zero_hal::ws2812_rmt_single::{Ws2812Async, Ws2812Fixed, RMT_FREQ_MHZ};

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[embassy_executor::task]
async fn led(mut ws2812: Ws2812Fixed) {
    loop {
        for c in [
            Rgb::new(255, 0, 0),
            Rgb::new(0, 255, 0),
            Rgb::new(0, 0, 255),
        ] {
            log::info!("{c:?}");
            ws2812.set(c).expect("Error setting Ws2812");
            Timer::after(Duration::from_secs(1)).await;
        }
    }
}

#[embassy_executor::task]
async fn led_generic(
    chan: esp_hal::rmt::ChannelCreator<esp_hal::Async, 0>,
    gpio: esp_hal::gpio::AnyPin<'static>,
) {
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
    let mut _delay = Delay;

    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    info!("Embassy initialized!");

    let rng = esp_hal::rng::Rng::new(peripherals.RNG);
    let timer1 = TimerGroup::new(peripherals.TIMG0);

    let wifi_init =
        esp_wifi::init(timer1.timer0, rng).expect("Failed to initialize WIFI/BLE controller");
    let (mut _wifi_controller, _interfaces) = esp_wifi::wifi::new(&wifi_init, peripherals.WIFI)
        .expect("Failed to initialize WIFI controller");

    // Initialize RMT peripheral
    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(RMT_FREQ_MHZ))
        .expect("Error initialising RMT")
        .into_async();

    spawner
        .spawn(led_generic(rmt.channel0, peripherals.GPIO21.into()))
        .expect("Error spawning led task");

    let mut ow = OneWire::new(rmt.channel3, rmt.channel4, peripherals.GPIO6).unwrap();
    let mut ds18b20: Option<Ds18b20> = None;

    let mut s = Search::new();
    loop {
        match s.next(&mut ow).await {
            Ok(address) => {
                let a = address.0.to_le_bytes();
                if a[0] == 0x28 && check_onewire_crc(&a) {
                    log::info!("Found DS18B20 {address:?}");
                    ds18b20 = Some(Ds18b20::new(address.0));
                } else {
                    log::info!("Found device {address:?} {}", check_onewire_crc(&a));
                }
            }
            Err(_) => {
                log::info!("End of search");
                break;
            }
        }
    }
    loop {
        if let Some(ref ds) = ds18b20 {
            if let Ok(temp) = ds.read_temp(&mut ow).await {
                log::info!("Temp: {temp}");
            }
            let _ = ds.initiate_conversion(&mut ow).await;
        }
        Timer::after(Duration::from_secs(1)).await;
    }
}
