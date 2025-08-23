#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Timer};
use embedded_hal::delay::DelayNs;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{DriveMode, Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::rmt::Rmt;
use esp_hal::time::Rate;
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::timer::timg::TimerGroup;
use log::info;
use onewire::{ds18b20, DeviceSearch, OneWire, DS18B20};

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
        for c in [
            Rgb::new(255, 0, 0),
            Rgb::new(0, 255, 0),
            Rgb::new(0, 0, 255),
        ] {
            log::info!("{c:?}");
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
    let mut delay = Delay;

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
    /*
    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(RMT_FREQ_MHZ))
        .expect("Error initialising RMT")
        .into_async();

    spawner
        .spawn(led_generic(rmt.channel0, peripherals.GPIO21.into()))
        .expect("Error spawning led task");
    */

    let mut ow_pin = Output::new(
        peripherals.GPIO6,
        Level::High,
        OutputConfig::default().with_drive_mode(DriveMode::OpenDrain),
    )
    .into_flex();
    ow_pin.set_input_enable(true);

    let p5 = Input::new(
        peripherals.GPIO5,
        InputConfig::default().with_pull(Pull::Down),
    );

    log::info!("p5: {:?}", p5.level());
    log::info!("p6: {:?}", ow_pin.level());

    let mut wire = OneWire::new(&mut ow_pin, false);

    match wire.reset(&mut delay) {
        Ok(_) => {
            log::info!("OneWire Reset Ok");
            // search for devices
            let mut search = DeviceSearch::new();
            while let Some(device) = wire.search_next(&mut search, &mut delay).unwrap() {
                match device.address[0] {
                    ds18b20::FAMILY_CODE => {
                        log::info!("DS18B20: {:?}", device.address);
                        let ds18b20 = DS18B20::new(device).unwrap();

                        // request sensor to measure temperature
                        let resolution =
                            ds18b20.measure_temperature(&mut wire, &mut delay).unwrap();

                        // wait for compeltion, depends on resolution
                        delay.delay_ms(resolution.time_ms() as u32);

                        // read temperature
                        let temperature = ds18b20.read_temperature(&mut wire, &mut delay).unwrap();
                        log::info!("DS12B20: Temperature = {temperature}");
                    }
                    _ => {
                        // unknown device type
                        log::info!("Unknown Device: {:?}", device.address);
                    }
                }
            }
        }
        Err(e) => log::error!("OneWire Error: {e:?}"),
    };

    loop {
        log::info!("+++ MAIN +++");
        Timer::after(Duration::from_secs(2)).await;
    }
}
