#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use bme280_rs::{AsyncBme280, Configuration, Oversampling, SensorMode};
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
use esp_hal::timer::timg::TimerGroup;
use esp_hal::Async;
use esp_hal_rmt_onewire::{OneWire, Search};
use log::info;
use static_cell::StaticCell;

use s3zero_hal::ds18b20::{check_onewire_crc, Ds18b20};
use s3zero_hal::rgb::Rgb;
use s3zero_hal::ws2812_rmt_single::{Ws2812Async, Ws2812Fixed, RMT_FREQ_MHZ};

static I2C_BUS: StaticCell<Mutex<NoopRawMutex, i2c::master::I2c<Async>>> = StaticCell::new();

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

// OW Peripherals
type OwTxRmtChan = ChannelCreator<Async, 3>;
type OwRxRmtChan = ChannelCreator<Async, 4>;
type OwPin = esp_hal::peripherals::GPIO6<'static>;

#[embassy_executor::task]
async fn onewire_task(tx_chan: OwTxRmtChan, rx_chan: OwRxRmtChan, pin: OwPin) {
    log::info!("Starting OneWire task");
    let mut ow = OneWire::new(tx_chan, rx_chan, pin).unwrap();
    let mut ds18b20 = heapless::Vec::<Ds18b20, 5>::new();

    // Scan OneWIre bus for DS18B20
    let mut s = Search::new();
    loop {
        match s.next(&mut ow).await {
            Ok(address) => {
                let a = address.0.to_le_bytes();
                if a[0] == 0x28 && check_onewire_crc(&a) {
                    log::info!("Found DS18B20 {address:?}");
                    ds18b20.push(Ds18b20::new(address.0)).unwrap();
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
    // Read temp
    loop {
        for ds in &ds18b20 {
            if let Ok(temp) = ds.read_temp(&mut ow).await {
                log::info!("DS18B20 [{}]: {temp:.2}°C", ds.address);
            }
            let _ = ds.initiate_conversion(&mut ow).await;
        }
        Timer::after(Duration::from_secs(1)).await;
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
    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(RMT_FREQ_MHZ))
        .expect("Error initialising RMT")
        .into_async();

    spawner
        .spawn(led_generic(rmt.channel0, peripherals.GPIO21.into()))
        .expect("Error spawning led task");

    spawner
        .spawn(onewire_task(rmt.channel3, rmt.channel4, peripherals.GPIO6))
        .expect("Error spawning OneWire task");

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
    let bme280_device = I2cDevice::new(i2c_bus);

    let mut bme280 = AsyncBme280::new(bme280_device, &mut delay);
    bme280.init().await.expect("Error initialising BME280");

    let configuration = Configuration::default()
        .with_temperature_oversampling(Oversampling::Oversample1)
        .with_pressure_oversampling(Oversampling::Oversample1)
        .with_humidity_oversampling(Oversampling::Oversample1)
        .with_sensor_mode(SensorMode::Normal);

    bme280
        .set_sampling_configuration(configuration)
        .await
        .expect("Error serring BME280 configuretion");

    let bme280_id = bme280
        .chip_id()
        .await
        .expect("Error getting BME280 chip_id");
    log::info!("BME280: ID={bme280_id}");

    loop {
        Timer::after(Duration::from_secs(1)).await;
        if let Ok(Some(t)) = bme280.read_temperature().await {
            log::info!("BME280: Temp = {t:.2}°C");
        }
        if let Ok(Some(p)) = bme280.read_pressure().await {
            log::info!("BME280: Pressure = {:.2} hPa", p / 100.0);
        }
    }
}
