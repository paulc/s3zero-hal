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
use embedded_hal::delay::DelayNs;
use embedded_hal_async::i2c::I2c;
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
    let a = sgp41.get_serial_number().await.expect("SGP41");
    log::info!("Serial: {a:012x}");

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}

mod sgp41 {

    use embedded_hal_async::delay::DelayNs;
    use embedded_hal_async::i2c::I2c;

    const SGP41_I2C_ADDRESS: u8 = 0x59;
    const SGP4X_GET_SERIAL_NUMBER: [u8; 2] = [0x36, 0x82];

    #[derive(Debug)]
    pub enum SensorError<E> {
        I2c(E),
        CrcError,
        InvalidResponse,
    }

    pub struct Sgp41<I2C, T> {
        i2c: I2C,
        timer: T,
        address: u8,
    }

    impl<I2C, T> Sgp41<I2C, T>
    where
        I2C: I2c,
        T: DelayNs,
    {
        pub fn new(i2c: I2C, timer: T) -> Self {
            Self {
                i2c,
                timer,
                address: SGP41_I2C_ADDRESS,
            }
        }
        pub async fn get_serial_number(&mut self) -> Result<u64, SensorError<I2C::Error>> {
            let mut buf = [0u8; 9];
            self.i2c
                .write_read(self.address, &SGP4X_GET_SERIAL_NUMBER, &mut buf)
                .await
                .map_err(|e| SensorError::I2c(e))?;
            log::info!(">> Serial: {buf:?}");
            let mut out: u64 = 0;
            for c in buf.chunks_exact(3) {
                let [a, b] = self.check_crc(c)?;
                out = (out << 8) + a as u64;
                out = (out << 8) + b as u64;
            }
            Ok(out)
        }
        fn check_crc(&self, data: &[u8]) -> Result<[u8; 2], SensorError<I2C::Error>> {
            if data.len() == 3 && crc(&data[..2]) == data[2] {
                Ok([data[0], data[1]])
            } else {
                Err(SensorError::CrcError)
            }
        }
    }

    fn crc(data: &[u8]) -> u8 {
        const CRC8_POLYNOMIAL: u8 = 0x31;
        let mut crc: u8 = 0xff;
        for byte in data {
            crc ^= byte;
            for _ in 0..8 {
                if (crc & 0x80) > 0 {
                    crc = (crc << 1) ^ CRC8_POLYNOMIAL;
                } else {
                    crc <<= 1;
                }
            }
        }
        crc
    }
}
