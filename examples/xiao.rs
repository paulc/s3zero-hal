#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embedded_storage::ReadStorage;
use esp_backtrace as _;
use esp_bootloader_esp_idf::partitions;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{AnyPin, Level, Output, OutputConfig};
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;
use esp_storage::FlashStorage;
use sequential_storage::map;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[embassy_executor::task]
async fn led(led: AnyPin<'static>) {
    let mut led = Output::new(led, Level::High, OutputConfig::default());
    loop {
        led.toggle();
        log::info!("LED: {}", led.is_set_low());
        Timer::after(Duration::from_millis(200)).await;
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.5.0

    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    log::info!("Embassy initialized!");

    let rng = esp_hal::rng::Rng::new(peripherals.RNG);
    let timer1 = TimerGroup::new(peripherals.TIMG0);

    let wifi_init =
        esp_wifi::init(timer1.timer0, rng).expect("Failed to initialize WIFI/BLE controller");
    let (mut _wifi_controller, _interfaces) = esp_wifi::wifi::new(&wifi_init, peripherals.WIFI)
        .expect("Failed to initialize WIFI controller");

    spawner
        .spawn(led(peripherals.GPIO21.into()))
        .expect("Error spawning led task");

    // PSRAM
    let (_, size) = esp_hal::psram::psram_raw_parts(&peripherals.PSRAM);
    log::info!("PSRAM Avail: {size}");

    // Partition Table
    let mut flash = FlashStorage::new();
    println!("Flash size = {}", flash.capacity());

    let mut pt_mem = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
    let pt = partitions::read_partition_table(&mut flash, &mut pt_mem).unwrap();

    for i in 0..pt.len() {
        let raw = pt.get_partition(i).unwrap();
        println!("{:?}", raw);
    }

    // NVS
    let nvs = pt
        .find_partition(partitions::PartitionType::Data(
            partitions::DataPartitionSubType::Nvs,
        ))
        .unwrap()
        .unwrap();
    let nvs_partition = nvs.as_embedded_storage(&mut flash);
    println!("NVS Capacity: {}", nvs_partition.capacity());
    let nvs_partition_range = 0_u32..(nvs_partition.capacity() as u32 - 4096_u32);

    let mut buf = [0_u8; 128];

    let mut nvs = BlockingAsync::new(nvs_partition);

    for key in ["AAAAAAAA", "BBBBBBBB", "CCCCCCCC"] {
        let (k, v) = to_kv::<8, 32>(key, &key.repeat(4));

        map::store_item(
            &mut nvs,
            nvs_partition_range.clone(),
            &mut sequential_storage::cache::NoCache::new(),
            &mut buf,
            &k,
            &v,
        )
        .await
        .unwrap();
    }
    println!("Wrote items");

    for key in ["AAAAAAAA", "BBBBBBBB", "CCCCCCCC", "DDDDDDDD"] {
        let k = to_u8_array::<8>(key);
        let out = map::fetch_item::<[u8; 8], [u8; 32], _>(
            &mut nvs,
            nvs_partition_range.clone(),
            &mut sequential_storage::cache::NoCache::new(),
            &mut buf,
            &k,
        )
        .await
        .unwrap();
        println!("Fetch: {key} = {out:?}");
    }

    // Create the iterator of map items
    let mut cache = sequential_storage::cache::NoCache::new();
    let mut iterator = map::fetch_all_items::<u8, _, _>(
        &mut nvs,
        nvs_partition_range.clone(),
        &mut cache,
        &mut buf,
    )
    .await
    .unwrap();

    // Iterate through all items
    while let Some((key, value)) = iterator.next::<[u8; 8], [u8; 32]>(&mut buf).await.unwrap() {
        // Do somethinmg with the item.
        // Please note that for the same key there might be multiple items returned,
        // the last one is the current active one.
        println!(
            "{} -> {}",
            unsafe { core::str::from_utf8_unchecked(&key) },
            unsafe { core::str::from_utf8_unchecked(&value) }
        );
    }

    loop {
        log::info!("+++ MAIN +++");
        Timer::after(Duration::from_secs(5)).await;
    }
}

fn to_u8_array<const N: usize>(s: &str) -> [u8; N] {
    let mut a = [0_u8; N];
    let len = s.len().min(N);
    a[..len].copy_from_slice(&s.as_bytes()[..len]);
    a
}

fn to_kv<const K: usize, const V: usize>(k: &str, v: &str) -> ([u8; K], [u8; V]) {
    (to_u8_array::<K>(k), to_u8_array::<V>(v))
}
