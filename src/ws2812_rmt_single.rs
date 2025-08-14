use esp_hal::gpio::interconnect::PeripheralOutput;
use esp_hal::gpio::Level;
use esp_hal::rmt::{PulseCode, TxChannel, TxChannelConfig, TxChannelCreator, TxChannelInternal};
use esp_hal::Blocking;

use crate::rgb::{Rgb, RgbLayout};

pub const RMT_FREQ_MHZ: u32 = 80;
pub const RMT_CLK_DIVIDER: u32 = 2;

// WS2812 timings: 1us = RMT_FREQ / RMT_CLK_DIVIDER
const RMT_CHAN_FREQ: u16 = RMT_FREQ_MHZ as u16 / RMT_CLK_DIVIDER as u16;
const T0H: u16 = RMT_CHAN_FREQ * 400 / 1000; // 0.4us
const T0L: u16 = RMT_CHAN_FREQ * 850 / 1000; // 0.85us
const T1H: u16 = RMT_CHAN_FREQ * 800 / 1000; // 0.8us
const T1L: u16 = RMT_CHAN_FREQ * 450 / 1000; // 0.45us
const RESET: u16 = RMT_CHAN_FREQ * 50; // 50us

pub struct Ws2812<C: TxChannelInternal> {
    channel: Option<esp_hal::rmt::Channel<Blocking, C>>,
    t0: u32, // PulseCode
    t1: u32,
    reset: u32,
}

impl<'a, C: TxChannelInternal> Ws2812<C> {
    pub fn new(
        c: impl TxChannelCreator<'a, Blocking, Raw = C>,
        gpio: impl PeripheralOutput<'a>,
    ) -> anyhow::Result<Self> {
        let tx_config = TxChannelConfig::default()
            .with_clk_divider(2)
            .with_idle_output_level(Level::Low)
            .with_carrier_modulation(false);
        let channel = c
            .configure_tx(gpio, tx_config)
            .map_err(|e| anyhow::anyhow!("RMT Channel: {e:?}"))?;

        let t0: u32 = PulseCode::new(Level::High, T0H, Level::Low, T0L);
        let t1: u32 = PulseCode::new(Level::High, T1H, Level::Low, T1L);
        let reset: u32 = PulseCode::new(Level::Low, RESET, Level::High, 0);
        Ok(Self {
            channel: Some(channel),
            t0,
            t1,
            reset,
        })
    }
    pub fn set(&mut self, colour: Rgb) -> anyhow::Result<()> {
        let c = colour.to_u32(RgbLayout::Grb);

        // Generate pulses
        let mut pulses = [0_u32; 25];
        for i in 0..24 {
            // Send MSB first
            let bit = (c >> (23 - i)) & 1;
            pulses[i] = if bit == 0 { self.t0 } else { self.t1 };
        }
        pulses[24] = self.reset;

        // Take ownership of the channel
        let channel = self
            .channel
            .take()
            .ok_or(anyhow::anyhow!("Error taking channel"))?;

        // Transmit the data
        let tx = channel
            .transmit(&pulses)
            .map_err(|e| anyhow::anyhow!("RMT Transmit: {e:?}"))?;

        // Wait for transmission to complete and get the channel back
        self.channel = Some(tx.wait().map_err(|e| anyhow::anyhow!("RMT Wait: {e:?}"))?);
        Ok(())
    }
}
