#![allow(dead_code)]
use esp_hal::gpio::interconnect::PeripheralOutput;
use esp_hal::gpio::Level;
use esp_hal::rmt::{
    Channel, ChannelCreator, ConstChannelAccess, PulseCode, Tx, TxChannel, TxChannelAsync,
    TxChannelConfig, TxChannelCreator, TxChannelInternal,
};
use esp_hal::{Async, Blocking};

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

// Ws2812 on fixed RMT channel (#0)
pub type Ws2812Chan = ChannelCreator<Blocking, 0>;

pub struct Ws2812Fixed {
    channel: Option<Channel<Blocking, ConstChannelAccess<Tx, 0>>>,
    t0: u32, // PulseCode
    t1: u32,
    reset: u32,
}

impl Ws2812Fixed {
    pub fn new<'a>(c: Ws2812Chan, gpio: impl PeripheralOutput<'a>) -> anyhow::Result<Self> {
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

// Ws2812 on generic channel
pub struct Ws2812<C: TxChannelInternal> {
    channel: Option<Channel<Blocking, C>>,
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

// Async Ws2812 on generic channel
pub struct Ws2812Async<C: TxChannelInternal> {
    channel: Channel<Async, C>,
    t0: u32, // PulseCode
    t1: u32,
    reset: u32,
}

impl<'a, C: TxChannelInternal> Ws2812Async<C> {
    pub fn new(
        c: impl TxChannelCreator<'a, Async, Raw = C>,
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
            channel,
            t0,
            t1,
            reset,
        })
    }
    pub async fn set(&mut self, colour: Rgb) -> anyhow::Result<()> {
        let c = colour.to_u32(RgbLayout::Grb);

        // Generate pulses
        let mut pulses = [0_u32; 25];
        for i in 0..24 {
            // Send MSB first
            let bit = (c >> (23 - i)) & 1;
            pulses[i] = if bit == 0 { self.t0 } else { self.t1 };
        }
        pulses[24] = self.reset;

        self.channel
            .transmit(&pulses)
            .await
            .map_err(|e| anyhow::anyhow!("RMT Transmit: {e:?}"))
    }
}
