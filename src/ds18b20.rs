use esp_hal_rmt_onewire::{OneWire, OneWireConfig};

pub struct Ds18b20 {
    address: u64,
}

impl Ds18b20 {
    pub fn new(address: u64) -> Self {
        Self { address }
    }
    pub async fn initiate_conversion<'a, OW: OneWireConfig>(
        &self,
        ow: &mut OneWire<'a, OW>,
    ) -> anyhow::Result<()> {
        if !ow
            .reset()
            .await
            .map_err(|e| anyhow::anyhow!("OW Reset: {e:?}"))?
        {
            anyhow::bail!("OW Reset: Bus Error");
        }
        ow.send_byte(0x55)
            .await
            .map_err(|e| anyhow::anyhow!("OW Send Byte: {e:?}"))?;
        ow.send_u64(self.address)
            .await
            .map_err(|e| anyhow::anyhow!("OW Send u64: {e:?}"))?;
        ow.send_byte(0x44)
            .await
            .map_err(|e| anyhow::anyhow!("OW Send Byte: {e:?}"))?;
        Ok(())
    }
    pub async fn read_temp<'a, OW: OneWireConfig>(
        &self,
        ow: &mut OneWire<'a, OW>,
    ) -> anyhow::Result<f32> {
        if !ow
            .reset()
            .await
            .map_err(|e| anyhow::anyhow!("OW Reset: {e:?}"))?
        {
            anyhow::bail!("OW Reset: Bus Error");
        }
        ow.send_byte(0x55)
            .await
            .map_err(|e| anyhow::anyhow!("OW Send Byte: {e:?}"))?;
        ow.send_u64(self.address)
            .await
            .map_err(|e| anyhow::anyhow!("OW Send u64: {e:?}"))?;
        ow.send_byte(0xBE)
            .await
            .map_err(|e| anyhow::anyhow!("OW Send Byte: {e:?}"))?;
        let temp_low = ow
            .exchange_byte(0xFF)
            .await
            .map_err(|e| anyhow::anyhow!("OW Exchange Byte: {e:?}"))?;
        let temp_high = ow
            .exchange_byte(0xFF)
            .await
            .map_err(|e| anyhow::anyhow!("OW Exchange Byte: {e:?}"))?;
        Ok(self.convert_temp(temp_high, temp_low))
    }
    fn convert_temp(&self, h: u8, l: u8) -> f32 {
        // https://www.analog.com/media/en/technical-documentation/data-sheets/DS18B20.pdf
        let sign = (h >> 3) > 0;
        let i = (((h & 7) << 4) + (l >> 4)) as f32;
        let f = (l & 15) as f32 / 16_f32;
        if sign {
            -(i + f)
        } else {
            i + f
        }
    }
}

pub fn check_onewire_crc(data: &[u8]) -> bool {
    onewire_crc(&data[..7]) == data[7]
}

pub fn onewire_crc(data: &[u8]) -> u8 {
    let mut crc: u8 = 0;
    for &byte in data {
        crc ^= byte;
        // Process each bit of the current byte
        for _ in 0..8 {
            if (crc & 0x01) != 0 {
                crc = (crc >> 1) ^ 0x8C; // 0x8C = 0b10001100
            } else {
                crc >>= 1;
            }
        }
    }
    crc
}
