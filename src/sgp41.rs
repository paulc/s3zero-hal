use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;

const SGP41_I2C_ADDRESS: u8 = 0x59;
const SGP4X_GET_SERIAL_NUMBER: [u8; 2] = [0x36, 0x82];
const SGP4X_EXECUTE_CONDITIONING: [u8; 2] = [0x26, 0x12];
const SGP4X_EXECUTE_CONDITIONING_DELAY: u32 = 10_000;
const SGP4X_EXECUTE_SELF_TEST: [u8; 2] = [0x28, 0x0E];
const SGP4X_EXECUTE_SELF_TEST_DELAY: u32 = 320;
const SGP4X_MEASURE_RAW_SIGNALS: [u8; 2] = [0x26, 0x19];
const SGP4X_MEASURE_RAW_SIGNALS_DELAY: u32 = 50;
const SGP4X_TURN_HEATER_OFF: [u8; 2] = [0x36, 0x15];

#[derive(Debug)]
pub enum SensorError<E> {
    I2c(E),
    CrcError,
}

#[derive(Debug)]
pub struct Sgp41<I2C, T> {
    i2c: I2C,
    timer: T,
    address: u8,
}

#[derive(Debug)]
pub struct Sgp41Measurement {
    pub sraw_voc: u16,
    pub sraw_nox: u16,
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

#[derive(Debug)]
pub struct CrcPacket([u8; 3]);

impl CrcPacket {
    pub fn to_u16(&self) -> u16 {
        u16::from_be_bytes([self.0[0], self.0[1]])
    }
}

impl TryFrom<&[u8]> for CrcPacket {
    type Error = CrcPacketError;
    fn try_from(data: &[u8]) -> Result<Self, CrcPacketError> {
        if data.len() == 3 && crc(&data[..2]) == data[2] {
            // We know that data.len() == 3 so safe
            Ok(Self(data.try_into().unwrap()))
        } else {
            Err(CrcPacketError)
        }
    }
}

impl From<[u8; 2]> for CrcPacket {
    fn from(data: [u8; 2]) -> Self {
        let pkt = [data[0], data[1], crc(&data)];
        Self(pkt)
    }
}

#[derive(Debug)]
pub struct CrcPacketError;

impl core::fmt::Display for CrcPacketError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "CRC Packet Error")
    }
}

impl core::error::Error for CrcPacketError {}

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
        let mut out: u64 = 0;
        for c in buf.chunks_exact(3) {
            out = (out << 16)
                + CrcPacket::try_from(c)
                    .map_err(|_| SensorError::CrcError)?
                    .to_u16() as u64;
        }
        Ok(out)
    }
    pub async fn execute_self_test(&mut self) -> Result<(bool, bool), SensorError<I2C::Error>> {
        self.i2c
            .write(self.address, &SGP4X_EXECUTE_SELF_TEST)
            .await
            .map_err(|e| SensorError::I2c(e))?;
        self.timer.delay_ms(SGP4X_EXECUTE_SELF_TEST_DELAY).await;
        let mut buf = [0u8; 3];
        self.i2c
            .read(self.address, &mut buf)
            .await
            .map_err(|e| SensorError::I2c(e))?;
        let st = CrcPacket::try_from(&buf[..])
            .map_err(|_| SensorError::CrcError)?
            .to_u16();
        Ok(((st & 0x01) == 0, (st & 0x02) == 0))
    }
    pub async fn execute_conditioning(&mut self) -> Result<u16, SensorError<I2C::Error>> {
        let mut cmd = [0_u8; 8];
        cmd[0..2].copy_from_slice(&SGP4X_EXECUTE_CONDITIONING);
        cmd[2..5].copy_from_slice(&CrcPacket::from([0x80, 0x00]).0);
        cmd[5..8].copy_from_slice(&CrcPacket::from([0x66, 0x66]).0);
        self.i2c
            .write(self.address, &cmd)
            .await
            .map_err(|e| SensorError::I2c(e))?;
        self.timer.delay_ms(SGP4X_EXECUTE_CONDITIONING_DELAY).await;
        let mut buf = [0u8; 3];
        self.i2c
            .read(self.address, &mut buf)
            .await
            .map_err(|e| SensorError::I2c(e))?;
        Ok(CrcPacket::try_from(&buf[..])
            .map_err(|_| SensorError::CrcError)?
            .to_u16())
    }
    pub async fn measure_raw_signals(
        &mut self,
        humidity: Option<f32>,
        temp: Option<f32>,
    ) -> Result<Sgp41Measurement, SensorError<I2C::Error>> {
        let mut cmd = [0_u8; 8];
        let h_comp = match humidity {
            Some(h) => ((h * 65535.0) as u16).to_be_bytes(),
            None => [0x80, 0x00],
        };
        let t_comp = match temp {
            Some(t) => (((t + 45.0) * 65535.0 / 175.0) as u16).to_be_bytes(),
            None => [0x66, 0x66],
        };
        cmd[0..2].copy_from_slice(&SGP4X_MEASURE_RAW_SIGNALS);
        cmd[2..5].copy_from_slice(&CrcPacket::from(h_comp).0);
        cmd[5..8].copy_from_slice(&CrcPacket::from(t_comp).0);
        self.i2c
            .write(self.address, &cmd)
            .await
            .map_err(|e| SensorError::I2c(e))?;
        self.timer.delay_ms(SGP4X_MEASURE_RAW_SIGNALS_DELAY).await;
        let mut buf = [0u8; 6];
        self.i2c
            .read(self.address, &mut buf)
            .await
            .map_err(|e| SensorError::I2c(e))?;
        let (sraw_voc, sraw_nox) = buf.split_at(3);
        let sraw_voc = CrcPacket::try_from(sraw_voc)
            .map_err(|_| SensorError::CrcError)?
            .to_u16();
        let sraw_nox = CrcPacket::try_from(sraw_nox)
            .map_err(|_| SensorError::CrcError)?
            .to_u16();
        Ok(Sgp41Measurement { sraw_voc, sraw_nox })
    }
    pub async fn turn_heater_off(&mut self) -> Result<(), SensorError<I2C::Error>> {
        self.i2c
            .write(self.address, &SGP4X_TURN_HEATER_OFF)
            .await
            .map_err(|e| SensorError::I2c(e))
    }
}
