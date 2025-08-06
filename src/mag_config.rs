//! BNO055 Magnetometer Configuration

use num_derive::FromPrimitive;
use num_traits::FromPrimitive;

#[derive(Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum Error {
    InvalidMagDataRate,
    InvalidMagOperationMode,
    InvalidMagPowerMode,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, FromPrimitive)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[repr(u8)]
pub enum MagDataRate {
    Hz2 = 0b000,
    Hz6 = 0b001,
    Hz8 = 0b010,
    Hz10 = 0b011,
    Hz15 = 0b100,
    Hz20 = 0b101,
    Hz25 = 0b110,
    Hz30 = 0b111,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, FromPrimitive)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[repr(u8)]
pub enum MagOperationMode {
    LowPower = 0b00,
    Regular = 0b01,
    EnhancedRegular = 0b10,
    HighAccuracy = 0b11,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, FromPrimitive)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[repr(u8)]
pub enum MagPowerMode {
    Normal = 0b00,
    Sleep = 0b01,
    Suspend = 0b10,
    ForceMode = 0b11,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct MagConfig {
    data_rate: MagDataRate,
    op_mode: MagOperationMode,
    power_mode: MagPowerMode,
}

impl Default for MagConfig {
    fn default() -> Self {
        Self {
            data_rate: MagDataRate::Hz20,
            op_mode: MagOperationMode::Regular,
            power_mode: MagPowerMode::Normal,
        }
    }
}

impl MagConfig {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn set_data_rate(&mut self, rate: MagDataRate) {
        self.data_rate = rate;
    }

    pub fn data_rate(&self) -> MagDataRate {
        self.data_rate
    }

    pub fn set_op_mode(&mut self, mode: MagOperationMode) {
        self.op_mode = mode;
    }

    pub fn op_mode(&self) -> MagOperationMode {
        self.op_mode
    }

    pub fn set_power_mode(&mut self, mode: MagPowerMode) {
        self.power_mode = mode;
    }

    pub fn power_mode(&self) -> MagPowerMode {
        self.power_mode
    }

    pub fn to_bits(&self) -> u8 {
        ((self.power_mode as u8) << 5) | ((self.op_mode as u8) << 3) | (self.data_rate as u8)
    }

    pub fn from_bits(bits: u8) -> Result<Self, Error> {
        let data_rate = MagDataRate::from_u8(bits & 0b111).ok_or(Error::InvalidMagDataRate)?;
        let op_mode = MagOperationMode::from_u8((bits >> 3) & 0b11).ok_or(Error::InvalidMagOperationMode)?;
        let power_mode = MagPowerMode::from_u8((bits >> 5) & 0b11).ok_or(Error::InvalidMagPowerMode)?;
        Ok(Self {
            data_rate,
            op_mode,
            power_mode,
        })
    }
}
