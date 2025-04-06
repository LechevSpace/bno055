use bitflags::bitflags;

use num_derive::FromPrimitive;
use num_traits::FromPrimitive;

use crate::BIT_7_RESERVED_MASK;

#[allow(clippy::unusual_byte_groupings)]
const ACC_G_RANGE_MASK: u8 = 0b000_000_11;
#[allow(clippy::unusual_byte_groupings)]
const ACC_BANDWIDTH_MASK: u8 = 0b000_111_00;
#[allow(clippy::unusual_byte_groupings)]
const ACC_OPERATION_MODE_MASK: u8 = 0b111_000_00;

#[derive(Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[allow(clippy::enum_variant_names)]
pub enum Error {
    BadAccGRange,
    BadAccBandwidth,
    BadAccOperationMode,
    BadAccIntSettings,
    BadAccAmThreshold,
    BadAccHgDuration,
    BadAccHgThreshold,
    BadAccNmThreshold,
    BadAccNmSettings,
}

#[derive(FromPrimitive, Default, Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[repr(u8)]
#[allow(clippy::unusual_byte_groupings)]
pub enum AccGRange {
    G2 = 0b000_000_00,
    /// 4G is the default value in the register on Reset.
    /// 
    /// Table 3-4: Default sensor settings
    /// Sensor Range Bandwidth
    /// Accelerometer 4G 62.5 Hz
    /// Magnetometer NA 10 Hz
    /// Gyroscope 2000 dps 32 Hz
    #[default]
    G4 = 0b000_000_01,
    G8 = 0b000_000_10,
    G16 = 0b000_000_11,
}
#[derive(Default, Debug, Clone, Copy, FromPrimitive)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[repr(u8)]
#[allow(clippy::unusual_byte_groupings)]
pub enum AccBandwidth {
    Hz7_81 = 0b000_000_00,
    Hz15_63 = 0b000_001_00,
    Hz31_25 = 0b000_010_00,
    /// The default value in the register on Reset.
    ///
    /// Table 3-4: Default sensor settings
    /// Sensor Range Bandwidth
    /// Accelerometer 4G 62.5 Hz
    /// Magnetometer NA 10 Hz
    /// Gyroscope 2000 dps 32 Hz
    #[default]
    Hz62_5 = 0b000_011_00,
    Hz125 = 0b000_100_00,
    Hz250 = 0b000_101_00,
    Hz500 = 0b000_110_00,
    Hz1000 = 0b000_111_00,
}

#[derive(Default, Debug, Clone, Copy, FromPrimitive)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[repr(u8)]
#[allow(clippy::unusual_byte_groupings)]
pub enum AccOperationMode {
    #[default]
    /// The default value in the register on Reset.
    Normal = 0b000_000_00,
    Suspend = 0b001_000_00,
    LowPower1 = 0b010_000_00,
    Standby = 0b011_000_00,
    LowPower2 = 0b100_000_00,
    DeepSuspend = 0b101_000_00,
}

#[derive(Default, Debug, Clone)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct AccConfig {
    g_range: AccGRange,
    bandwidth: AccBandwidth,
    operation_mode: AccOperationMode,
}

impl AccConfig {
    pub fn try_from_bits(bits: u8) -> Result<Self, Error> {
        let g_range = AccGRange::from_u8(bits & ACC_G_RANGE_MASK).ok_or(Error::BadAccGRange)?;
        let bandwidth =
            AccBandwidth::from_u8(bits & ACC_BANDWIDTH_MASK).ok_or(Error::BadAccBandwidth)?;
        let operation_mode = AccOperationMode::from_u8(bits & ACC_OPERATION_MODE_MASK)
            .ok_or(Error::BadAccOperationMode)?;

        Ok(Self {
            g_range,
            bandwidth,
            operation_mode,
        })
    }

    pub fn bits(&self) -> u8 {
        self.operation_mode as u8 | self.bandwidth as u8 | self.g_range as u8
    }

    pub fn g_range(&self) -> AccGRange {
        self.g_range
    }

    pub fn bandwidth(&self) -> AccBandwidth {
        self.bandwidth
    }

    pub fn operation_mode(&self) -> AccOperationMode {
        self.operation_mode
    }

    pub fn set_g_range(&mut self, g_range: AccGRange) {
        self.g_range = g_range;
    }

    pub fn set_bandwidth(&mut self, bandwidth: AccBandwidth) {
        self.bandwidth = bandwidth;
    }

    pub fn set_operation_mode(&mut self, operation_mode: AccOperationMode) {
        self.operation_mode = operation_mode;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct BNO055AccIntSettingsFlags(u8);

bitflags! {
    /// BNO055 accelerometer interrupt settings
    impl BNO055AccIntSettingsFlags: u8 {
    // pub struct BNO055AccIntSettingsFlags: u8 {
        const HG_Z_AXIS = 0b10000000;
        const HG_Y_AXIS = 0b01000000;
        const HG_X_AXIS = 0b00100000;
        const AMNM_Z_AXIS = 0b00010000;
        const AMNM_Y_AXIS = 0b00001000;
        const AMNM_X_AXIS = 0b00000100;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
// #[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct BNO055AccIntSettings {
    pub flags: BNO055AccIntSettingsFlags,
    /// `am_dur` in [0, 3]
    pub am_dur: u8,
}

impl num_traits::FromPrimitive for BNO055AccIntSettings {
    fn from_u8(regval: u8) -> Option<Self> {
        Some(Self::from(regval))
    }

    fn from_i64(n: i64) -> Option<Self> {
        use core::convert::TryFrom;
        let regval = u8::try_from(n).ok();

        regval.map(Self::from)
    }

    fn from_u64(n: u64) -> Option<Self> {
        use core::convert::TryFrom;
        let regval = u8::try_from(n).ok();

        regval.map(Self::from)
    }
}

impl From<u8> for BNO055AccIntSettings {
    fn from(regval: u8) -> Self {
        Self {
            flags: BNO055AccIntSettingsFlags::from_bits_truncate(regval),
            am_dur: regval & 3,
        }
    }
}

impl Into<u8> for BNO055AccIntSettings {
    fn into(self) -> u8 {
        let mut dur = self.am_dur;
        if dur > 3 {
            dur = 3;
        }
        self.flags.bits() | dur
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct BNO055AccNmSettings {
    /// `dur` in [0, 0b111111]. Details of how actual duration is calculated in official doc
    pub dur: u8,
    /// true - no motion, false - slow motion
    pub is_no_motion: bool,
}

impl num_traits::FromPrimitive for BNO055AccNmSettings {
    fn from_u8(regval: u8) -> Option<Self> {
        Some(Self::from(regval))
    }

    fn from_i64(n: i64) -> Option<Self> {
        use core::convert::TryFrom;
        let regval = u8::try_from(n).ok();

        regval.map(Self::from)
    }

    fn from_u64(n: u64) -> Option<Self> {
        use core::convert::TryFrom;
        let regval = u8::try_from(n).ok();

        regval.map(Self::from)
    }
}

impl From<u8> for BNO055AccNmSettings {
    fn from(regval: u8) -> Self {
        Self {
            dur: (regval & 0b01111110) >> 1,
            is_no_motion: match regval & 1 {
                0 => false,
                _ => true,
            },
        }
    }
}

impl Into<u8> for BNO055AccNmSettings {
    fn into(self) -> u8 {
        let mut dur = self.dur;
        if dur > 0b111111 {
            dur = 0b111111;
        }
        let is_no_motion = match self.is_no_motion {
            true => 1,
            false => 0,
        };
        BIT_7_RESERVED_MASK & ((dur << 1) | is_no_motion)
    }
}
