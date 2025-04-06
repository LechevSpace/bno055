use bitflags::bitflags;

// use num_derive::FromPrimitive;
// use num_traits::FromPrimitive;

use crate::BIT_7_RESERVED_MASK;

#[derive(Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[allow(clippy::enum_variant_names)]
pub enum Error {
    BadGyrHrSettings,
    BadGyrDurX,
    BadGyrHrYSettings,
    BadGyrDurY,
    BadGyrHrZSettings,
    BadGyrDurZ,
    BadGyrAmThreshold,
    BadGyrAmSettings,
}


bitflags! {
    /// BNO055 gyroscope interrupt settings
    #[cfg_attr(not(feature = "defmt-03"), derive(Debug, Clone, Copy, PartialEq, Eq))]
    pub struct BNO055GyrIntSettings: u8 {
        const HR_FILT = 0b10000000;
        const AM_FILT = 0b01000000;
        const HR_Z_AXIS = 0b00100000;
        const HR_Y_AXIS = 0b00010000;
        const HR_X_AXIS = 0b00001000;
        const AM_Z_AXIS = 0b00000100;
        const AM_Y_AXIS = 0b00000010;
        const AM_X_AXIS = 0b00000001;
    }
}

/// Gyroscope High Rate interrupt settings
#[derive(Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]

pub struct BNO055GyrHrSettings {
    /// `hysteresis` in [0, 0b11]. Actual value is `hysteresis` * base-unit based on gyroscope range set in GYR_CONFIG
    pub hysteresis: u8,
    /// `threshold` in [0, 0b11111]. Actual value is `threshold` * base-unit based on gyroscope range set in GYR_CONFIG
    pub threshold: u8,
}

impl num_traits::FromPrimitive for BNO055GyrHrSettings {
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

impl From<u8> for BNO055GyrHrSettings {
    fn from(regval: u8) -> Self {
        Self {
            hysteresis: (regval & 0b01100000) >> 5,
            threshold: regval & 0b00011111,
        }
    }
}

impl Into<u8> for BNO055GyrHrSettings {
    fn into(self) -> u8 {
        let mut hysteresis = self.hysteresis;
        if hysteresis > 0b11 {
            hysteresis = 0b11;
        }
        let mut threshold = self.threshold;
        if threshold > 0b11111 {
            threshold = 0b11111;
        }
        BIT_7_RESERVED_MASK & (hysteresis << 5 | threshold)
    }
}

/// Gyroscope Any Motion interrupt settings
#[derive(Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]

pub struct BNO055GyrAmSettings {
    pub awake_duration: GyrAmSamplesAwake,
    /// `slope_samples` in [0, 0b11]. Actual value is (`slope_samples` + 1) * 4
    pub slope_samples: u8,
}

impl num_traits::FromPrimitive for BNO055GyrAmSettings {
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

impl From<u8> for BNO055GyrAmSettings {
    fn from(regval: u8) -> Self {
        Self {
            awake_duration: ((regval >> 2) & 3).into(),
            slope_samples: regval & 3,
        }
    }
}

impl Into<u8> for BNO055GyrAmSettings {
    fn into(self) -> u8 {
        let mut slope_samples = self.slope_samples;
        if slope_samples > 0b11 {
            slope_samples = 0b11;
        }
        let awake_duration: u8 = self.awake_duration.into();
        0b00001111 & (awake_duration | slope_samples)
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum GyrAmSamplesAwake {
    Samples8,
    Samples16,
    Samples32,
    Samples64,
}

impl From<u8> for GyrAmSamplesAwake {
    fn from(regval: u8) -> Self {
        match regval {
            0 => Self::Samples8,
            1 => Self::Samples16,
            2 => Self::Samples32,
            3 => Self::Samples64,
            _ => Self::Samples8, // TODO: handle error case?
        }
    }
}

impl Into<u8> for GyrAmSamplesAwake {
    fn into(self) -> u8 {
        match self {
            Self::Samples8 => 0,
            Self::Samples16 => 1,
            Self::Samples32 => 2,
            Self::Samples64 => 3,
        }
    }
}

impl num_traits::FromPrimitive for GyrAmSamplesAwake {
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