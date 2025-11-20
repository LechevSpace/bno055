#![doc(html_root_url = "https://docs.rs/bno055/0.5.0")] // Version bump
#![cfg_attr(not(feature = "std"), no_std)]
#![allow(clippy::bad_bit_mask)]

//! Bosch Sensortec BNO055 9-axis IMU sensor driver.
//! Datasheet: https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BNO055-DS000.pdf

// Use maybe_async to toggle between blocking and async implementations
use maybe_async::maybe_async;

// Cfg-gate the HAL traits to select the correct ones based on the "async" feature
#[cfg(not(feature = "async"))]
use embedded_hal::{
    delay::DelayNs,
    i2c::{I2c, SevenBitAddress},
};

#[cfg(feature = "async")]
use embedded_hal_async::{
    delay::DelayNs,
    i2c::{I2c, SevenBitAddress},
};

use bitflags::bitflags;
use byteorder::{ByteOrder, LittleEndian};
pub use mint;
use num_traits::FromPrimitive;
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

pub mod acc_config;
pub mod gyr_config;
pub mod mag_config;
mod regs;
#[cfg(feature = "std")]
pub mod std;

#[doc(inline)]
pub use acc_config::{
    AccBandwidth, AccConfig, AccGRange, AccOperationMode, BNO055AccIntSettings, BNO055AccNmSettings,
};
#[doc(inline)]
pub use gyr_config::{
    BNO055GyrAmSettings, BNO055GyrHrSettings, BNO055GyrIntSettings, GyrAmSamplesAwake,
    GyrBandwidth, GyrConfig, GyrPowerMode, GyrRange,
};
#[doc(inline)]
pub use mag_config::{MagConfig, MagDataRate, MagOperationMode, MagPowerMode};
#[doc(inline)]
pub use regs::BNO055_ID;

pub(crate) const BIT_7_RESERVED_MASK: u8 = 0b01111111;

/// 1 m/s^2 = 100 lsb
pub const ACCEL_SCALING: f32 = 1f32 / 100f32;
/// 1 deg/s = 16 lsb
pub const GYRO_SCALING: f32 = 1f32 / 16f32;
/// 1 uT = 16 lsb
pub const MAG_SCALING: f32 = 1f32 / 16f32;

/// All possible errors in this crate
#[derive(Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum Error<E> {
    /// I2C bus error
    I2c(E),

    /// Invalid chip ID was read
    InvalidChipId(u8),

    /// Invalid (not applicable) device mode.
    InvalidMode,

    /// Accelerometer configuration error
    AccConfig(acc_config::Error),

    /// Gyroscope configuration error
    GyroConfig(gyr_config::Error),

    /// Magnetometer configuration error
    MagConfig(mag_config::Error),
}

#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct Bno055<I> {
    i2c: I,
    pub mode: BNO055OperationMode,
    use_default_addr: bool,
}

#[maybe_async]
impl<I, E> Bno055<I>
where
    I: I2c<SevenBitAddress, Error = E>,
{
    /// Side-effect-free constructor.
    /// Nothing will be read or written before `init()` call.
    pub fn new(i2c: I) -> Self {
        Bno055 {
            i2c,
            mode: BNO055OperationMode::CONFIG_MODE,
            use_default_addr: true,
        }
    }

    /// Destroy driver instance, return I2C bus instance.
    pub fn destroy(self) -> I {
        self.i2c
    }

    /// Enables use of alternative I2C address `regs::BNO055_ALTERNATE_ADDR`.
    pub fn with_alternative_address(mut self) -> Self {
        self.use_default_addr = false;
        self
    }

    /// Initializes the BNO055 device.
    ///
    /// Side-effects:
    /// - Software reset of BNO055
    /// - Sets BNO055 to `CONFIG` mode
    /// - Sets BNO055's power mode to `NORMAL`
    /// - Clears `SYS_TRIGGER` register
    pub async fn init<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;

        let id = self.id().await?;
        if id != regs::BNO055_ID {
            return Err(Error::InvalidChipId(id));
        }

        self.soft_reset(delay).await?;
        self.set_mode(BNO055OperationMode::CONFIG_MODE, delay)
            .await?;
        self.set_power_mode(BNO055PowerMode::NORMAL).await?;
        self.write_u8(regs::BNO055_SYS_TRIGGER, 0x00).await?;

        Ok(())
    }

    /// Resets the BNO055, initializing the register map to default values.
    /// More in section 3.2.
    pub async fn soft_reset<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;
        self.write_u8(
            regs::BNO055_SYS_TRIGGER,
            BNO055SystemTrigger::RST_SYS.bits(),
        )
        .await?;

        // As per table 1.2
        delay.delay_ms(650).await;
        Ok(())
    }

    /// Run Self-test on the BNO055
    ///
    /// See section 3.9.2 Built-In Self-Test (BIST)
    pub async fn self_test<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;
        let prev = self.mode;
        self.set_mode(BNO055OperationMode::CONFIG_MODE, delay)
            .await?;
        self.write_u8(
            regs::BNO055_SYS_TRIGGER,
            BNO055SystemTrigger::SELF_TEST.bits(),
        )
        .await?;
        self.set_mode(prev, delay).await?;
        Ok(())
    }

    /// Sets the operating mode, see [BNO055OperationMode].
    /// See section 3.3.
    pub async fn set_mode<D: DelayNs>(
        &mut self,
        mode: BNO055OperationMode,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        if self.mode != mode {
            self.set_page(BNO055RegisterPage::PAGE_0).await?;
            self.mode = mode;
            self.write_u8(regs::BNO055_OPR_MODE, mode.bits()).await?;
            // Table 3-6 says 19ms to switch to CONFIG_MODE
            delay.delay_ms(19).await;
        }
        Ok(())
    }

    /// Sets the power mode, see [BNO055PowerMode](enum.BNO055PowerMode.html)
    /// See section 3.2
    pub async fn set_power_mode(&mut self, mode: BNO055PowerMode) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;
        self.write_u8(regs::BNO055_PWR_MODE, mode.bits()).await?;
        Ok(())
    }

    /// Returns BNO055's power mode.
    pub async fn power_mode(&mut self) -> Result<BNO055PowerMode, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;
        let mode = self.read_u8(regs::BNO055_PWR_MODE).await?;
        Ok(BNO055PowerMode::from_bits_truncate(mode))
    }

    /// Enables/Disables usage of external 32k crystal.
    pub async fn set_external_crystal<D: DelayNs>(
        &mut self,
        ext: bool,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;
        let prev = self.mode;
        self.set_mode(BNO055OperationMode::CONFIG_MODE, delay)
            .await?;
        let value = if ext {
            BNO055SystemTrigger::EXT_CLK_SEL.bits()
        } else {
            0x00
        };
        self.write_u8(regs::BNO055_SYS_TRIGGER, value).await?;
        self.set_mode(prev, delay).await?;
        Ok(())
    }

    /// Configures axis remap of the device.
    pub async fn set_axis_remap(&mut self, remap: AxisRemap) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;
        let remap_value = (remap.x.bits() & 0b11)
            | ((remap.y.bits() & 0b11) << 2)
            | ((remap.z.bits() & 0b11) << 4);
        self.write_u8(regs::BNO055_AXIS_MAP_CONFIG, remap_value)
            .await?;
        Ok(())
    }

    /// Returns axis remap of the device.
    pub async fn axis_remap(&mut self) -> Result<AxisRemap, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;
        let value = self.read_u8(regs::BNO055_AXIS_MAP_CONFIG).await?;
        let remap = AxisRemap {
            x: BNO055AxisConfig::from_bits_truncate(value & 0b11),
            y: BNO055AxisConfig::from_bits_truncate((value >> 2) & 0b11),
            z: BNO055AxisConfig::from_bits_truncate((value >> 4) & 0b11),
        };
        Ok(remap)
    }

    /// Configures device's axes sign: positive or negative.
    pub async fn set_axis_sign(&mut self, sign: BNO055AxisSign) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;
        self.write_u8(regs::BNO055_AXIS_MAP_SIGN, sign.bits())
            .await?;
        Ok(())
    }

    /// Return device's axes sign.
    pub async fn axis_sign(&mut self) -> Result<BNO055AxisSign, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;
        let value = self.read_u8(regs::BNO055_AXIS_MAP_SIGN).await?;
        Ok(BNO055AxisSign::from_bits_truncate(value))
    }

    /// Gets the revision of software, bootloader, accelerometer, magnetometer, and gyroscope.
    pub async fn get_revision(&mut self) -> Result<BNO055Revision, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;
        let mut buf: [u8; 6] = [0; 6];
        self.read_bytes(regs::BNO055_ACC_ID, &mut buf).await?;
        Ok(BNO055Revision {
            software: LittleEndian::read_u16(&buf[3..5]),
            bootloader: buf[5],
            accelerometer: buf[0],
            magnetometer: buf[1],
            gyroscope: buf[2],
        })
    }

    /// Returns device's system status.
    pub async fn get_system_status<D: DelayNs>(
        &mut self,
        do_selftest: bool,
        delay: &mut D,
    ) -> Result<BNO055SystemStatus, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;

        let selftest = if do_selftest {
            let prev = self.mode;
            self.set_mode(BNO055OperationMode::CONFIG_MODE, delay)
                .await?;
            let sys_trigger = self.read_u8(regs::BNO055_SYS_TRIGGER).await?;
            self.write_u8(regs::BNO055_SYS_TRIGGER, sys_trigger | 0x1)
                .await?;
            delay.delay_ms(1000).await; // Wait for self-test result
            let result = self.read_u8(regs::BNO055_ST_RESULT).await?;
            self.set_mode(prev, delay).await?; // Restore previous mode
            Some(BNO055SelfTestStatus::from_bits_truncate(result))
        } else {
            None
        };

        let status = self.read_u8(regs::BNO055_SYS_STATUS).await?;
        let error = self.read_u8(regs::BNO055_SYS_ERR).await?;

        Ok(BNO055SystemStatus {
            status: BNO055SystemStatusCode::from_bits_truncate(status),
            error: BNO055SystemErrorCode::from_bits_truncate(error),
            selftest,
        })
    }

    /// Gets a quaternion (`mint::Quaternion<f32>`) reading from the BNO055.
    pub async fn quaternion(&mut self) -> Result<mint::Quaternion<f32>, Error<E>> {
        if !self.mode.is_fusion_enabled() {
            return Err(Error::InvalidMode);
        }
        self.set_page(BNO055RegisterPage::PAGE_0).await?;
        let mut buf: [u8; 8] = [0; 8];
        self.read_bytes(regs::BNO055_QUA_DATA_W_LSB, &mut buf)
            .await?;
        let w = LittleEndian::read_i16(&buf[0..2]);
        let x = LittleEndian::read_i16(&buf[2..4]);
        let y = LittleEndian::read_i16(&buf[4..6]);
        let z = LittleEndian::read_i16(&buf[6..8]);
        let scale = 1.0 / ((1 << 14) as f32);
        Ok(mint::Quaternion {
            v: mint::Vector3 {
                x: x as f32 * scale,
                y: y as f32 * scale,
                z: z as f32 * scale,
            },
            s: w as f32 * scale,
        })
    }

    /// Get Euler angles representation of heading in degrees.
    pub async fn euler_angles(&mut self) -> Result<mint::EulerAngles<f32, ()>, Error<E>> {
        if !self.mode.is_fusion_enabled() {
            return Err(Error::InvalidMode);
        }
        self.set_page(BNO055RegisterPage::PAGE_0).await?;
        let mut buf: [u8; 6] = [0; 6];
        self.read_bytes(regs::BNO055_EUL_HEADING_LSB, &mut buf)
            .await?;
        let heading = LittleEndian::read_i16(&buf[0..2]) as f32;
        let roll = LittleEndian::read_i16(&buf[2..4]) as f32;
        let pitch = LittleEndian::read_i16(&buf[4..6]) as f32;
        let scale = 1f32 / 16f32;
        Ok(mint::EulerAngles::from([
            roll * scale,
            pitch * scale,
            heading * scale,
        ]))
    }

    /// Get calibration status
    pub async fn get_calibration_status(&mut self) -> Result<BNO055CalibrationStatus, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;
        let status = self.read_u8(regs::BNO055_CALIB_STAT).await?;
        Ok(BNO055CalibrationStatus {
            sys: (status >> 6) & 0b11,
            gyr: (status >> 4) & 0b11,
            acc: (status >> 2) & 0b11,
            mag: status & 0b11,
        })
    }

    /// Checks whether device is fully calibrated or not.
    pub async fn is_fully_calibrated(&mut self) -> Result<bool, Error<E>> {
        let status = self.get_calibration_status().await?;
        Ok(status.mag == 3 && status.gyr == 3 && status.acc == 3 && status.sys == 3)
    }

    /// Reads current calibration profile of the device.
    pub async fn calibration_profile<D: DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<BNO055Calibration, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;
        let prev_mode = self.mode;
        self.set_mode(BNO055OperationMode::CONFIG_MODE, delay)
            .await?;
        let mut buf: [u8; BNO055_CALIB_SIZE] = [0; BNO055_CALIB_SIZE];
        self.read_bytes(regs::BNO055_ACC_OFFSET_X_LSB, &mut buf[..])
            .await?;
        let res = BNO055Calibration::from_buf(&buf);
        self.set_mode(prev_mode, delay).await?;
        Ok(res)
    }

    /// Sets current calibration profile.
    pub async fn set_calibration_profile<D: DelayNs>(
        &mut self,
        calib: BNO055Calibration,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;
        let prev_mode = self.mode;
        self.set_mode(BNO055OperationMode::CONFIG_MODE, delay)
            .await?;
        let buf_profile = calib.as_bytes();
        self.write_bytes(regs::BNO055_ACC_OFFSET_X_LSB, buf_profile)
            .await?;
        self.set_mode(prev_mode, delay).await?;
        Ok(())
    }

    /// Returns device's factory-programmed and constant chip ID.
    pub async fn id(&mut self) -> Result<u8, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;
        self.read_u8(regs::BNO055_CHIP_ID).await
    }

    /// Returns device's operation mode.
    pub async fn get_mode(&mut self) -> Result<BNO055OperationMode, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;
        let mode = self.read_u8(regs::BNO055_OPR_MODE).await?;
        let mode = BNO055OperationMode::from_bits_truncate(mode);
        self.mode = mode;
        Ok(mode)
    }

    /// Checks whether the device is in Sensor Fusion mode or not by reading from the device.
    pub async fn is_in_fusion_mode(&mut self) -> Result<bool, Error<E>> {
        let mode = self.get_mode().await?;
        Ok(mode.is_fusion_enabled())
    }

    /// Returns the current accelerometer config
    pub async fn get_acc_config(&mut self) -> Result<AccConfig, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_1).await?;
        let bits = self.read_u8(regs::BNO055_ACC_CONFIG).await?;
        AccConfig::try_from_bits(bits).map_err(Error::AccConfig)
    }

    /// Sets the accelerometer config
    pub async fn set_acc_config<D: DelayNs>(
        &mut self,
        acc_config: &AccConfig,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        let prev_mode = self.mode;
        self.set_mode(BNO055OperationMode::CONFIG_MODE, delay)
            .await?;
        self.set_page(BNO055RegisterPage::PAGE_1).await?;
        self.write_u8(regs::BNO055_ACC_CONFIG, acc_config.bits())
            .await?;
        self.set_mode(prev_mode, delay).await?;

        Ok(())
    }

    /// Returns the current gyroscope config
    pub async fn get_gyr_config(&mut self) -> Result<GyrConfig, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_1).await?;
        let bits0 = self.read_u8(regs::BNO055_GYR_CONFIG_0).await?;
        let bits1 = self.read_u8(regs::BNO055_GYR_CONFIG_1).await?;
        GyrConfig::from_bits(bits0, bits1).map_err(Error::GyroConfig)
    }

    /// Sets the gyroscope config
    pub async fn set_gyr_config<D: DelayNs>(
        &mut self,
        gyr_config: &GyrConfig,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        let prev_mode = self.mode;
        self.set_mode(BNO055OperationMode::CONFIG_MODE, delay)
            .await?;
        self.set_page(BNO055RegisterPage::PAGE_1).await?;
        let (bits0, bits1) = gyr_config.to_bits();
        self.write_u8(regs::BNO055_GYR_CONFIG_0, bits0).await?;
        self.write_u8(regs::BNO055_GYR_CONFIG_1, bits1).await?;
        self.set_mode(prev_mode, delay).await?;

        Ok(())
    }

    /// Returns the current magnetometer config
    pub async fn get_mag_config(&mut self) -> Result<MagConfig, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_1).await?;
        let bits = self.read_u8(regs::BNO055_MAG_CONFIG).await?;
        MagConfig::from_bits(bits).map_err(Error::MagConfig)
    }

    /// Sets the magnetometer config
    pub async fn set_mag_config<D: DelayNs>(
        &mut self,
        mag_config: &MagConfig,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        let prev_mode = self.mode;
        self.set_mode(BNO055OperationMode::CONFIG_MODE, delay)
            .await?;
        self.set_page(BNO055RegisterPage::PAGE_1).await?;
        self.write_u8(regs::BNO055_MAG_CONFIG, mag_config.to_bits())
            .await?;
        self.set_mode(prev_mode, delay).await?;

        Ok(())
    }

    /// Returns linear acceleration vector in cm/s^2 units.
    pub async fn linear_acceleration_fixed(&mut self) -> Result<mint::Vector3<i16>, Error<E>> {
        if !self.mode.is_fusion_enabled() {
            return Err(Error::InvalidMode);
        }
        self.set_page(BNO055RegisterPage::PAGE_0).await?;
        self.read_vec_raw(regs::BNO055_LIA_DATA_X_LSB).await
    }

    /// Returns linear acceleration vector in m/s^2 units.
    pub async fn linear_acceleration(&mut self) -> Result<mint::Vector3<f32>, Error<E>> {
        let lia = self.linear_acceleration_fixed().await?;
        Ok(Self::scale_vec(lia, ACCEL_SCALING))
    }

    /// Returns gravity vector in cm/s^2 units.
    pub async fn gravity_fixed(&mut self) -> Result<mint::Vector3<i16>, Error<E>> {
        if !self.mode.is_fusion_enabled() {
            return Err(Error::InvalidMode);
        }
        self.set_page(BNO055RegisterPage::PAGE_0).await?;
        self.read_vec_raw(regs::BNO055_GRV_DATA_X_LSB).await
    }

    /// Returns gravity vector in m/s^2 units.
    pub async fn gravity(&mut self) -> Result<mint::Vector3<f32>, Error<E>> {
        let grv = self.gravity_fixed().await?;
        Ok(Self::scale_vec(grv, ACCEL_SCALING))
    }

    /// Returns Acceleration and Gyroscope vectors in this order.
    pub async fn dof6_fixed(
        &mut self,
    ) -> Result<(mint::Vector3<i16>, mint::Vector3<i16>), Error<E>> {
        if !self.mode.is_accel_enabled() || !self.mode.is_gyro_enabled() {
            return Err(Error::InvalidMode);
        }
        self.set_page(BNO055RegisterPage::PAGE_0).await?;
        let mut buf: [u8; 12] = [0; 12];
        self.read_bytes(regs::BNO055_ACC_DATA_X_LSB, &mut buf)
            .await?;
        let accel = mint::Vector3::from([
            LittleEndian::read_i16(&buf[0..2]),
            LittleEndian::read_i16(&buf[2..4]),
            LittleEndian::read_i16(&buf[4..6]),
        ]);
        let gyro = mint::Vector3::from([
            LittleEndian::read_i16(&buf[6..8]),
            LittleEndian::read_i16(&buf[8..10]),
            LittleEndian::read_i16(&buf[10..12]),
        ]);
        Ok((accel, gyro))
    }

    /// Returns Acceleration and Gyroscope vectors in this order.
    pub async fn dof6(&mut self) -> Result<(mint::Vector3<f32>, mint::Vector3<f32>), Error<E>> {
        let (accel, gyro) = self.dof6_fixed().await?;
        Ok((
            Self::scale_vec(accel, ACCEL_SCALING),
            Self::scale_vec(gyro, GYRO_SCALING),
        ))
    }

    /// Returns Acceleration, Gyroscope and Magnetometer vectors in this order.
    #[allow(clippy::type_complexity)]
    pub async fn dof9_fixed(
        &mut self,
    ) -> Result<(mint::Vector3<i16>, mint::Vector3<i16>, mint::Vector3<i16>), Error<E>> {
        if !self.mode.is_accel_enabled()
            || !self.mode.is_gyro_enabled()
            || !self.mode.is_mag_enabled()
        {
            return Err(Error::InvalidMode);
        }
        self.set_page(BNO055RegisterPage::PAGE_0).await?;
        let mut buf: [u8; 18] = [0; 18];
        self.read_bytes(regs::BNO055_ACC_DATA_X_LSB, &mut buf)
            .await?;
        let accel = mint::Vector3::from([
            LittleEndian::read_i16(&buf[0..2]),
            LittleEndian::read_i16(&buf[2..4]),
            LittleEndian::read_i16(&buf[4..6]),
        ]);
        let mag = mint::Vector3::from([
            LittleEndian::read_i16(&buf[6..8]),
            LittleEndian::read_i16(&buf[8..10]),
            LittleEndian::read_i16(&buf[10..12]),
        ]);
        let gyro = mint::Vector3::from([
            LittleEndian::read_i16(&buf[12..14]),
            LittleEndian::read_i16(&buf[14..16]),
            LittleEndian::read_i16(&buf[16..18]),
        ]);
        Ok((accel, mag, gyro))
    }

    /// Returns Acceleration, Gyroscope and Magnetometer vectors in this order.
    #[allow(clippy::type_complexity)]
    pub async fn dof9(
        &mut self,
    ) -> Result<(mint::Vector3<f32>, mint::Vector3<f32>, mint::Vector3<f32>), Error<E>> {
        let (accel, mag, gyro) = self.dof9_fixed().await?;
        Ok((
            Self::scale_vec(accel, ACCEL_SCALING),
            Self::scale_vec(mag, MAG_SCALING),
            Self::scale_vec(gyro, GYRO_SCALING),
        ))
    }

    /// Returns current accelerometer data in cm/s^2 units.
    pub async fn accel_data_fixed(&mut self) -> Result<mint::Vector3<i16>, Error<E>> {
        if !self.mode.is_accel_enabled() {
            return Err(Error::InvalidMode);
        }
        self.set_page(BNO055RegisterPage::PAGE_0).await?;
        self.read_vec_raw(regs::BNO055_ACC_DATA_X_LSB).await
    }

    /// Returns current accelerometer data in m/s^2 units.
    pub async fn accel_data(&mut self) -> Result<mint::Vector3<f32>, Error<E>> {
        let a = self.accel_data_fixed().await?;
        Ok(Self::scale_vec(a, ACCEL_SCALING))
    }

    /// Returns current gyroscope data in 1/16th deg/s units.
    pub async fn gyro_data_fixed(&mut self) -> Result<mint::Vector3<i16>, Error<E>> {
        if !self.mode.is_gyro_enabled() {
            return Err(Error::InvalidMode);
        }
        self.set_page(BNO055RegisterPage::PAGE_0).await?;
        self.read_vec_raw(regs::BNO055_GYR_DATA_X_LSB).await
    }

    /// Returns current gyroscope data in deg/s units.
    pub async fn gyro_data(&mut self) -> Result<mint::Vector3<f32>, Error<E>> {
        let g = self.gyro_data_fixed().await?;
        Ok(Self::scale_vec(g, GYRO_SCALING))
    }

    /// Returns current magnetometer data in 1/16th uT units.
    pub async fn mag_data_fixed(&mut self) -> Result<mint::Vector3<i16>, Error<E>> {
        if !self.mode.is_mag_enabled() {
            return Err(Error::InvalidMode);
        }
        self.set_page(BNO055RegisterPage::PAGE_0).await?;
        self.read_vec_raw(regs::BNO055_MAG_DATA_X_LSB).await
    }

    /// Returns current magnetometer data in uT units.
    pub async fn mag_data(&mut self) -> Result<mint::Vector3<f32>, Error<E>> {
        let m = self.mag_data_fixed().await?;
        Ok(Self::scale_vec(m, MAG_SCALING))
    }

    /// Returns current temperature of the chip (in degrees Celsius).
    pub async fn temperature(&mut self) -> Result<i8, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;
        let temp = self.read_u8(regs::BNO055_TEMP).await? as i8;
        Ok(temp)
    }

    /// Read which interrupts are currently triggered/active
    pub async fn interrupts_triggered(&mut self) -> Result<BNO055Interrupt, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;
        let bits = self.read_u8(regs::BNO055_INT_STA).await?;
        Ok(BNO055Interrupt::from_u8(bits).unwrap_or_default())
        // self.read_flags(BNO055RegisterPage::PAGE_0, regs::BNO055_INT_STA)
        //     .await
    }

    /// Resets the interrupts register and the INT pin.
    ///
    pub async fn clear_interrupts(&mut self) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;
        // We need to fetch the SYS_TRIG first as the external clock bit might be set.
        let sys_trig = self.read_u8(regs::BNO055_SYS_TRIGGER).await?;
        self.write_u8(
            regs::BNO055_SYS_TRIGGER,
            sys_trig | BNO055SystemTrigger::RST_INT.bits(),
        )
        .await
        // self.write_flags(
        //     BNO055RegisterPage::PAGE_0,
        //     regs::BNO055_SYS_TRIGGER,
        //     BNO055SystemTrigger::RST_INT,
        // )
        // .await
    }

    /// Sets which interrupts are enabled, overrides all current interrupts
    pub async fn set_interrupts_enabled<D: DelayNs>(
        &mut self,
        interrupts: BNO055Interrupt,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        let prev_mode = self.mode;
        self.set_mode(BNO055OperationMode::CONFIG_MODE, delay)
            .await?;

        self.set_page(BNO055RegisterPage::PAGE_1).await?;
        self.write_u8(regs::BNO055_INT_EN, interrupts.bits())
            .await?;
        // self.write_flags(BNO055RegisterPage::PAGE_1, regs::BNO055_INT_EN, interrupts)
        //     .await?;

        self.set_mode(prev_mode, delay).await
    }

    /// Returns currently enabled interrupts
    pub async fn interrupts_enabled(&mut self) -> Result<BNO055Interrupt, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_1).await?;
        let bits = self.read_u8(regs::BNO055_INT_EN).await?;
        Ok(BNO055Interrupt::from_u8(bits).unwrap_or_default())
        // self.read_flags(BNO055RegisterPage::PAGE_1, regs::BNO055_INT_EN)
        //     .await
    }

    /// Sets interrupts mask
    pub async fn set_interrupts_mask<D: DelayNs>(
        &mut self,
        mask: BNO055Interrupt,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        let prev_mode = self.mode;
        self.set_mode(BNO055OperationMode::CONFIG_MODE, delay)
            .await?;

        self.set_page(BNO055RegisterPage::PAGE_1).await?;
        self.write_u8(regs::BNO055_INT_MSK, mask.bits()).await?;
        // self.write_flags(BNO055RegisterPage::PAGE_1, regs::BNO055_INT_MSK, mask)
        //     .await

        self.set_mode(prev_mode, delay).await
    }

    /// Returns the current interrupts mask
    pub async fn interrupts_mask(&mut self) -> Result<BNO055Interrupt, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_1).await?;
        let bits = self.read_u8(regs::BNO055_INT_MSK).await?;
        Ok(BNO055Interrupt::from_u8(bits).unwrap_or_default())
        // self.read_flags(BNO055RegisterPage::PAGE_1, regs::BNO055_INT_MSK)
        //     .await
    }

    /// Sets accelerometer interrupt settings
    pub async fn set_acc_interrupt_settings<D: DelayNs>(
        &mut self,
        settings: BNO055AccIntSettings,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        self.set_config_from(
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_ACC_INT_SETTING,
            settings,
            delay,
        )
        .await
    }

    /// Returns current accelerometer interrupt settings
    pub async fn acc_interrupt_settings(&mut self) -> Result<BNO055AccIntSettings, Error<E>> {
        self.read_u8_into(BNO055RegisterPage::PAGE_1, regs::BNO055_ACC_INT_SETTING)
            .await?
            .ok_or(Error::AccConfig(acc_config::Error::BadAccIntSettings))
    }

    /// Sets accelerometer any motion interrupt threshold setting
    pub async fn set_acc_am_threshold<D: DelayNs>(
        &mut self,
        mult: u8,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        self.set_config_from(
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_ACC_AM_THRES,
            mult,
            delay,
        )
        .await
    }

    /// Returns current accelerometer any motion interrupt threshold setting
    pub async fn acc_am_threshold(&mut self) -> Result<u8, Error<E>> {
        self.read_u8_into::<u8>(BNO055RegisterPage::PAGE_1, regs::BNO055_ACC_AM_THRES)
            .await?
            .ok_or(Error::AccConfig(acc_config::Error::BadAccAmThreshold))
    }

    /// Sets accelerometer High-G interrupt duration setting
    pub async fn set_acc_hg_duration<D: DelayNs>(
        &mut self,
        dur: u8,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        self.set_config_from(
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_ACC_HG_DURATION,
            dur,
            delay,
        )
        .await
    }

    /// Returns current accelerometer High-G interrupt duration setting
    pub async fn acc_hg_duration(&mut self) -> Result<u8, Error<E>> {
        self.read_u8_into::<u8>(BNO055RegisterPage::PAGE_1, regs::BNO055_ACC_HG_DURATION)
            .await?
            .ok_or(Error::AccConfig(acc_config::Error::BadAccHgDuration))
    }

    /// Sets accelerometer High-G interrupt threshold setting
    pub async fn set_acc_hg_threshold<D: DelayNs>(
        &mut self,
        mult: u8,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        self.set_config_from(
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_ACC_HG_THRES,
            mult,
            delay,
        )
        .await
    }

    /// Returns current accelerometer High-G interrupt threshold setting
    pub async fn acc_hg_threshold(&mut self) -> Result<u8, Error<E>> {
        self.read_u8_into::<u8>(BNO055RegisterPage::PAGE_1, regs::BNO055_ACC_HG_THRES)
            .await?
            .ok_or(Error::AccConfig(acc_config::Error::BadAccHgThreshold))
    }

    /// Sets accelerometer no/slow-motion interrupt threshold setting
    pub async fn set_acc_nm_threshold<D: DelayNs>(
        &mut self,
        mult: u8,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        self.set_config_from(
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_ACC_NM_THRES,
            mult,
            delay,
        )
        .await
    }

    /// Returns current accelerometer no/slow-motion interrupt threshold setting
    pub async fn acc_nm_threshold(&mut self) -> Result<u8, Error<E>> {
        self.read_u8_into::<u8>(BNO055RegisterPage::PAGE_1, regs::BNO055_ACC_NM_THRES)
            .await?
            .ok_or(Error::AccConfig(acc_config::Error::BadAccNmThreshold))
    }

    /// Sets accelerometer no/slow-motion interrupt settings
    pub async fn set_acc_nm_settings<D: DelayNs>(
        &mut self,
        settings: BNO055AccNmSettings,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        self.set_config_from(
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_ACC_NM_SET,
            settings,
            delay,
        )
        .await
    }

    /// Returns current accelerometer no/slow-motion interrupt settings
    pub async fn acc_nm_settings(&mut self) -> Result<BNO055AccNmSettings, Error<E>> {
        self.read_u8_into(BNO055RegisterPage::PAGE_1, regs::BNO055_ACC_NM_SET)
            .await?
            .ok_or(Error::AccConfig(acc_config::Error::BadAccNmSettings))
    }

    /// Sets gyroscope interrupt settings
    pub async fn set_gyr_interrupt_settings<D: DelayNs>(
        &mut self,
        settings: BNO055GyrIntSettings,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        self.write_config_flags(
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_GYR_INT_SETTING,
            settings,
            delay,
        )
        .await
    }

    /// Returns the current gyroscope interrupt settings
    pub async fn gyr_interrupt_settings(&mut self) -> Result<BNO055GyrIntSettings, Error<E>> {
        self.read_flags(BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_INT_SETTING)
            .await
    }

    /// Sets gyroscope high-rate interrupt settings for x-axis
    pub async fn set_gyr_hr_x_settings<D: DelayNs>(
        &mut self,
        settings: BNO055GyrHrSettings,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        self.set_config_from(
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_GYR_HR_X_SET,
            settings,
            delay,
        )
        .await
    }

    /// Returns current gyroscope high-rate interrupt settings for x-axis
    pub async fn gyr_hr_x_settings(&mut self) -> Result<BNO055GyrHrSettings, Error<E>> {
        self.read_u8_into(BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_HR_X_SET)
            .await?
            .ok_or(Error::GyroConfig(gyr_config::Error::BadGyrHrSettings))
    }

    /// Sets gyroscope high-rate interrupt duration for x-axis
    pub async fn set_gyr_dur_x<D: DelayNs>(
        &mut self,
        duration: u8,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        self.set_config_from(
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_GYR_DUR_X,
            duration,
            delay,
        )
        .await
    }

    /// Returns current gyroscope high-rate interrupt settings for x-axis
    pub async fn gyr_dur_x(&mut self) -> Result<u8, Error<E>> {
        self.read_u8_into::<u8>(BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_DUR_X)
            .await?
            .ok_or(Error::GyroConfig(gyr_config::Error::BadGyrDurX))
    }

    /// Sets gyroscope high-rate interrupt settings for y-axis
    pub async fn set_gyr_hr_y_settings<D: DelayNs>(
        &mut self,
        settings: BNO055GyrHrSettings,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        self.set_config_from(
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_GYR_HR_Y_SET,
            settings,
            delay,
        )
        .await
    }

    /// Returns current gyroscope high-rate interrupt settings for y-axis
    pub async fn gyr_hr_y_settings(&mut self) -> Result<BNO055GyrHrSettings, Error<E>> {
        self.read_u8_into(BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_HR_Y_SET)
            .await?
            .ok_or(Error::GyroConfig(gyr_config::Error::BadGyrHrYSettings))
    }

    /// Sets gyroscope high-rate interrupt duration for y-axis
    pub async fn set_gyr_dur_y<D: DelayNs>(
        &mut self,
        duration: u8,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        self.set_config_from(
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_GYR_DUR_Y,
            duration,
            delay,
        )
        .await
    }

    /// Returns current gyroscope high-rate interrupt settings for y-axis
    pub async fn gyr_dur_y(&mut self) -> Result<u8, Error<E>> {
        self.read_u8_into::<u8>(BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_DUR_Y)
            .await?
            .ok_or(Error::GyroConfig(gyr_config::Error::BadGyrDurY))
    }

    /// Sets gyroscope high-rate interrupt settings for z-axis
    pub async fn set_gyr_hr_z_settings<D: DelayNs>(
        &mut self,
        settings: BNO055GyrHrSettings,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        self.set_config_from(
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_GYR_HR_Z_SET,
            settings,
            delay,
        )
        .await
    }

    /// Returns current gyroscope high-rate interrupt settings for z-axis
    pub async fn gyr_hr_z_settings(&mut self) -> Result<BNO055GyrHrSettings, Error<E>> {
        self.read_u8_into(BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_HR_Z_SET)
            .await?
            .ok_or(Error::GyroConfig(gyr_config::Error::BadGyrHrZSettings))
    }

    /// Sets gyroscope high-rate interrupt duration for z-axis
    pub async fn set_gyr_dur_z<D: DelayNs>(
        &mut self,
        duration: u8,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        self.set_config_from(
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_GYR_DUR_Z,
            duration,
            delay,
        )
        .await
    }

    /// Returns current gyroscope high-rate interrupt settings for z-axis
    pub async fn gyr_dur_z(&mut self) -> Result<u8, Error<E>> {
        self.read_u8_into::<u8>(BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_DUR_Z)
            .await?
            .ok_or(Error::GyroConfig(gyr_config::Error::BadGyrDurZ))
    }

    /// Sets gyroscope any-motion interrupt threshold
    pub async fn set_gyr_am_threshold<D: DelayNs>(
        &mut self,
        mult: u8,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        let mult = mult & BIT_7_RESERVED_MASK; // Ensure reserved bit is 0
        self.set_config_from(
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_GYR_AM_THRES,
            mult,
            delay,
        )
        .await
    }

    /// Returns current gyroscope high-rate interrupt settings for z-axis
    pub async fn gyr_am_threshold(&mut self) -> Result<u8, Error<E>> {
        let mult = self
            .read_u8_into::<u8>(BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_AM_THRES)
            .await?
            .ok_or(Error::GyroConfig(gyr_config::Error::BadGyrAmThreshold))?;
        Ok(mult & BIT_7_RESERVED_MASK)
    }

    /// Sets gyroscope any-motion interrupt settings
    pub async fn set_gyr_am_settings<D: DelayNs>(
        &mut self,
        settings: BNO055GyrAmSettings,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        self.set_config_from(
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_GYR_AM_SET,
            settings,
            delay,
        )
        .await
    }

    /// Returns current gyroscope any-motion interrupt settings
    pub async fn gyr_am_settings(&mut self) -> Result<BNO055GyrAmSettings, Error<E>> {
        self.read_u8_into(BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_AM_SET)
            .await?
            .ok_or(Error::GyroConfig(gyr_config::Error::BadGyrAmSettings))
    }

    // ------------------
    // Private helper functions
    // ------------------

    #[inline(always)]
    fn i2c_addr(&self) -> u8 {
        if self.use_default_addr {
            regs::BNO055_DEFAULT_ADDR
        } else {
            regs::BNO055_ALTERNATE_ADDR
        }
    }

    /// Sets current register map page.
    async fn set_page(&mut self, page: BNO055RegisterPage) -> Result<(), Error<E>> {
        self.write_u8(regs::BNO055_PAGE_ID, page.bits()).await
    }

    /// Reads a vector of sensor data from the device.
    async fn read_vec_raw(&mut self, reg: u8) -> Result<mint::Vector3<i16>, Error<E>> {
        let mut buf: [u8; 6] = [0; 6];
        self.read_bytes(reg, &mut buf).await?;
        let x = LittleEndian::read_i16(&buf[0..2]);
        let y = LittleEndian::read_i16(&buf[2..4]);
        let z = LittleEndian::read_i16(&buf[4..6]);
        Ok(mint::Vector3::from([x, y, z]))
    }

    /// Applies the given scaling to the vector of sensor data from the device.
    fn scale_vec(raw: mint::Vector3<i16>, scaling: f32) -> mint::Vector3<f32> {
        mint::Vector3::from([
            raw.x as f32 * scaling,
            raw.y as f32 * scaling,
            raw.z as f32 * scaling,
        ])
    }

    /// Helper to set a value from a type that can be converted to u8
    async fn set_u8_from<T: Into<u8> + Send>(
        &mut self,
        page: BNO055RegisterPage,
        reg: u8,
        x: T,
    ) -> Result<(), Error<E>> {
        self.set_page(page).await?;
        self.write_u8(reg, x.into()).await?;
        Ok(())
    }

    /// Helper to set a configuration value, which requires switching to CONFIG_MODE
    async fn set_config_from<T: Into<u8> + Send, D: DelayNs>(
        &mut self,
        page: BNO055RegisterPage,
        reg: u8,
        x: T,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        let prev = self.mode;
        self.set_mode(BNO055OperationMode::CONFIG_MODE, delay)
            .await?;
        let res = self.set_u8_from(page, reg, x).await;
        self.set_mode(prev, delay).await?;
        res
    }

    /// Helper to read a u8 value and convert it to a target type
    async fn read_u8_into<T: FromPrimitive>(
        &mut self,
        page: BNO055RegisterPage,
        reg: u8,
    ) -> Result<Option<T>, Error<E>> {
        self.set_page(page).await?;
        let regval = self.read_u8(reg).await?;
        Ok(T::from_u8(regval))
    }

    /// Helper to write bitflags to a register
    async fn write_flags<F>(
        &mut self,
        page: BNO055RegisterPage,
        reg: u8,
        flags: F,
    ) -> Result<(), Error<E>>
    where
        F: bitflags::Flags<Bits = u8> + Send,
    {
        self.set_page(page).await?;
        self.write_u8(reg, flags.bits()).await
    }

    /// Helper to write configuration bitflags, which requires switching to CONFIG_MODE
    async fn write_config_flags<F, D>(
        &mut self,
        page: BNO055RegisterPage,
        reg: u8,
        flags: F,
        delay: &mut D,
    ) -> Result<(), Error<E>>
    where
        F: bitflags::Flags<Bits = u8> + Send,
        D: DelayNs,
    {
        let prev = self.mode;
        self.set_mode(BNO055OperationMode::CONFIG_MODE, delay)
            .await?;
        let res = self.write_flags(page, reg, flags).await;
        self.set_mode(prev, delay).await?;
        res
    }

    /// Helper to read bitflags from a register
    async fn read_flags<F>(&mut self, page: BNO055RegisterPage, reg: u8) -> Result<F, Error<E>>
    where
        F: bitflags::Flags<Bits = u8>,
    {
        self.set_page(page).await?;
        let bits = self.read_u8(reg).await?;
        Ok(F::from_bits_truncate(bits))
    }

    /// Low-level I2C read of a single u8
    async fn read_u8(&mut self, reg: u8) -> Result<u8, Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        (self
            .i2c
            .write_read(self.i2c_addr(), &[reg], &mut byte)
            .await)
            .map_err(Error::I2c)?;
        Ok(byte[0])
    }

    /// Low-level I2C read of multiple bytes
    async fn read_bytes(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<E>> {
        (self.i2c.write_read(self.i2c_addr(), &[reg], buf).await).map_err(Error::I2c)
    }

    /// Low-level I2C write of a single u8
    async fn write_u8(&mut self, reg: u8, value: u8) -> Result<(), Error<E>> {
        (self.i2c.write(self.i2c_addr(), &[reg, value]).await).map_err(Error::I2c)
    }

    /// Low-level I2C write of multiple bytes
    async fn write_bytes(&mut self, reg: u8, values: &[u8]) -> Result<(), Error<E>> {
        let mut buffer = [0u8; BNO055_CALIB_SIZE + 1];
        buffer[0] = reg;
        buffer[1..values.len() + 1].copy_from_slice(values);
        (self
            .i2c
            .write(self.i2c_addr(), &buffer[..=values.len()])
            .await)
            .map_err(Error::I2c)
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct BNO055AxisConfig: u8 {
        const AXIS_AS_X = 0b00;
        const AXIS_AS_Y = 0b01;
        const AXIS_AS_Z = 0b10;
    }
}

#[cfg(feature = "defmt-03")]
impl defmt::Format for BNO055AxisConfig {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "BNO055AxisConfig({})",
            match self.bits() {
                0b00 => "X",
                0b01 => "Y",
                0b10 => "Z",
                _ => "Unknown",
            }
        )
    }
}

#[allow(clippy::misnamed_getters)]
impl AxisRemap {
    pub fn x(&self) -> BNO055AxisConfig {
        self.x
    }

    pub fn y(&self) -> BNO055AxisConfig {
        self.x
    }

    pub fn z(&self) -> BNO055AxisConfig {
        self.z
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct AxisRemap {
    x: BNO055AxisConfig,
    y: BNO055AxisConfig,
    z: BNO055AxisConfig,
}

#[derive(Debug)]
pub struct AxisRemapBuilder {
    remap: AxisRemap,
}

impl AxisRemap {
    pub fn builder() -> AxisRemapBuilder {
        AxisRemapBuilder {
            remap: AxisRemap {
                x: BNO055AxisConfig::AXIS_AS_X,
                y: BNO055AxisConfig::AXIS_AS_Y,
                z: BNO055AxisConfig::AXIS_AS_Z,
            },
        }
    }
}

impl AxisRemapBuilder {
    pub fn swap_x_with(mut self, to: BNO055AxisConfig) -> AxisRemapBuilder {
        let old_x = self.remap.x;

        match to {
            BNO055AxisConfig::AXIS_AS_X => self.remap.x = old_x,
            BNO055AxisConfig::AXIS_AS_Y => self.remap.y = old_x,
            BNO055AxisConfig::AXIS_AS_Z => self.remap.z = old_x,

            _ => (),
        }

        self.remap.x = to;

        AxisRemapBuilder { remap: self.remap }
    }

    pub fn swap_y_with(mut self, to: BNO055AxisConfig) -> AxisRemapBuilder {
        let old_y = self.remap.y;

        match to {
            BNO055AxisConfig::AXIS_AS_X => self.remap.x = old_y,
            BNO055AxisConfig::AXIS_AS_Y => self.remap.y = old_y,
            BNO055AxisConfig::AXIS_AS_Z => self.remap.z = old_y,

            _ => (),
        }

        self.remap.y = to;

        AxisRemapBuilder { remap: self.remap }
    }

    pub fn swap_z_with(mut self, to: BNO055AxisConfig) -> AxisRemapBuilder {
        let old_z = self.remap.z;

        match to {
            BNO055AxisConfig::AXIS_AS_X => self.remap.x = old_z,
            BNO055AxisConfig::AXIS_AS_Y => self.remap.y = old_z,
            BNO055AxisConfig::AXIS_AS_Z => self.remap.z = old_z,

            _ => (),
        }

        self.remap.z = to;

        AxisRemapBuilder { remap: self.remap }
    }

    fn is_invalid(&self) -> bool {
        // Each axis must be swapped only once,
        // For example, one cannot remap X to Y and Z to Y at the same time, or similar.
        // See datasheet, section 3.4.
        self.remap.x == self.remap.y || self.remap.y == self.remap.z || self.remap.z == self.remap.x
    }

    #[allow(clippy::result_unit_err)]
    pub fn build(self) -> Result<AxisRemap, ()> {
        if self.is_invalid() {
            Err(())
        } else {
            Ok(self.remap)
        }
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct BNO055AxisSign: u8 {
        const X_NEGATIVE = 0b100;
        const Y_NEGATIVE = 0b010;
        const Z_NEGATIVE = 0b001;
    }
}

#[cfg(feature = "defmt-03")]
impl defmt::Format for BNO055AxisSign {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "BNO055AxisSign(");
        if self.is_empty() {
            defmt::write!(f, "Positive");
        } else {
            if self.contains(BNO055AxisSign::X_NEGATIVE) {
                defmt::write!(f, "X_NEGATIVE ");
            }
            if self.contains(BNO055AxisSign::Y_NEGATIVE) {
                defmt::write!(f, "Y_NEGATIVE ");
            }
            if self.contains(BNO055AxisSign::Z_NEGATIVE) {
                defmt::write!(f, "Z_NEGATIVE ");
            }
        }
        defmt::write!(f, ")");
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    /// SYS_STATUS 0x39
    pub struct BNO055SystemStatusCode: u8 {
        /// 0 System idle
        const SYSTEM_IDLE = 0;
        /// 1 System Error
        const SYSTEM_ERROR = 1;
        /// 2 Initializing peripherals
        const INIT_PERIPHERALS = 2;
        /// 3 System Initialization
        const SYSTEM_INIT = 3;
        /// 4 Executing selftest
        const EXECUTING = 4;
        /// 5 Sensor fusion algorithm running
        const RUNNING = 5;
        /// 6 System running without fusion algorithm
        const RUNNING_WITHOUT_FUSION = 6;
    }
}

#[cfg(feature = "defmt-03")]
impl defmt::Format for BNO055SystemStatusCode {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "BNO055SystemStatusCode({})",
            match self.bits() {
                0 => "SystemIdle",
                1 => "SystemError",
                2 => "InitPeripherals",
                3 => "SystemInit",
                4 => "Executing",
                5 => "Running",
                6 => "RunningWithoutFusion",
                _ => "Unknown",
            }
        )
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct BNO055SystemErrorCode: u8 {
        const NONE = 0;
        const PERIPHERAL_INIT = 1;
        const SYSTEM_INIT = 2;
        const SELF_TEST = 3;
        const REGISTER_MAP_VALUE = 4;
        const REGISTER_MAP_ADDRESS = 5;
        const REGISTER_MAP_WRITE = 6;
        const LOW_POWER_MODE_NOT_AVAIL = 7;
        const ACCEL_POWER_MODE_NOT_AVAIL = 8;
        const FUSION_ALGO_CONFIG = 9;
        const SENSOR_CONFIG = 10;
    }
}

#[cfg(feature = "defmt-03")]
impl defmt::Format for BNO055SystemErrorCode {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "BNO055SystemErrorCode({})",
            match self.bits() {
                0 => "None",
                1 => "PeripheralInit",
                2 => "SystemInit",
                3 => "SelfTest",
                4 => "RegisterMapValue",
                5 => "RegisterMapAddress",
                6 => "RegisterMapWrite",
                7 => "LowPowerModeNotAvail",
                8 => "AccelPowerModeNotAvail",
                9 => "FusionAlgoConfig",
                10 => "SensorConfig",
                _ => "Unknown",
            }
        )
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct BNO055SelfTestStatus: u8 {
        const ACC_OK = 0b0001;
        const MAG_OK = 0b0010;
        const GYR_OK = 0b0100;
        const MCU_OK = 0b1000; // Corrected from SYS_OK to match datasheet
    }
}

#[cfg(feature = "defmt-03")]
impl defmt::Format for BNO055SelfTestStatus {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "BNO055SelfTestStatus(");
        if self.is_empty() {
            defmt::write!(f, "None");
        } else {
            if self.contains(BNO055SelfTestStatus::ACC_OK) {
                defmt::write!(f, "ACC_OK ");
            }
            if self.contains(BNO055SelfTestStatus::MAG_OK) {
                defmt::write!(f, "MAG_OK ");
            }
            if self.contains(BNO055SelfTestStatus::GYR_OK) {
                defmt::write!(f, "GYR_OK ");
            }
            if self.contains(BNO055SelfTestStatus::MCU_OK) {
                defmt::write!(f, "MCU_OK ");
            }
        }
        defmt::write!(f, ")");
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct BNO055SystemStatus {
    status: BNO055SystemStatusCode,
    selftest: Option<BNO055SelfTestStatus>,
    error: BNO055SystemErrorCode,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
// #[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct BNO055Revision {
    pub software: u16,
    pub bootloader: u8,
    pub accelerometer: u8,
    pub magnetometer: u8,
    pub gyroscope: u8,
}

#[cfg(feature = "defmt-03")]
impl defmt::Format for BNO055Revision {
    fn format(&self, f: defmt::Formatter) {
        let [major, minor] = self.software.to_be_bytes();
        defmt::write!(
            f,
            "BNO055Revision {{ software: {=u8}.{=u8}, bootloader: {=u8}, accelerometer: {=u8}, magnetometer: {=u8}, gyroscope: {=u8} }}",
            major, minor, self.bootloader, self.accelerometer, self.magnetometer, self.gyroscope
        )
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[repr(C)]
pub struct BNO055Calibration {
    pub acc_offset_x_lsb: u8,
    pub acc_offset_x_msb: u8,
    pub acc_offset_y_lsb: u8,
    pub acc_offset_y_msb: u8,
    pub acc_offset_z_lsb: u8,
    pub acc_offset_z_msb: u8,

    pub mag_offset_x_lsb: u8,
    pub mag_offset_x_msb: u8,
    pub mag_offset_y_lsb: u8,
    pub mag_offset_y_msb: u8,
    pub mag_offset_z_lsb: u8,
    pub mag_offset_z_msb: u8,

    pub gyr_offset_x_lsb: u8,
    pub gyr_offset_x_msb: u8,
    pub gyr_offset_y_lsb: u8,
    pub gyr_offset_y_msb: u8,
    pub gyr_offset_z_lsb: u8,
    pub gyr_offset_z_msb: u8,

    pub acc_radius_lsb: u8,
    pub acc_radius_msb: u8,
    pub mag_radius_lsb: u8,
    pub mag_radius_msb: u8,
}

/// BNO055's calibration profile size.
pub const BNO055_CALIB_SIZE: usize = core::mem::size_of::<BNO055Calibration>();

impl BNO055Calibration {
    pub fn from_buf(buf: &[u8; BNO055_CALIB_SIZE]) -> BNO055Calibration {
        unsafe { core::ptr::read(buf.as_ptr() as *const _) }
    }

    pub fn as_bytes(&self) -> &[u8] {
        unsafe { core::slice::from_raw_parts((self as *const _) as *const u8, BNO055_CALIB_SIZE) }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct BNO055CalibrationStatus {
    pub sys: u8,
    pub gyr: u8,
    pub acc: u8,
    pub mag: u8,
}

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct BNO055RegisterPage: u8 {
        const PAGE_0 = 0;
        const PAGE_1 = 1;
    }
}

#[cfg(feature = "defmt-03")]
impl defmt::Format for BNO055RegisterPage {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "BNO055RegisterPage({})",
            match self.bits() {
                0 => "Page0",
                1 => "Page1",
                _ => "Unknown",
            }
        )
    }
}

bitflags! {
    #[derive(Default, Debug, Clone, Copy, PartialEq, Eq)]
    pub struct BNO055PowerMode: u8 {
        const NORMAL = 0b00;
        const LOW_POWER = 0b01;
        const SUSPEND = 0b10;
    }
}

#[cfg(feature = "defmt-03")]
impl defmt::Format for BNO055PowerMode {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "BNO055PowerMode({})",
            match self.bits() {
                0b00 => "Normal",
                0b01 => "LowPower",
                0b10 => "Suspend",
                _ => "Unknown",
            }
        )
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct BNO055OperationMode: u8 {
        const CONFIG_MODE = 0b0000;
        const ACC_ONLY = 0b0001;
        const MAG_ONLY = 0b0010;
        const GYRO_ONLY = 0b0011;
        const ACC_MAG = 0b0100;
        const ACC_GYRO = 0b0101;
        const MAG_GYRO = 0b0110;
        const AMG = 0b0111;
        const IMU = 0b1000;
        const COMPASS = 0b1001;
        const M4G = 0b1010;
        const NDOF_FMC_OFF = 0b1011;
        const NDOF = 0b1100;
    }
}

impl Default for BNO055OperationMode {
    fn default() -> Self {
        Self::CONFIG_MODE
    }
}

#[cfg(feature = "defmt-03")]
impl defmt::Format for BNO055OperationMode {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "BNO055OperationMode({})",
            match self.bits() {
                0b0000 => "ConfigMode",
                0b0001 => "AccOnly",
                0b0010 => "MagOnly",
                0b0011 => "GyroOnly",
                0b0100 => "AccMag",
                0b0101 => "AccGyro",
                0b0110 => "MagGyro",
                0b0111 => "Amg",
                0b1000 => "Imu",
                0b1001 => "Compass",
                0b1010 => "M4g",
                0b1011 => "NdofFmcOff",
                0b1100 => "Ndof",
                _ => "Unknown",
            }
        )
    }
}

impl BNO055OperationMode {
    fn is_fusion_enabled(&self) -> bool {
        matches!(
            *self,
            Self::IMU | Self::COMPASS | Self::M4G | Self::NDOF_FMC_OFF | Self::NDOF,
        )
    }

    fn is_accel_enabled(&self) -> bool {
        matches!(
            *self,
            Self::ACC_ONLY
                | Self::ACC_MAG
                | Self::ACC_GYRO
                | Self::AMG
                | Self::IMU
                | Self::COMPASS
                | Self::M4G
                | Self::NDOF_FMC_OFF
                | Self::NDOF,
        )
    }

    fn is_gyro_enabled(&self) -> bool {
        matches!(
            *self,
            Self::GYRO_ONLY
                | Self::ACC_GYRO
                | Self::MAG_GYRO
                | Self::AMG
                | Self::IMU
                | Self::NDOF_FMC_OFF
                | Self::NDOF,
        )
    }

    fn is_mag_enabled(&self) -> bool {
        matches!(
            *self,
            Self::MAG_ONLY
                | Self::ACC_MAG
                | Self::MAG_GYRO
                | Self::AMG
                | Self::COMPASS
                | Self::M4G
                | Self::NDOF_FMC_OFF
                | Self::NDOF,
        )
    }
}

bitflags! {
    #[derive(Default, Debug, Clone, Copy, PartialEq, Eq)]
    pub struct BNO055Interrupt: u8 {
        const ACC_NM = 0b10000000;
        const ACC_AM = 0b01000000;
        const ACC_HIGH_G = 0b00100000;
        const GYR_DRDY = 0b00010000;
        const GYR_HIGH_RATE = 0b00001000;
        const GYRO_AM = 0b00000100;
        const MAG_DRDY = 0b00000010;
        const ACC_BSX_DRDY = 0b00000001;
    }
}

impl FromPrimitive for BNO055Interrupt {
    /// Converts an i64 to a `BNO055Interrupt` if it's a valid bit pattern.
    fn from_i64(n: i64) -> Option<Self> {
        u8::from_i64(n).and_then(Self::from_bits)
    }

    /// Converts a u64 to a `BNO055Interrupt` if it's a valid bit pattern.
    fn from_u64(n: u64) -> Option<Self> {
        u8::from_u64(n).and_then(Self::from_bits)
    }

    /// Converts a u8 to a `BNO055Interrupt` if it's a valid bit pattern.
    fn from_u8(n: u8) -> Option<Self> {
        Self::from_bits(n)
    }
}

#[cfg(feature = "defmt-03")]
impl defmt::Format for BNO055Interrupt {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "BNO055Interrupt(");
        if self.is_empty() {
            defmt::write!(f, "None");
        } else {
            if self.contains(BNO055Interrupt::ACC_NM) {
                defmt::write!(f, "ACC_NM ");
            }
            if self.contains(BNO055Interrupt::ACC_AM) {
                defmt::write!(f, "ACC_AM ");
            }
            if self.contains(BNO055Interrupt::ACC_HIGH_G) {
                defmt::write!(f, "ACC_HIGH_G ");
            }
            if self.contains(BNO055Interrupt::GYR_DRDY) {
                defmt::write!(f, "GYR_DRDY ");
            }
            if self.contains(BNO055Interrupt::GYR_HIGH_RATE) {
                defmt::write!(f, "GYR_HIGH_RATE ");
            }
            if self.contains(BNO055Interrupt::GYRO_AM) {
                defmt::write!(f, "GYRO_AM ");
            }
            if self.contains(BNO055Interrupt::MAG_DRDY) {
                defmt::write!(f, "MAG_DRDY ");
            }
            if self.contains(BNO055Interrupt::ACC_BSX_DRDY) {
                defmt::write!(f, "ACC_BSX_DRDY ");
            }
        }
        defmt::write!(f, ")");
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct BNO055SystemTrigger: u8 {
        /// Select External Clock
        const EXT_CLK_SEL = 0b1000_0000;
        /// Clear interrupts command
        const RST_INT = 0b0100_0000;
        /// Reset command
        const RST_SYS = 0b0010_0000;
        /// Self-test command
        const SELF_TEST = 0b0000_0001;
    }
}

impl FromPrimitive for BNO055SystemTrigger {
    /// Converts an i64 to a `BNO055SystemTrigger` if it's a valid bit pattern.
    fn from_i64(n: i64) -> Option<Self> {
        u8::from_i64(n).and_then(Self::from_bits)
    }

    /// Converts a u64 to a `BNO055SystemTrigger` if it's a valid bit pattern.
    fn from_u64(n: u64) -> Option<Self> {
        u8::from_u64(n).and_then(Self::from_bits)
    }

    /// Converts a u8 to a `BNO055SystemTrigger` if it's a valid bit pattern.
    fn from_u8(n: u8) -> Option<Self> {
        Self::from_bits(n)
    }
}

#[cfg(feature = "defmt-03")]
impl defmt::Format for BNO055SystemTrigger {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "BNO055SystemTrigger(");
        if self.is_empty() {
            defmt::write!(f, "None");
        } else {
            if self.contains(BNO055SystemTrigger::EXT_CLK_SEL) {
                defmt::write!(f, "EXT_CLK_SEL ");
            }
            if self.contains(BNO055SystemTrigger::RST_INT) {
                defmt::write!(f, "RST_INT ");
            }
            if self.contains(BNO055SystemTrigger::RST_SYS) {
                defmt::write!(f, "RST_SYS ");
            }
            if self.contains(BNO055SystemTrigger::SELF_TEST) {
                defmt::write!(f, "SELF_TEST ");
            }
        }
        defmt::write!(f, ")");
    }
}

#[cfg(test)]
mod tests {
    use crate::BNO055Interrupt;

    #[test]
    fn test_interrupts() {
        assert!(BNO055Interrupt::ACC_NM.contains(BNO055Interrupt::ACC_NM));
        assert!((BNO055Interrupt::ACC_NM | BNO055Interrupt::ACC_HIGH_G)
            .contains(BNO055Interrupt::ACC_NM));
        assert!((BNO055Interrupt::all()).contains(BNO055Interrupt::ACC_NM));
        assert!((BNO055Interrupt::all()).contains(BNO055Interrupt::ACC_AM));
        assert!((BNO055Interrupt::all()).contains(BNO055Interrupt::ACC_HIGH_G));
        assert!((BNO055Interrupt::all()).contains(BNO055Interrupt::GYR_DRDY));
        assert!((BNO055Interrupt::all()).contains(BNO055Interrupt::GYR_HIGH_RATE));
        assert!((BNO055Interrupt::all()).contains(BNO055Interrupt::GYRO_AM));
        assert!((BNO055Interrupt::all()).contains(BNO055Interrupt::MAG_DRDY));
        assert!((BNO055Interrupt::all()).contains(BNO055Interrupt::ACC_BSX_DRDY));

        assert!(
            BNO055Interrupt::from_bits_truncate(0b10011).contains(BNO055Interrupt::ACC_BSX_DRDY)
        );
        assert!(BNO055Interrupt::from_bits_truncate(0b10011).contains(BNO055Interrupt::GYR_DRDY));
        assert!(!BNO055Interrupt::from_bits_truncate(0b10011).contains(BNO055Interrupt::MAG_DRDY));
    }
}
