#![doc(html_root_url = "https://docs.rs/bno055/0.4.0")]
#![cfg_attr(not(feature = "std"), no_std)]
#![allow(clippy::bad_bit_mask)]

//! Bosch Sensortec BNO055 9-axis IMU sensor driver.
//! Datasheet: https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BNO055-DS000.pdf
use embedded_hal::{
    delay::DelayNs,
    i2c::{I2c, SevenBitAddress},
};

use bitflags::bitflags;
// #[cfg(not(feature = "defmt-03"))]
// #[cfg(feature = "defmt-03")]
// use defmt::bitflags;

use byteorder::{ByteOrder, LittleEndian};
pub use mint;
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

mod acc_config;
mod gyr_config;
mod regs;
#[cfg(feature = "std")]
mod std;

#[doc(inline)]
pub use acc_config::{
    AccBandwidth, AccConfig, AccGRange, AccOperationMode, BNO055AccIntSettings, BNO055AccNmSettings,
};
#[doc(inline)]
pub use gyr_config::{
    BNO055GyrAmSettings, BNO055GyrHrSettings, BNO055GyrIntSettings, GyrAmSamplesAwake,
};
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
}

#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct Bno055<I> {
    i2c: I,
    pub mode: BNO055OperationMode,
    use_default_addr: bool,
}

macro_rules! set_u8_from {
    ($bno055 : expr, $page : expr, $reg : expr, $x : expr) => {{
        $bno055.set_page($page)?;
        $bno055.write_u8($reg, $x.into()).map_err(Error::I2c)?;
        Ok(())
    }};
}

macro_rules! set_config_from {
    ($bno055 : expr, $page : expr, $reg : expr, $x : expr, $delay : expr) => {{
        let prev = $bno055.mode;
        $bno055.set_mode(BNO055OperationMode::CONFIG_MODE, $delay)?;
        let res = set_u8_from!($bno055, $page, $reg, $x);
        $bno055.set_mode(prev, $delay)?;
        res
    }};
}

macro_rules! read_u8_into {
    ($bno055 : expr, $page : expr, $reg : expr) => {{
        $bno055.set_page($page)?;
        let regval = $bno055.read_u8($reg).map_err(Error::I2c)?;
        <_ as num_traits::FromPrimitive>::from_u8(regval)
    }};
}

macro_rules! write_flags {
    ($bno055 : expr, $page : expr, $reg : expr, $flags : expr) => {{
        $bno055.set_page($page)?;
        $bno055.write_u8($reg, $flags.bits()).map_err(Error::I2c)?;
        Ok(())
    }};
}

macro_rules! write_config_flags {
    ($bno055 : expr, $page : expr, $reg : expr, $flags : expr, $delay : expr) => {{
        let prev = $bno055.mode;
        $bno055.set_mode(BNO055OperationMode::CONFIG_MODE, $delay)?;
        let res = write_flags!($bno055, $page, $reg, $flags);
        $bno055.set_mode(prev, $delay)?;
        res
    }};
}

macro_rules! read_flags {
    ($bno055 : expr, $page : expr, $reg : expr, $flag_type : ty) => {{
        $bno055.set_page($page)?;
        let flags = $bno055.read_u8($reg).map_err(Error::I2c)?;
        let flags = <$flag_type>::from_bits_truncate(flags);
        Ok(flags)
    }};
}

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
    ///
    /// # Usage Example
    ///
    /// ```rust
    /// // use your_chip_hal::{I2c, Delay}; // <- import your chip's I2c and Delay
    /// use bno055::Bno055;
    /// #
    /// # // All of this is needed for example to work:
    /// # use bno055::BNO055_ID;
    /// # use embedded_hal::delay::DelayNs;
    /// # use embedded_hal::i2c::{I2c as I2cTrait, Operation, Error, ErrorType, ErrorKind};
    /// # struct Delay {}
    /// # impl Delay { pub fn new() -> Self { Delay{ } }}
    /// # impl DelayNs for Delay {
    /// #    fn delay_ns(&mut self, ms: u32) {
    /// #        // no-op for example purposes
    /// #    }
    /// # }
    /// # struct I2c {}
    /// # impl I2c { pub fn new() -> Self { I2c { } }}
    /// # #[derive(Debug)]
    /// # struct DummyError {}
    /// # impl Error for DummyError { fn kind(&self) -> ErrorKind { ErrorKind::Other } }
    /// # impl ErrorType for I2c { type Error = DummyError; }
    /// # // 3 calls are made, 2 Writes and 1 Write/Read. We want to mock the 3rd call's read.
    /// # impl I2cTrait for I2c { fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Self::Error> { match operations.get_mut(1) { Some(Operation::Read(read)) => { read[0] = BNO055_ID; }, _ => {} }; Ok(()) } }
    /// #
    /// # // Actual example:
    /// let mut delay = Delay::new(/* ... */);
    /// let mut i2c = I2c::new(/* ... */);
    /// let mut bno055 = Bno055::new(i2c);
    /// bno055.init(&mut delay)?;
    /// # Result::<(), bno055::Error<DummyError>>::Ok(())
    /// ```
    pub fn init(&mut self, delay: &mut dyn DelayNs) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        let id = self.id()?;
        if id != regs::BNO055_ID {
            return Err(Error::InvalidChipId(id));
        }

        self.soft_reset(delay)?;
        self.set_mode(BNO055OperationMode::CONFIG_MODE, delay)?;
        self.set_power_mode(BNO055PowerMode::NORMAL)?;
        self.write_u8(regs::BNO055_SYS_TRIGGER, 0x00)
            .map_err(Error::I2c)?;

        Ok(())
    }

    /// Resets the BNO055, initializing the register map to default values.
    /// More in section 3.2.
    pub fn soft_reset(&mut self, delay: &mut dyn DelayNs) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        self.write_u8(
            regs::BNO055_SYS_TRIGGER,
            BNO055SystemTrigger::RST_SYS.bits(),
        )
        .map_err(Error::I2c)?;

        // As per table 1.2
        delay.delay_ms(650);
        Ok(())
    }

    /// Run Self-test on the BNO055
    ///
    /// See section 3.9.2 Built-In Self-Test (BIST)
    ///
    /// To know if the BIST succeeded/failed you should:
    /// 1. Trigger BIST
    /// 2. Wait for 400ms
    /// 3. Read `SYS_ERROR` register (`0x3A`):
    /// - `SYS_ERROR` will remain at 0 in case of success (0 = No error)
    /// - `SYS_ERROR` will show 3 in case of self-test failure (3 = Self-test result failed)
    /// 4. In case of failed BIST (`SYS_ERROR`` != 0 above), you can see which sensor
    /// failed by reading the `ST_RESULT` (`0x36`) register (bit of the corresponding
    /// sensor is ‘1’ if self-test was successful, but will show ‘0’ if the self-test failed).
    ///
    /// | Components      | Test type          |
    /// | --------------- | ------------------ |
    /// | Accelerometer   | built in self-test |
    /// | Magnetometer    | built in self-test |
    /// | Gyroscope       | built in self-test |
    /// | Microcontroller | No test performed  |
    pub fn self_test(&mut self, delay: &mut dyn DelayNs) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        let prev = self.mode;

        self.set_mode(BNO055OperationMode::CONFIG_MODE, delay)?;

        self.write_u8(
            regs::BNO055_SYS_TRIGGER,
            BNO055SystemTrigger::RST_SYS.bits(),
        )
        .map_err(Error::I2c)?;

        self.set_mode(prev, delay)?;

        Ok(())
    }
    /// Sets the operating mode, see [BNO055OperationMode].
    /// See section 3.3.
    pub fn set_mode(
        &mut self,
        mode: BNO055OperationMode,
        delay: &mut dyn DelayNs,
    ) -> Result<(), Error<E>> {
        if self.mode != mode {
            self.set_page(BNO055RegisterPage::PAGE_0)?;

            self.mode = mode;

            self.write_u8(regs::BNO055_OPR_MODE, mode.bits())
                .map_err(Error::I2c)?;

            // Table 3-6 says 19ms to switch to CONFIG_MODE
            delay.delay_ms(19);
        }

        Ok(())
    }

    /// Sets the power mode, see [BNO055PowerMode](enum.BNO055PowerMode.html)
    /// See section 3.2
    pub fn set_power_mode(&mut self, mode: BNO055PowerMode) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        self.write_u8(regs::BNO055_PWR_MODE, mode.bits())
            .map_err(Error::I2c)?;

        Ok(())
    }

    /// Returns BNO055's power mode.
    pub fn power_mode(&mut self) -> Result<BNO055PowerMode, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        let mode = self.read_u8(regs::BNO055_PWR_MODE).map_err(Error::I2c)?;

        Ok(BNO055PowerMode::from_bits_truncate(mode))
    }

    /// Enables/Disables usage of external 32k crystal.
    ///
    /// > It takes minimum ~600ms to configure the external crystal and startup the BNO055
    ///
    /// See section 5.5.1 External 32kHz Crystal Oscillator
    pub fn set_external_crystal(
        &mut self,
        ext: bool,
        delay: &mut dyn DelayNs,
    ) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        let prev = self.mode;
        self.set_mode(BNO055OperationMode::CONFIG_MODE, delay)?;
        self.write_u8(
            regs::BNO055_SYS_TRIGGER,
            if ext {
                BNO055SystemTrigger::EXT_CLK_SEL.bits()
            } else {
                0x00
            },
        )
        .map_err(Error::I2c)?;

        self.set_mode(prev, delay)?;

        Ok(())
    }

    /// Configures axis remap of the device.
    pub fn set_axis_remap(&mut self, remap: AxisRemap) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        let remap_value = (remap.x.bits() & 0b11)
            | ((remap.y.bits() & 0b11) << 2)
            | ((remap.z.bits() & 0b11) << 4);

        self.write_u8(regs::BNO055_AXIS_MAP_CONFIG, remap_value)
            .map_err(Error::I2c)?;

        Ok(())
    }

    /// Returns axis remap of the device.
    pub fn axis_remap(&mut self) -> Result<AxisRemap, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        let value = self
            .read_u8(regs::BNO055_AXIS_MAP_CONFIG)
            .map_err(Error::I2c)?;

        let remap = AxisRemap {
            x: BNO055AxisConfig::from_bits_truncate(value & 0b11),
            y: BNO055AxisConfig::from_bits_truncate((value >> 2) & 0b11),
            z: BNO055AxisConfig::from_bits_truncate((value >> 4) & 0b11),
        };

        Ok(remap)
    }

    /// Configures device's axes sign: positive or negative.
    pub fn set_axis_sign(&mut self, sign: BNO055AxisSign) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        self.write_u8(regs::BNO055_AXIS_MAP_SIGN, sign.bits())
            .map_err(Error::I2c)?;

        Ok(())
    }

    /// Return device's axes sign.
    pub fn axis_sign(&mut self) -> Result<BNO055AxisSign, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        let value = self
            .read_u8(regs::BNO055_AXIS_MAP_SIGN)
            .map_err(Error::I2c)?;

        Ok(BNO055AxisSign::from_bits_truncate(value))
    }

    /// Gets the revision of software, bootloader, accelerometer, magnetometer, and gyroscope of
    /// the BNO055 device.
    pub fn get_revision(&mut self) -> Result<BNO055Revision, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        let mut buf: [u8; 6] = [0; 6];

        self.read_bytes(regs::BNO055_ACC_ID, &mut buf)
            .map_err(Error::I2c)?;

        Ok(BNO055Revision {
            software: LittleEndian::read_u16(&buf[3..5]),
            bootloader: buf[5],
            accelerometer: buf[0],
            magnetometer: buf[1],
            gyroscope: buf[2],
        })
    }

    /// Returns device's system status.
    pub fn get_system_status(
        &mut self,
        do_selftest: bool,
        delay: &mut dyn DelayNs,
    ) -> Result<BNO055SystemStatus, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        let selftest = if do_selftest {
            let prev = self.mode;
            self.set_mode(BNO055OperationMode::CONFIG_MODE, delay)?;

            let sys_trigger = self.read_u8(regs::BNO055_SYS_TRIGGER).map_err(Error::I2c)?;

            self.write_u8(regs::BNO055_SYS_TRIGGER, sys_trigger | 0x1)
                .map_err(Error::I2c)?;

            // Wait for self-test result
            for _ in 0..4 {
                delay.delay_ms(255);
            }

            let result = self.read_u8(regs::BNO055_ST_RESULT).map_err(Error::I2c)?;

            self.set_mode(prev, delay)?; // Restore previous mode

            Some(BNO055SelfTestStatus::from_bits_truncate(result))
        } else {
            None
        };

        let status = self.read_u8(regs::BNO055_SYS_STATUS).map_err(Error::I2c)?;
        let error = self.read_u8(regs::BNO055_SYS_ERR).map_err(Error::I2c)?;

        Ok(BNO055SystemStatus {
            status: BNO055SystemStatusCode::from_bits_truncate(status),
            error: BNO055SystemErrorCode::from_bits_truncate(error),
            selftest,
        })
    }

    /// Gets a quaternion (`mint::Quaternion<f32>`) reading from the BNO055.
    /// Available only in sensor fusion modes.
    pub fn quaternion(&mut self) -> Result<mint::Quaternion<f32>, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        // Device should be in fusion mode to be able to produce quaternions
        if self.mode.is_fusion_enabled() {
            let mut buf: [u8; 8] = [0; 8];
            self.read_bytes(regs::BNO055_QUA_DATA_W_LSB, &mut buf)
                .map_err(Error::I2c)?;

            let w = LittleEndian::read_i16(&buf[0..2]);
            let x = LittleEndian::read_i16(&buf[2..4]);
            let y = LittleEndian::read_i16(&buf[4..6]);
            let z = LittleEndian::read_i16(&buf[6..8]);

            let scale = 1.0 / ((1 << 14) as f32);

            let x = x as f32 * scale;
            let y = y as f32 * scale;
            let z = z as f32 * scale;
            let w = w as f32 * scale;

            let quat = mint::Quaternion {
                v: mint::Vector3 { x, y, z },
                s: w,
            };

            Ok(quat)
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Get Euler angles representation of heading in degrees.
    /// Euler angles is represented as (`roll`, `pitch`, `yaw/heading`).
    /// Available only in sensor fusion modes.
    pub fn euler_angles(&mut self) -> Result<mint::EulerAngles<f32, ()>, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        // Device should be in fusion mode to be able to produce Euler angles
        if self.mode.is_fusion_enabled() {
            let mut buf: [u8; 6] = [0; 6];

            self.read_bytes(regs::BNO055_EUL_HEADING_LSB, &mut buf)
                .map_err(Error::I2c)?;

            let heading = LittleEndian::read_i16(&buf[0..2]) as f32;
            let roll = LittleEndian::read_i16(&buf[2..4]) as f32;
            let pitch = LittleEndian::read_i16(&buf[4..6]) as f32;

            let scale = 1f32 / 16f32; // 1 degree = 16 LSB

            let rot = mint::EulerAngles::from([roll * scale, pitch * scale, heading * scale]);

            Ok(rot)
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Get calibration status
    pub fn get_calibration_status(&mut self) -> Result<BNO055CalibrationStatus, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        let status = self.read_u8(regs::BNO055_CALIB_STAT).map_err(Error::I2c)?;

        let sys = (status >> 6) & 0b11;
        let gyr = (status >> 4) & 0b11;
        let acc = (status >> 2) & 0b11;
        let mag = status & 0b11;

        Ok(BNO055CalibrationStatus { sys, gyr, acc, mag })
    }

    /// Checks whether device is fully calibrated or not.
    pub fn is_fully_calibrated(&mut self) -> Result<bool, Error<E>> {
        let status = self.get_calibration_status()?;
        Ok(status.mag == 3 && status.gyr == 3 && status.acc == 3 && status.sys == 3)
    }

    /// Reads current calibration profile of the device.
    pub fn calibration_profile(
        &mut self,
        delay: &mut dyn DelayNs,
    ) -> Result<BNO055Calibration, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        let prev_mode = self.mode;
        self.set_mode(BNO055OperationMode::CONFIG_MODE, delay)?;

        let mut buf: [u8; BNO055_CALIB_SIZE] = [0; BNO055_CALIB_SIZE];

        self.read_bytes(regs::BNO055_ACC_OFFSET_X_LSB, &mut buf[..])
            .map_err(Error::I2c)?;

        let res = BNO055Calibration::from_buf(&buf);

        self.set_mode(prev_mode, delay)?;

        Ok(res)
    }

    /// Sets current calibration profile.
    pub fn set_calibration_profile(
        &mut self,
        calib: BNO055Calibration,
        delay: &mut dyn DelayNs,
    ) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        let prev_mode = self.mode;
        self.set_mode(BNO055OperationMode::CONFIG_MODE, delay)?;

        let buf_profile = calib.as_bytes();

        // Combine register address and profile into single buffer
        let buf_reg = [regs::BNO055_ACC_OFFSET_X_LSB; 1];
        let mut buf_with_reg = [0u8; 1 + BNO055_CALIB_SIZE];
        for (to, from) in buf_with_reg
            .iter_mut()
            .zip(buf_reg.iter().chain(buf_profile.iter()))
        {
            *to = *from
        }

        self.i2c
            .write(self.i2c_addr(), &buf_with_reg[..])
            .map_err(Error::I2c)?;

        // change operation mode to fusion mode
        self.set_mode(prev_mode, delay)?;

        Ok(())
    }

    /// Returns device's factory-programmed and constant chip ID.
    /// This ID is device model ID and not a BNO055's unique ID, whic is stored in different register.
    pub fn id(&mut self) -> Result<u8, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;
        self.read_u8(regs::BNO055_CHIP_ID).map_err(Error::I2c)
    }

    /// Returns device's operation mode.
    pub fn get_mode(&mut self) -> Result<BNO055OperationMode, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        let mode = self.read_u8(regs::BNO055_OPR_MODE).map_err(Error::I2c)?;
        let mode = BNO055OperationMode::from_bits_truncate(mode);
        self.mode = mode;

        Ok(mode)
    }

    /// Checks whether the device is in Sensor Fusion mode or not.
    pub fn is_in_fusion_mode(&mut self) -> Result<bool, Error<E>> {
        Ok(self.mode.is_fusion_enabled())
    }

    pub fn get_acc_config(&mut self) -> Result<AccConfig, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_1)?;

        let bits = self.read_u8(regs::BNO055_ACC_CONFIG).map_err(Error::I2c)?;

        let acc_config = AccConfig::try_from_bits(bits).map_err(Error::AccConfig)?;

        Ok(acc_config)
    }

    pub fn set_acc_config(&mut self, acc_config: &AccConfig) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_1)?;

        self.write_u8(regs::BNO055_ACC_CONFIG, acc_config.bits())
            .map_err(Error::I2c)?;

        Ok(())
    }

    /// Sets current register map page.
    fn set_page(&mut self, page: BNO055RegisterPage) -> Result<(), Error<E>> {
        self.write_u8(regs::BNO055_PAGE_ID, page.bits())
            .map_err(Error::I2c)?;

        Ok(())
    }

    /// Reads a vector of sensor data from the device.
    fn read_vec_raw(&mut self, reg: u8) -> Result<mint::Vector3<i16>, Error<E>> {
        let mut buf: [u8; 6] = [0; 6];

        self.read_bytes(reg, &mut buf).map_err(Error::I2c)?;

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

    /// Returns linear acceleration vector in cm/s^2 units.
    /// Available only in sensor fusion modes.
    pub fn linear_acceleration_fixed(&mut self) -> Result<mint::Vector3<i16>, Error<E>> {
        if self.mode.is_fusion_enabled() {
            self.set_page(BNO055RegisterPage::PAGE_0)?;
            self.read_vec_raw(regs::BNO055_LIA_DATA_X_LSB)
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Returns linear acceleration vector in m/s^2 units.
    /// Available only in sensor fusion modes.
    pub fn linear_acceleration(&mut self) -> Result<mint::Vector3<f32>, Error<E>> {
        let linear_acceleration = self.linear_acceleration_fixed()?;
        let scaling = 1f32 / 100f32; // 1 m/s^2 = 100 lsb
        Ok(Self::scale_vec(linear_acceleration, scaling))
    }

    /// Returns gravity vector in cm/s^2 units.
    /// Available only in sensor fusion modes.
    pub fn gravity_fixed(&mut self) -> Result<mint::Vector3<i16>, Error<E>> {
        if self.mode.is_fusion_enabled() {
            self.set_page(BNO055RegisterPage::PAGE_0)?;
            self.read_vec_raw(regs::BNO055_GRV_DATA_X_LSB)
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Returns gravity vector in m/s^2 units.
    /// Available only in sensor fusion modes.
    pub fn gravity(&mut self) -> Result<mint::Vector3<f32>, Error<E>> {
        let gravity = self.gravity_fixed()?;
        let scaling = 1f32 / 100f32; // 1 m/s^2 = 100 lsb
        Ok(Self::scale_vec(gravity, scaling))
    }

    /// Returns Acceleration and Gyroscope vectors in this order.
    ///
    /// Available only in modes in which accelerometer and gyroscope are enabled.
    pub fn dof6_fixed(&mut self) -> Result<(mint::Vector3<i16>, mint::Vector3<i16>), Error<E>> {
        if self.mode.is_accel_enabled() && self.mode.is_gyro_enabled() {
            self.set_page(BNO055RegisterPage::PAGE_0)?;

            // start the read from ACC_DATA_X (0x08) registry to GYR_DATA_Z_MSB 0x19
            // from 8th to 13th incl. reg = 12 bytes
            let reg = regs::BNO055_ACC_DATA_X_LSB;

            let mut buf: [u8; 12] = [0; 12];

            self.read_bytes(reg, &mut buf).map_err(Error::I2c)?;

            let accel = {
                let x = LittleEndian::read_i16(&buf[0..2]);
                let y = LittleEndian::read_i16(&buf[2..4]);
                let z = LittleEndian::read_i16(&buf[4..6]);

                mint::Vector3::from([x, y, z])
            };
            let gyro = {
                let x = LittleEndian::read_i16(&buf[6..8]);
                let y = LittleEndian::read_i16(&buf[8..10]);
                let z = LittleEndian::read_i16(&buf[10..12]);

                mint::Vector3::from([x, y, z])
            };

            Ok((accel, gyro))
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Returns Acceleration and Gyroscope vectors in this order.
    ///
    /// Available only in modes in which accelerometer and gyroscope are enabled.
    pub fn dof6(&mut self) -> Result<(mint::Vector3<f32>, mint::Vector3<f32>), Error<E>> {
        let (accel, gyro) = self.dof6_fixed()?;

        Ok((
            Self::scale_vec(accel, ACCEL_SCALING),
            Self::scale_vec(gyro, GYRO_SCALING),
        ))
    }

    /// Returns Acceleration, Gyroscope and Magnetometer vectors in this order.
    ///
    /// Available only in modes in which accelerometer, gyroscope and magnetometer are enabled.
    pub fn dof9_fixed(
        &mut self,
    ) -> Result<(mint::Vector3<i16>, mint::Vector3<i16>, mint::Vector3<i16>), Error<E>> {
        if self.mode.is_accel_enabled() && self.mode.is_gyro_enabled() && self.mode.is_mag_enabled()
        {
            self.set_page(BNO055RegisterPage::PAGE_0)?;

            // start the read from ACC_DATA_X (0x08) registry to GYR_DATA_Z_MSB 0x19
            // from 8th to 25th incl. reg = 18 bytes
            // self.read_vec_raw(regs::BNO055_ACC_DATA_X_LSB)
            let reg = regs::BNO055_ACC_DATA_X_LSB;

            let mut buf: [u8; 18] = [0; 18];

            self.read_bytes(reg, &mut buf).map_err(Error::I2c)?;

            let accel = {
                let x = LittleEndian::read_i16(&buf[0..2]);
                let y = LittleEndian::read_i16(&buf[2..4]);
                let z = LittleEndian::read_i16(&buf[4..6]);

                mint::Vector3::from([x, y, z])
            };
            let gyro = {
                let x = LittleEndian::read_i16(&buf[6..8]);
                let y = LittleEndian::read_i16(&buf[8..10]);
                let z = LittleEndian::read_i16(&buf[10..12]);

                mint::Vector3::from([x, y, z])
            };
            let mag = {
                let x = LittleEndian::read_i16(&buf[12..14]);
                let y = LittleEndian::read_i16(&buf[14..16]);
                let z = LittleEndian::read_i16(&buf[16..18]);

                mint::Vector3::from([x, y, z])
            };

            Ok((accel, gyro, mag))
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Returns Acceleration, Gyroscope and Magnetometer vectors in this order.
    ///
    /// Available only in modes in which accelerometer, gyroscope and magnetometer are enabled.
    pub fn dof9(
        &mut self,
    ) -> Result<(mint::Vector3<f32>, mint::Vector3<f32>, mint::Vector3<f32>), Error<E>> {
        let (accel, gyro, mag) = self.dof9_fixed()?;
        Ok((
            Self::scale_vec(accel, ACCEL_SCALING),
            Self::scale_vec(gyro, GYRO_SCALING),
            Self::scale_vec(mag, MAG_SCALING),
        ))
    }

    /// Returns current accelerometer data in cm/s^2 units.
    /// Available only in modes in which accelerometer is enabled.
    pub fn accel_data_fixed(&mut self) -> Result<mint::Vector3<i16>, Error<E>> {
        if self.mode.is_accel_enabled() {
            self.set_page(BNO055RegisterPage::PAGE_0)?;
            self.read_vec_raw(regs::BNO055_ACC_DATA_X_LSB)
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Returns current accelerometer data in m/s^2 units.
    /// Available only in modes in which accelerometer is enabled.
    pub fn accel_data(&mut self) -> Result<mint::Vector3<f32>, Error<E>> {
        let a = self.accel_data_fixed()?;
        // let scaling = 1f32 / 100f32; // 1 m/s^2 = 100 lsb
        Ok(Self::scale_vec(a, ACCEL_SCALING))
    }

    /// Returns current gyroscope data in 1/16th deg/s units.
    /// Available only in modes in which gyroscope is enabled.
    pub fn gyro_data_fixed(&mut self) -> Result<mint::Vector3<i16>, Error<E>> {
        if self.mode.is_gyro_enabled() {
            self.set_page(BNO055RegisterPage::PAGE_0)?;
            self.read_vec_raw(regs::BNO055_GYR_DATA_X_LSB)
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Returns current gyroscope data in deg/s units.
    /// Available only in modes in which gyroscope is enabled.
    pub fn gyro_data(&mut self) -> Result<mint::Vector3<f32>, Error<E>> {
        let g = self.gyro_data_fixed()?;
        // let scaling = 1f32 / 16f32; // 1 deg/s = 16 lsb
        Ok(Self::scale_vec(g, GYRO_SCALING))
    }

    /// Returns current magnetometer data in 1/16th uT units.
    /// Available only in modes in which magnetometer is enabled.
    pub fn mag_data_fixed(&mut self) -> Result<mint::Vector3<i16>, Error<E>> {
        if self.mode.is_mag_enabled() {
            self.set_page(BNO055RegisterPage::PAGE_0)?;
            self.read_vec_raw(regs::BNO055_MAG_DATA_X_LSB)
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Returns current magnetometer data in uT units.
    /// Available only in modes in which magnetometer is enabled.
    pub fn mag_data(&mut self) -> Result<mint::Vector3<f32>, Error<E>> {
        let m = self.mag_data_fixed()?;
        // let scaling = 1f32 / 16f32; // 1 uT = 16 lsb
        Ok(Self::scale_vec(m, MAG_SCALING))
    }

    /// Returns current temperature of the chip (in degrees Celsius).
    ///
    /// By default this uses the Accelerometer temperature.
    pub fn temperature(&mut self) -> Result<i8, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;

        // Read temperature signed byte
        let temp = self.read_u8(regs::BNO055_TEMP).map_err(Error::I2c)? as i8;
        Ok(temp)
    }

    /// Read which interrupts are currently triggered/active
    pub fn interrupts_triggered(&mut self) -> Result<BNO055Interrupt, Error<E>> {
        read_flags!(
            self,
            BNO055RegisterPage::PAGE_0,
            regs::BNO055_INT_STA,
            BNO055Interrupt
        )
    }

    /// Resets the interrupts register and the INT pin
    pub fn clear_interrupts(&mut self) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0)?;
        self.write_u8(
            regs::BNO055_SYS_TRIGGER,
            BNO055SystemTrigger::RST_INT.bits(),
        )
        .map_err(Error::I2c)?;
        Ok(())
    }

    /// Sets which interrupts are enabled
    ///
    /// One of the only config options that dont need to be in config mode to write to
    pub fn set_interrupts_enabled(&mut self, interrupts: BNO055Interrupt) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_1)?;
        self.write_u8(regs::BNO055_INT_EN, interrupts.bits())
            .map_err(Error::I2c)
    }

    /// Returns currently enabled interrupts
    pub fn interrupts_enabled(&mut self) -> Result<BNO055Interrupt, Error<E>> {
        // self.set_page(BNO055RegisterPage::PAGE_0)?;

        // let value = self
        //     .read_u8(regs::BNO055_AXIS_MAP_CONFIG)
        //     .map_err(Error::I2c)?;

        // let remap = AxisRemap {
        //     x: BNO055AxisConfig::from_bits_truncate(value & 0b11),
        //     y: BNO055AxisConfig::from_bits_truncate((value >> 2) & 0b11),
        //     z: BNO055AxisConfig::from_bits_truncate((value >> 4) & 0b11),
        // };

        // Ok(remap)

        read_flags!(
            self,
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_INT_EN,
            BNO055Interrupt
        )
    }

    /// Sets interrupts mask
    ///
    /// Official doc: when mask=1, the interrupt will update INT_STA register and trigger a change in INT pin,
    /// When mask=0, only INT_STA register will be updated
    /// One of the only config options that dont need to be in config mode to write to
    pub fn set_interrupts_mask(&mut self, mask: BNO055Interrupt) -> Result<(), Error<E>> {
        // self.set_page(BNO055RegisterPage::PAGE_1)?;
        // self.write_u8(
        //     regs::BNO055_INT_MSK,
        //     mask.bits(),
        // )
        // .map_err(Error::I2c)
        write_flags!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_INT_MSK, mask)
    }

    /// Returns the current interrupts mask
    pub fn interrupts_mask(&mut self) -> Result<BNO055Interrupt, Error<E>> {
        read_flags!(
            self,
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_INT_MSK,
            BNO055Interrupt
        )
    }

    /// Returns current accelerometer config settings
    // pub fn acc_config(&mut self) -> Result<AccConfig, Error<E>> {
    //     read_u8_into!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_ACC_CONFIG)
    // }

    /// Sets accelerometer config settings
    // pub fn set_acc_config(
    //     &mut self,
    //     cfg: AccConfig,
    //     delay: &mut dyn DelayNs,
    // ) -> Result<(), Error<E>> {
    //     set_config_from!(
    //         self,
    //         BNO055RegisterPage::PAGE_1,
    //         regs::BNO055_ACC_CONFIG,
    //         cfg,
    //         delay
    //     )
    // }

    /// Sets accelerometer interrupt settings
    pub fn set_acc_interrupt_settings(
        &mut self,
        settings: BNO055AccIntSettings,
        delay: &mut dyn DelayNs,
    ) -> Result<(), Error<E>> {
        set_config_from!(
            self,
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_ACC_INT_SETTING,
            settings,
            delay
        )
    }

    /// Returns current accelerometer interrupt settings
    pub fn acc_interrupt_settings(&mut self) -> Result<BNO055AccIntSettings, Error<E>> {
        read_u8_into!(
            self,
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_ACC_INT_SETTING
        )
        .ok_or_else(|| Error::AccConfig(acc_config::Error::BadAccIntSettings))
    }

    /// Sets accelerometer any motion interrupt threshold setting
    ///
    /// Actual value is `mult` * base-unit based on accelerometer range set in ACC_CONFIG
    pub fn set_acc_am_threshold(
        &mut self,
        mult: u8,
        delay: &mut dyn DelayNs,
    ) -> Result<(), Error<E>> {
        set_config_from!(
            self,
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_ACC_AM_THRES,
            mult,
            delay
        )
    }

    /// Returns current accelerometer any motion interrupt threshold setting
    ///
    /// Actual value is `mult` * base-unit based on accelerometer range set in ACC_CONFIG
    pub fn acc_am_threshold(&mut self) -> Result<u8, Error<E>> {
        read_u8_into!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_ACC_AM_THRES)
            .ok_or_else(|| Error::AccConfig(acc_config::Error::BadAccAmThreshold))
    }

    /// Sets accelerometer High-G interrupt duration setting
    pub fn set_acc_hg_duration(
        &mut self,
        dur: u8,
        delay: &mut dyn DelayNs,
    ) -> Result<(), Error<E>> {
        set_config_from!(
            self,
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_ACC_HG_DURATION,
            dur,
            delay
        )
    }

    /// Returns current accelerometer High-G interrupt duration setting
    pub fn acc_hg_duration(&mut self) -> Result<u8, Error<E>> {
        read_u8_into!(
            self,
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_ACC_HG_DURATION
        )
        .ok_or_else(|| Error::AccConfig(acc_config::Error::BadAccHgDuration))
    }

    /// Sets accelerometer High-G interrupt threshold setting
    ///
    /// Actual value is `mult` * base-unit based on accelerometer range set in ACC_CONFIG
    pub fn set_acc_hg_threshold(
        &mut self,
        mult: u8,
        delay: &mut dyn DelayNs,
    ) -> Result<(), Error<E>> {
        set_config_from!(
            self,
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_ACC_HG_THRES,
            mult,
            delay
        )
    }

    /// Returns current accelerometer High-G interrupt threshold setting
    ///
    /// Actual value is `mult` * base-unit based on accelerometer range set in ACC_CONFIG
    pub fn acc_hg_threshold(&mut self) -> Result<u8, Error<E>> {
        read_u8_into!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_ACC_HG_THRES)
            .ok_or_else(|| Error::AccConfig(acc_config::Error::BadAccHgThreshold))
    }

    /// Sets accelerometer no/slow-motion interrupt threshold setting
    ///
    /// Actual value is `mult` * base-unit based on accelerometer range set in ACC_CONFIG
    pub fn set_acc_nm_threshold(
        &mut self,
        mult: u8,
        delay: &mut dyn DelayNs,
    ) -> Result<(), Error<E>> {
        set_config_from!(
            self,
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_ACC_NM_THRES,
            mult,
            delay
        )
    }

    /// Returns current accelerometer no/slow-motion interrupt threshold setting
    ///
    /// Actual value is `mult` * base-unit based on accelerometer range set in ACC_CONFIG
    pub fn acc_nm_threshold(&mut self) -> Result<u8, Error<E>> {
        read_u8_into!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_ACC_NM_THRES)
            .ok_or_else(|| Error::AccConfig(acc_config::Error::BadAccNmThreshold))
    }

    /// Sets accelerometer no/slow-motion interrupt settings
    pub fn set_acc_nm_settings(
        &mut self,
        settings: BNO055AccNmSettings,
        delay: &mut dyn DelayNs,
    ) -> Result<(), Error<E>> {
        set_config_from!(
            self,
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_ACC_NM_SET,
            settings,
            delay
        )
    }

    /// Returns current accelerometer no/slow-motion interrupt settings
    pub fn acc_nm_settings(&mut self) -> Result<BNO055AccNmSettings, Error<E>> {
        read_u8_into!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_ACC_NM_SET)
            .ok_or_else(|| Error::AccConfig(acc_config::Error::BadAccNmSettings))
    }

    /// Sets gyroscope interrupt settings
    pub fn set_gyr_interrupt_settings(
        &mut self,
        settings: BNO055GyrIntSettings,
        delay: &mut dyn DelayNs,
    ) -> Result<(), Error<E>> {
        write_config_flags!(
            self,
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_GYR_INT_SETTING,
            settings,
            delay
        )
    }

    /// Returns the current gyroscope interrupt settings
    pub fn gyr_interrupt_settings(&mut self) -> Result<BNO055GyrIntSettings, Error<E>> {
        read_flags!(
            self,
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_GYR_INT_SETTING,
            BNO055GyrIntSettings
        )
    }

    /// Sets gyroscope high-rate interrupt settings for x-axis
    pub fn set_gyr_hr_x_settings(
        &mut self,
        settings: BNO055GyrHrSettings,
        delay: &mut dyn DelayNs,
    ) -> Result<(), Error<E>> {
        set_config_from!(
            self,
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_GYR_HR_X_SET,
            settings,
            delay
        )
    }

    /// Returns current gyroscope high-rate interrupt settings for x-axis
    pub fn gyr_hr_x_settings(&mut self) -> Result<BNO055GyrHrSettings, Error<E>> {
        read_u8_into!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_HR_X_SET)
            .ok_or_else(|| Error::GyroConfig(gyr_config::Error::BadGyrHrSettings))
    }

    /// Sets gyroscope high-rate interrupt duration for x-axis
    ///
    /// Actual duration is (`duration` + 1) * 2.5ms
    pub fn set_gyr_dur_x(&mut self, duration: u8, delay: &mut dyn DelayNs) -> Result<(), Error<E>> {
        set_config_from!(
            self,
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_GYR_DUR_X,
            duration,
            delay
        )
    }

    /// Returns current gyroscope high-rate interrupt settings for x-axis
    ///
    /// Actual duration is (result + 1) * 2.5ms
    pub fn gyr_dur_x(&mut self) -> Result<u8, Error<E>> {
        read_u8_into!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_DUR_X)
            .ok_or_else(|| Error::GyroConfig(gyr_config::Error::BadGyrDurX))
    }

    /// Sets gyroscope high-rate interrupt settings for y-axis
    pub fn set_gyr_hr_y_settings(
        &mut self,
        settings: BNO055GyrHrSettings,
        delay: &mut dyn DelayNs,
    ) -> Result<(), Error<E>> {
        set_config_from!(
            self,
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_GYR_HR_Y_SET,
            settings,
            delay
        )
    }

    /// Returns current gyroscope high-rate interrupt settings for y-axis
    pub fn gyr_hr_y_settings(&mut self) -> Result<BNO055GyrHrSettings, Error<E>> {
        read_u8_into!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_HR_Y_SET)
            .ok_or_else(|| Error::GyroConfig(gyr_config::Error::BadGyrHrYSettings))
    }

    /// Sets gyroscope high-rate interrupt duration for y-axis
    ///
    /// Actual duration is (`duration` + 1) * 2.5ms
    pub fn set_gyr_dur_y(&mut self, duration: u8, delay: &mut dyn DelayNs) -> Result<(), Error<E>> {
        set_config_from!(
            self,
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_GYR_DUR_Y,
            duration,
            delay
        )
    }

    /// Returns current gyroscope high-rate interrupt settings for y-axis
    ///
    /// Actual duration is (result + 1) * 2.5ms
    pub fn gyr_dur_y(&mut self) -> Result<u8, Error<E>> {
        read_u8_into!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_DUR_Y)
            .ok_or_else(|| Error::GyroConfig(gyr_config::Error::BadGyrDurY))
    }

    /// Sets gyroscope high-rate interrupt settings for z-axis
    pub fn set_gyr_hr_z_settings(
        &mut self,
        settings: BNO055GyrHrSettings,
        delay: &mut dyn DelayNs,
    ) -> Result<(), Error<E>> {
        set_config_from!(
            self,
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_GYR_HR_Z_SET,
            settings,
            delay
        )
    }

    /// Returns current gyroscope high-rate interrupt settings for z-axis
    pub fn gyr_hr_z_settings(&mut self) -> Result<BNO055GyrHrSettings, Error<E>> {
        read_u8_into!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_HR_Z_SET)
            .ok_or_else(|| Error::GyroConfig(gyr_config::Error::BadGyrHrZSettings))
    }

    /// Sets gyroscope high-rate interrupt duration for z-axis
    /// Actual duration is (`duration` + 1) * 2.5ms
    pub fn set_gyr_dur_z(&mut self, duration: u8, delay: &mut dyn DelayNs) -> Result<(), Error<E>> {
        set_config_from!(
            self,
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_GYR_DUR_Z,
            duration,
            delay
        )
    }

    /// Returns current gyroscope high-rate interrupt settings for z-axis
    /// Actual duration is (result + 1) * 2.5ms
    pub fn gyr_dur_z(&mut self) -> Result<u8, Error<E>> {
        read_u8_into!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_DUR_Z)
            .ok_or_else(|| Error::GyroConfig(gyr_config::Error::BadGyrDurZ))
    }

    /// Sets gyroscope any-motion interrupt threshold
    /// Actual value is `mult` * base-unit based on gyroscope range set in GYR_CONFIG_0
    pub fn set_gyr_am_threshold(
        &mut self,
        mult: u8,
        delay: &mut dyn DelayNs,
    ) -> Result<(), Error<E>> {
        let mut mult = mult;
        if mult > 0b01111111 {
            mult = 0b01111111;
        }
        set_config_from!(
            self,
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_GYR_DUR_Z,
            mult & BIT_7_RESERVED_MASK,
            delay
        )
    }

    /// Returns current gyroscope high-rate interrupt settings for z-axis
    /// Actual value is `mult` * base-unit based on gyroscope range set in GYR_CONFIG_0
    pub fn gyr_am_threshold(&mut self) -> Result<u8, Error<E>> {
        let res: Result<u8, Error<E>> =
            read_u8_into!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_DUR_Z)
                .ok_or_else(|| Error::GyroConfig(gyr_config::Error::BadGyrAmThreshold));
        res.map(|mult| BIT_7_RESERVED_MASK & mult)
    }

    /// Sets gyroscope any-motion interrupt settings
    pub fn set_gyr_am_settings(
        &mut self,
        settings: BNO055GyrAmSettings,
        delay: &mut dyn DelayNs,
    ) -> Result<(), Error<E>> {
        set_config_from!(
            self,
            BNO055RegisterPage::PAGE_1,
            regs::BNO055_GYR_AM_SET,
            settings,
            delay
        )
    }

    /// Returns current gyroscope any-motion interrupt settings
    pub fn gyr_am_settings(&mut self) -> Result<BNO055GyrAmSettings, Error<E>> {
        read_u8_into!(self, BNO055RegisterPage::PAGE_1, regs::BNO055_GYR_AM_SET)
            .ok_or_else(|| Error::GyroConfig(gyr_config::Error::BadGyrAmSettings))
    }

    #[inline(always)]
    fn i2c_addr(&self) -> u8 {
        if !self.use_default_addr {
            regs::BNO055_ALTERNATE_ADDR
        } else {
            regs::BNO055_DEFAULT_ADDR
        }
    }

    fn read_u8(&mut self, reg: u8) -> Result<u8, E> {
        let mut byte: [u8; 1] = [0; 1];

        match self.i2c.write_read(self.i2c_addr(), &[reg], &mut byte) {
            Ok(_) => Ok(byte[0]),
            Err(e) => Err(e),
        }
    }

    fn read_bytes(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), E> {
        self.i2c.write_read(self.i2c_addr(), &[reg], buf)
    }

    fn write_u8(&mut self, reg: u8, value: u8) -> Result<(), E> {
        self.i2c.write(self.i2c_addr(), &[reg, value])?;

        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct BNO055AxisConfig(u8);

bitflags! {
    /// BNO055 accelerometer interrupt settings
    impl BNO055AxisConfig: u8 {
        const AXIS_AS_X = 0b00;
        const AXIS_AS_Y = 0b01;
        const AXIS_AS_Z = 0b10;
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct BNO055AxisSign(u8);

bitflags! {
    impl BNO055AxisSign: u8 {
        const X_NEGATIVE = 0b100;
        const Y_NEGATIVE = 0b010;
        const Z_NEGATIVE = 0b001;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct BNO055SystemStatusCode(u8);

bitflags! {
    impl BNO055SystemStatusCode: u8 {
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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct BNO055SystemErrorCode(u8);

bitflags! {
    impl BNO055SystemErrorCode: u8 {
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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct BNO055SelfTestStatus(u8);

bitflags! {
    impl BNO055SelfTestStatus: u8 {
        const ACC_OK = 0b0001;
        const MAG_OK = 0b0010;
        const GYR_OK = 0b0100;
        const SYS_OK = 0b1000;
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
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct BNO055Revision {
    /// # Example
    ///
    /// For version 3.17
    ///
    /// ```rust
    /// let version_reg: [u8; 2] = [3, 17];
    /// assert_eq!(785_u16, u16::from_be_bytes(version_reg));
    /// ```
    pub software: u16,
    pub bootloader: u8,
    pub accelerometer: u8,
    pub magnetometer: u8,
    pub gyroscope: u8,
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
        unsafe {
            core::slice::from_raw_parts(
                (self as *const _) as *const u8,
                ::core::mem::size_of::<BNO055Calibration>(),
            )
        }
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct BNO055RegisterPage(u8);

bitflags! {
    impl  BNO055RegisterPage: u8 {
        const PAGE_0 = 0;
        const PAGE_1 = 1;
    }
}
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct BNO055PowerMode(u8);

bitflags! {
    impl BNO055PowerMode: u8 {
        const NORMAL = 0b00;
        const LOW_POWER = 0b01;
        const SUSPEND = 0b10;
    }
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct BNO055OperationMode(u8);

bitflags! {
    /// Possible BNO055 operation modes.
    // #[cfg_attr(not(feature = "defmt-03"), derive(Debug, Clone, Copy, PartialEq, Eq))]
    impl BNO055OperationMode: u8 {
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
        /// 3.3.3.5 NDOF
        ///
        /// This is a fusion mode with 9 degrees of freedom where the fused absolute orientation data is
        /// calculated from accelerometer, gyroscope and the magnetometer. The advantages of
        /// combining all three sensors are a fast calculation, resulting in high output data rate, and high
        /// robustness from magnetic field distortions. In this mode the Fast Magnetometer calibration is
        /// turned ON and thereby resulting in quick calibration of the magnetometer and higher output
        /// data accuracy. The current consumption is slightly higher in comparison to the
        /// NDOF_FMC_OFF fusion mode.
        const NDOF = 0b1100;
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

#[derive(num_derive::FromPrimitive, Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[repr(transparent)]
pub struct BNO055Interrupt(pub u8);

bitflags! {
    /// BNO055 interrupt enable/mask flags.
    impl BNO055Interrupt: u8 {
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

/// `SYS_TRIGGER` ([`regs::BNO055_SYS_TRIGGER`]) register values.
#[derive(num_derive::FromPrimitive, Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[repr(transparent)]
pub struct BNO055SystemTrigger(pub u8);

bitflags! {
    impl BNO055SystemTrigger: u8 {
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

        assert!(BNO055Interrupt(0b10011).contains(BNO055Interrupt::ACC_BSX_DRDY));
        assert!(BNO055Interrupt(0b10011).contains(BNO055Interrupt::GYR_DRDY));
        assert!(BNO055Interrupt(0b10011).contains(BNO055Interrupt::MAG_DRDY));
    }
}
