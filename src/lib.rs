//! An [embedded-hal] driver for the ICM-40627 6-axis IMU from InvenSense.
//!
//! The ICM-40627 combines a 3-axis accelerometer with a 3-axis gyroscope into a
//! single package. It has a configurable host interface which supports I²C,
//! and SPI communications. Presently this driver only supports using the
//! I²C interface.
//!
//! [embedded-hal]: https://docs.rs/embedded-hal/latest/embedded_hal/

#![no_std]
#![feature(const_index, const_trait_impl)]

use core::fmt::Debug;

use embedded_hal_async::i2c::I2c;

pub use crate::{
    config::{
        AccLpAvg,
        AccelDlpfBw,
        AccelOdr,
        AccelRange,
        Address,
        GyroLpFiltBw,
        GyroOdr,
        GyroRange,
        PowerMode,
        TempDlpfBw,
    },
    error::Error,
};
use crate::{
    config::{Bitfield, SoftReset},
    error::SensorError,
    register::{Bank0, Register},
};

mod config;
mod error;
mod register;

/// Device driver supporting ICM 42627 and 40627,
///  which seem to have the same spec.
#[derive(Debug, Clone, Copy)]
pub struct Icm42627<I2C> {
    /// Underlying I²C peripheral
    i2c: I2C,
    /// I²C slave address to use
    address: Address,
}

impl<I2C, E> Icm42627<I2C>
where
    I2C: I2c<Error = E>,
    E: Debug,
{
    /// WHO_AM_I values for 42627 and 40627
    pub const DEVICE_IDS: [u8; 2] = [0x20, 0x4E];

    /// Instantiate a new instance of the driver and initialize the device
    pub async fn new(i2c: I2C, address: Address) -> Result<Self, Error<E>> {
        let mut new = Self { i2c, address };

        // Verify that the device has the correct ID before continuing. If the ID does
        // not match either of the expected values then it is likely the wrong chip is
        // connected.
        if !Self::DEVICE_IDS.contains(&new.device_id().await?) {
            return Err(Error::SensorError(SensorError::BadChip));
        }

        // Make sure that any configuration has been restored to the default values when
        // initializing the driver.
        new.set_accel_range(AccelRange::default()).await?;
        new.set_gyro_range(GyroRange::default()).await?;

        // The IMU uses `PowerMode::Sleep` by default, which disables both the accel and
        // gyro, so we enable them both during driver initialization.
        new.set_power_mode(PowerMode::SixAxisLowNoise).await?;

        new.write_reg(&Bank0::SELF_TEST_CONFIG, 0x07).await?;

        Ok(new)
    }

    /// Return the raw interface to the underlying `I2C` instance
    pub fn destroy(self) -> I2C {
        self.i2c
    }

    /// Read the ID of the connected device
    pub async fn device_id(&mut self) -> Result<u8, Error<E>> {
        self.read_reg(&Bank0::WHO_AM_I).await
    }

    /// Perform a software-reset on the device
    pub async fn soft_reset(&mut self) -> Result<(), Error<E>> {
        self.update_reg(SoftReset).await
    }

    /// Enable the given self test config bits
    ///
    /// TODO: needs an actual config API
    pub async fn enable_self_test(&mut self, bits: u8) -> Result<(), Error<E>> {
        self.write_reg(&Bank0::SELF_TEST_CONFIG, bits).await
    }

    /// Return the normalized gyro data for each of the three axes
    pub async fn gyro_norm(&mut self) -> Result<F32x3, Error<E>> {
        let range = self.gyro_range().await?;
        let scale = range.scale_factor();

        // Scale the raw Gyroscope data using the appropriate factor based on the
        // configured range.
        let raw = self.gyro_raw().await?;
        let x = raw[Axis::X] as f32 / scale;
        let y = raw[Axis::Y] as f32 / scale;
        let z = raw[Axis::Z] as f32 / scale;

        Ok([x, y, z])
    }

    /// Read the raw gyro data for each of the three axes
    pub async fn gyro_raw(&mut self) -> Result<I16x3, Error<E>> {
        let (x, y, z) = self.read_reg_i16_triplet(&Bank0::GYRO_DATA_X1).await?;

        Ok([x, y, z])
    }

    /// Read the built-in temperature sensor and return the value in degrees
    /// centigrade
    pub async fn temperature(&mut self) -> Result<f32, Error<E>> {
        let raw = self.temperature_raw().await? as f32;
        let deg = (raw / 128.0) + 25.0;

        Ok(deg)
    }

    /// Read the raw data from the built-in temperature sensor
    pub async fn temperature_raw(&mut self) -> Result<i16, Error<E>> {
        self.read_reg_i16(&Bank0::TEMP_DATA1).await
    }

    /// Read all of the raw sensor data (accelerometer, gyro, temperature) at
    /// once.
    ///
    /// This is faster than reading the sensors individually
    pub async fn measure_raw(&mut self) -> Result<(I16x3, I16x3, i16), Error<E>> {
        let mut bytes = [0u8; 6 + 6 + 2]; // Accel + gyro + temp
        self.i2c
            .write_read(self.address as u8, &[Bank0::TEMP_DATA1.addr()], &mut bytes)
            .await
            .map_err(Error::BusError)?;

        let temp_raw = i16::from_be_bytes([bytes[0], bytes[1]]);
        let accel_raw_x = i16::from_be_bytes([bytes[2], bytes[3]]);
        let accel_raw_y = i16::from_be_bytes([bytes[4], bytes[5]]);
        let accel_raw_z = i16::from_be_bytes([bytes[6], bytes[7]]);
        let gyro_raw_x = i16::from_be_bytes([bytes[8], bytes[9]]);
        let gyro_raw_y = i16::from_be_bytes([bytes[10], bytes[11]]);
        let gyro_raw_z = i16::from_be_bytes([bytes[12], bytes[13]]);

        Ok((
            [accel_raw_x, accel_raw_y, accel_raw_z],
            [gyro_raw_x, gyro_raw_y, gyro_raw_z],
            temp_raw,
        ))
    }

    /// Read all of the normalized sensor data (accelerometer, gyro,
    /// temperature) at once.
    ///
    /// This is faster than reading the sensors individually
    pub async fn measure_norm(&mut self) -> Result<(F32x3, F32x3, f32), Error<E>> {
        let (acc_raw, gyro_raw, temp_raw) = self.measure_raw().await?;

        let mut conf_bytes = [0u8; 2];
        self.i2c
            .write_read(
                self.address as u8,
                &[Bank0::GYRO_CONFIG0.addr()],
                &mut conf_bytes,
            )
            .await
            .map_err(Error::BusError)?;

        let gyro_scale = GyroRange::try_from(conf_bytes[0] >> 5)?.scale_factor();
        let accel_scale = AccelRange::try_from(conf_bytes[1] >> 5)?.scale_factor();

        let temp = (temp_raw as f32 / 132.48) + 25.0;
        let accel_x = acc_raw[Axis::X] as f32 / accel_scale;
        let accel_y = acc_raw[Axis::Y] as f32 / accel_scale;
        let accel_z = acc_raw[Axis::Z] as f32 / accel_scale;

        let gyro_x = gyro_raw[Axis::X] as f32 / gyro_scale;
        let gyro_y = gyro_raw[Axis::Y] as f32 / gyro_scale;
        let gyro_z = gyro_raw[Axis::Z] as f32 / gyro_scale;

        Ok((
            [accel_x, accel_y, accel_z],
            [gyro_x, gyro_y, gyro_z],
            temp,
        ))
    }

    /// Sets the bandwidth of the temperature signal DLPF (Digital Low Pass
    /// Filter)
    ///
    /// This field can be changed on the fly even if the sensor is
    /// on
    pub async fn set_temp_dlpf(&mut self, freq: TempDlpfBw) -> Result<(), Error<E>> {
        self.update_reg(freq).await
    }

    /// Return the currently configured power mode
    pub async fn power_mode(&mut self) -> Result<PowerMode, Error<E>> {
        //  `GYRO_MODE` occupies bits 3:2 in the register
        // `ACCEL_MODE` occupies bits 1:0 in the register
        let bits = self.read_reg(&Bank0::PWR_MGMT0).await? & 0xF;
        let mode = PowerMode::try_from(bits)?;

        Ok(mode)
    }

    /// Set the power mode of the IMU
    pub async fn set_power_mode(&mut self, mode: PowerMode) -> Result<(), Error<E>> {
        self.update_reg(mode).await
    }

    /// Return the currently configured accelerometer range
    pub async fn accel_range(&mut self) -> Result<AccelRange, Error<E>> {
        // `ACCEL_UI_FS_SEL` occupies bits 6:5 in the register
        let fs_sel = self.read_reg(&Bank0::ACCEL_CONFIG0).await? >> 5;
        let range = AccelRange::try_from(fs_sel)?;

        Ok(range)
    }

    /// Set the range of the accelerometer
    pub async fn set_accel_range(&mut self, range: AccelRange) -> Result<(), Error<E>> {
        self.update_reg(range).await
    }

    /// Set acceleration low-power averaging value.
    ///
    /// This field cannot be changed when the accel sensor is in LPM
    /// (LowPowerMode)
    pub async fn set_accel_low_power_avg(&mut self, avg_val: AccLpAvg) -> Result<(), Error<E>> {
        self.update_reg(avg_val).await
    }

    /// Return the currently configured gyroscope range
    pub async fn gyro_range(&mut self) -> Result<GyroRange, Error<E>> {
        // `GYRO_UI_FS_SEL` occupies bits 6:5 in the register
        let fs_sel = self.read_reg(&Bank0::GYRO_CONFIG0).await? >> 5;
        let range = GyroRange::try_from(fs_sel)?;

        Ok(range)
    }

    /// Set the range of the gyro
    pub async fn set_gyro_range(&mut self, range: GyroRange) -> Result<(), Error<E>> {
        self.update_reg(range).await
    }

    /// Selects GYRO UI low pass filter bandwidth
    /// This field can be changed on the fly even if gyro sonsor is on
    pub async fn set_gyro_lp_filter_bandwidth(&mut self, freq: GyroLpFiltBw) -> Result<(), Error<E>> {
        self.update_reg(freq).await
    }

    /// Return the currently configured output data rate for the accelerometer
    pub async fn accel_odr(&mut self) -> Result<AccelOdr, Error<E>> {
        // `ACCEL_ODR` occupies bits 3:0 in the register
        let odr = self.read_reg(&Bank0::ACCEL_CONFIG0).await? & 0xF;
        let odr = AccelOdr::try_from(odr)?;

        Ok(odr)
    }

    /// Set the output data rate of the accelerometer
    pub async fn set_accel_odr(&mut self, odr: AccelOdr) -> Result<(), Error<E>> {
        self.update_reg(odr).await
    }

    /// Selects ACCEL UI low pass filter bandwidth
    /// This field can be changed on-the-fly even if accel sonsor is on
    pub async fn set_accel_dlpf_bw(&mut self, dlpf: AccelDlpfBw) -> Result<(), Error<E>> {
        self.update_reg(dlpf).await
    }

    /// Return the currently configured output data rate for the gyroscope
    pub async fn gyro_odr(&mut self) -> Result<GyroOdr, Error<E>> {
        // `GYRO_ODR` occupies bits 3:0 in the register
        let odr = self.read_reg(&Bank0::GYRO_CONFIG0).await? & 0xF;
        let odr = GyroOdr::try_from(odr)?;

        Ok(odr)
    }

    /// Set the output data rate of the gyroscope
    pub async fn set_gyro_odr(&mut self, odr: GyroOdr) -> Result<(), Error<E>> {
        self.update_reg(odr).await
    }

    /// Read a register at the provided address.
    async fn read_reg<R: Register>(&mut self, reg: &R) -> Result<u8, Error<E>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("read from {=u8:x}", reg.addr());
        let mut buffer = [0u8];
        self.i2c
            .write_read(self.address as u8, &[reg.addr()], &mut buffer)
            .await
            .map_err(Error::BusError)?;
        #[cfg(feature = "defmt")]
        defmt::trace!("read {=[u8:x]}", buffer);
        Ok(buffer[0])
    }

    /// Read a register and the one after it, combining them into a single
    /// value.
    async fn read_reg_i16<R: Register>(&mut self, reg_hi: &R) -> Result<i16, Error<E>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("read from {=u8:x}", reg_hi.addr());
        let mut bytes = [0u8; 2];
        self.i2c
            .write_read(self.address as u8, &[reg_hi.addr()], &mut bytes)
            .await
            .map_err(Error::BusError)?;

        #[cfg(feature = "defmt")]
        defmt::trace!("read {=[u8:x]}", bytes);
        let data = i16::from_be_bytes([bytes[0], bytes[1]]);

        Ok(data)
    }

    /// Read six consecutive registers and combine them into three 16-bit
    /// values.
    async fn read_reg_i16_triplet<R: Register>(
        &mut self,
        reg_hi: &R,
    ) -> Result<(i16, i16, i16), Error<E>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("read from {=u8:x}", reg_hi.addr());
        let mut bytes = [0u8; 6];
        self.i2c
            .write_read(self.address as u8, &[reg_hi.addr()], &mut bytes)
            .await
            .map_err(Error::BusError)?;

        #[cfg(feature = "defmt")]
        defmt::trace!("read {=[u8:x]}", bytes);

        let word1 = i16::from_be_bytes([bytes[0], bytes[1]]);
        let word2 = i16::from_be_bytes([bytes[2], bytes[3]]);
        let word3 = i16::from_be_bytes([bytes[4], bytes[5]]);

        Ok((word1, word2, word3))
    }

    /// Set a register at the provided address to a given value.
   async fn write_reg<R: Register>(&mut self, reg: &R, value: u8) -> Result<(), Error<E>> {
        if reg.read_only() {
            Err(Error::SensorError(SensorError::WriteToReadOnly))
        } else {
            #[cfg(feature = "defmt")]
            defmt::trace!("write to {=u8:x} <- {=u8:b}", reg.addr(), value);
            self.i2c
                .write(self.address as u8, &[reg.addr(), value])
                .await
                .map_err(Error::BusError)
        }
    }

    /// Update the register at the provided address.
    ///
    /// Rather than overwriting any active bits in the register, we first read
    /// in its current value and then update it accordingly using the given
    /// value and mask before writing back the desired value.
    async fn update_reg<BF: Bitfield>(&mut self, value: BF) -> Result<(), Error<E>> {
        if BF::REGISTER.read_only() {
            Err(Error::SensorError(SensorError::WriteToReadOnly))
        } else {
            #[cfg(feature = "defmt")]
            defmt::trace!("update {=u8:x}...", BF::REGISTER.addr());
            let current = self.read_reg(&BF::REGISTER).await?;
            let value = (current & !BF::BITMASK) | (value.bits() & BF::BITMASK);

            self.write_reg(&BF::REGISTER, value).await
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Axis {
    X,
    Y,
    Z,
}

pub const trait Vector: Copy + [const] core::ops::IndexMut<Axis, Output = Self::Component> {
    type Component: Copy;
    fn x(&self) -> &Self::Component {
        self.index(Axis::X)
    }
    fn y(&self) -> &Self::Component {
        self.index(Axis::Y)
    }
    fn z(&self) -> &Self::Component {
        self.index(Axis::Z)
    }
}

pub type I16x3 = [i16; 3];
pub type F32x3 = [f32; 3];

impl const Vector for I16x3 {
    type Component = i16;
}

impl const Vector for F32x3 {
    type Component = f32;
}

impl<T> const core::ops::Index<Axis> for [T; 3] {
    type Output = T;

    fn index(&self, index: Axis) -> &Self::Output {
        &self[index as usize]
    }
}

impl<T> const core::ops::IndexMut<Axis> for [T; 3] {
    fn index_mut(&mut self, index: Axis) -> &mut Self::Output {
        &mut self[index as usize]
    }
}