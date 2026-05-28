use crate::{
    error::SensorError,
    register::{Bank0, Bank1, Register},
};

pub(crate) trait Bitfield {
    const BITMASK: u8;
    type Reg: Register;
    const REGISTER: Self::Reg;

    /// Bit value of a discriminant, shifted to the correct position if
    /// necessary
    fn bits(self) -> u8;
}

/// I²C slave addresses, determined by the logic level of pin `AP_AD0`
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Address {
    /// `AP_AD0` pin == 0
    Primary   = 0b1101000,
    /// `AP_AD0` pin == 1
    Secondary = 0b1101001,
}

/// Configurable ranges of the Accelerometer
///
#[derive(Clone, Copy, Default, Debug, PartialEq)]
pub enum AccelRange {
    /// ±2G
    G2  = 3,
    /// ±4G
    G4  = 2,
    /// ±8G
    G8  = 1,
    /// ±16G
    #[default]
    G16 = 0,
}

impl AccelRange {
    /// Sensitivity scale factor, used to convert acceleration
    /// register values to g based on the configured range
    pub fn scale_factor(&self) -> f32 {
        use AccelRange::*;

        // Values taken from `Table 2. Accelerometer Specifications` of the data sheet
        match self {
            G2 => 16_384.0,
            G4 => 8_192.0,
            G8 => 4_096.0,
            G16 => 2_048.0,
        }
    }
}

impl Bitfield for AccelRange {
    const BITMASK: u8 = 0b1110_0000;
    type Reg = Bank0;
    const REGISTER: Self::Reg = Self::Reg::ACCEL_CONFIG0;

    fn bits(self) -> u8 {
        // `ACCEL_FS_SEL` occupies bits 7:5 in the register
        (self as u8) << 5
    }
}

impl TryFrom<u8> for AccelRange {
    type Error = SensorError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        use AccelRange::*;

        match value {
            0 => Ok(G16),
            1 => Ok(G8),
            2 => Ok(G4),
            3 => Ok(G2),
            _ => Err(SensorError::InvalidDiscriminant),
        }
    }
}

/// Configurable ranges of the Gyroscope
#[derive(Clone, Copy, Default, Debug, PartialEq)]
pub enum GyroRange {
    /// ±15.625 deg/sec
    Deg15_625 = 7,
    /// ±31.25 deg/sec
    Deg31_25  = 6,
    /// ±62.5 deg/sec
    Deg62_5   = 5,
    /// ±125 deg/sec
    Deg125    = 4,
    /// ±250 deg/sec
    Deg250    = 3,
    /// ±500 deg/sec
    Deg500    = 2,
    /// ±1000 deg/sec
    Deg1000   = 1,
    /// ±2000 deg/sec
    #[default]
    Deg2000   = 0,
}

impl GyroRange {
    /// Sensitivity scale factor
    pub fn scale_factor(&self) -> f32 {
        use GyroRange::*;

        // Values taken from `Table 1. Gyroscope Specifications` of the data sheet
        match self {
            Deg15_625 => 15.625,
            Deg31_25 => 31.25,
            Deg62_5 => 62.5,
            Deg125 => 125.0,
            Deg250 => 250.0,
            Deg500 => 500.0,
            Deg1000 => 1000.0,
            Deg2000 => 2000.0,
        }
    }
}

impl Bitfield for GyroRange {
    const BITMASK: u8 = 0b1110_0000;
    type Reg = Bank0;
    const REGISTER: Self::Reg = Self::Reg::GYRO_CONFIG0;

    fn bits(self) -> u8 {
        // `GYRO_FS_SEL` occupies bits 7:5 in the register
        (self as u8) << 5
    }
}

impl TryFrom<u8> for GyroRange {
    type Error = SensorError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        use GyroRange::*;

        match value {
            0 => Ok(Deg2000),
            1 => Ok(Deg1000),
            2 => Ok(Deg500),
            3 => Ok(Deg250),
            _ => Err(SensorError::InvalidDiscriminant),
        }
    }
}

/// Configurable power modes of the IMU
#[derive(Clone, Copy, Default, Debug, PartialEq)]
pub enum PowerMode {
    /// Gyroscope: OFF, Accelerometer: OFF
    #[default]
    Sleep           = 0b0000_0000,
    /// Sleep but with the RC oscillator powered on
    Idle            = 0b0001_0000,
    /// Gyroscope: DRIVE ON, Accelerometer: OFF
    Standby         = 0b0000_0100,
    /// Gyroscope: OFF, Accelerometer: DUTY-CYCLED
    AccelLowPower   = 0b0000_0010,
    /// Gyroscope: OFF, Accelerometer: ON
    AccelLowNoise   = 0b0000_0011,
    /// Gyroscope: ON, Accelerometer: OFF
    GyroLowNoise    = 0b0000_1100,
    /// Gyroscope: ON, Accelerometer: ON
    SixAxisLowNoise = 0b0000_1111,
    /// Temperature: OFF, Gyroscope: ON, Accelerometer: ON
    SixAxisLowNoiseWithoutTemperature = 0b0010_1111,
}

impl Bitfield for PowerMode {
    const BITMASK: u8 = 0b0011_1111;
    type Reg = Bank0;
    const REGISTER: Self::Reg = Self::Reg::PWR_MGMT0;

    fn bits(self) -> u8 {
        // `TEMP_DIS` occupies bit 5 in the register
        // `IDLE` occupies bit 4 in the register
        // `GYRO_MODE` occupies bits 3:2 in the register
        // `ACCEL_MODE` occupies bits 1:0 in the register
        self as u8
    }
}

impl TryFrom<u8> for PowerMode {
    type Error = SensorError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        use PowerMode::*;

        match value {
            0b0000_0000 => Ok(Sleep),
            0b0001_0000 => Ok(Idle),
            0b0000_0100 => Ok(Standby),
            0b0000_0010 => Ok(AccelLowPower),
            0b0000_0011 => Ok(AccelLowNoise),
            0b0000_1100 => Ok(GyroLowNoise),
            0b0000_1111 => Ok(SixAxisLowNoise),
            0b0010_1111 => Ok(SixAxisLowNoiseWithoutTemperature),
            _ => Err(SensorError::InvalidDiscriminant),
        }
    }
}

/// Accelerometer ODR selection values
#[derive(Clone, Copy, Default, Debug, PartialEq)]
pub enum AccelOdr {
    /// 8 kHz (LN mode)
    Hz8000   = 0b0011,
    /// 4 kHz (LN mode)
    Hz4000   = 0b0100,
    /// 2 kHz (LN mode)
    Hz2000   = 0b0101,
    /// 1 kHz (LN mode)
    #[default]
    Hz1000   = 0b0110,
    /// 1.5625 Hz (LP or LN mode)
    Hz500    = 0b1111,
    /// 200 Hz (LP or LN mode)
    Hz200    = 0b0111,
    /// 100 Hz (LP or LN mode)
    Hz100    = 0b1000,
    /// 50 Hz (LP or LN mode)
    Hz50     = 0b1001,
    /// 25 Hz (LP or LN mode)
    Hz25     = 0b1010,
    /// 12.5 Hz (LP or LN mode)
    Hz12_5   = 0b1011,
    /// 6.25 Hz (LP mode)
    Hz6_25   = 0b1100,
    /// 3.125 Hz (LP mode)
    Hz3_125  = 0b1101,
    /// 1.5625 Hz (LP mode)
    Hz1_5625 = 0b1110,
}

impl AccelOdr {
    pub fn as_f32(self) -> f32 {
        use AccelOdr::*;

        match self {
            Hz8000 => 8000.0,
            Hz4000 => 4000.0,
            Hz2000 => 2000.0,
            Hz1000 => 1000.0,
            Hz500 => 500.0,
            Hz200 => 200.0,
            Hz100 => 100.0,
            Hz50 => 50.0,
            Hz25 => 25.0,
            Hz12_5 => 12.5,
            Hz6_25 => 6.25,
            Hz3_125 => 3.125,
            Hz1_5625 => 1.5625,
        }
    }
}

impl Bitfield for AccelOdr {
    const BITMASK: u8 = 0b0000_1111;
    type Reg = Bank0;
    const REGISTER: Self::Reg = Self::Reg::ACCEL_CONFIG0;

    fn bits(self) -> u8 {
        // `ACCEL_ODR` occupies bits 3:0 in the register
        self as u8
    }
}

impl TryFrom<u8> for AccelOdr {
    type Error = SensorError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        use AccelOdr::*;

        match value {
            0b0011 => Ok(Hz8000),
            0b0100 => Ok(Hz4000),
            0b0101 => Ok(Hz2000),
            0b0110 => Ok(Hz1000),
            0b1111 => Ok(Hz500),
            0b0111 => Ok(Hz200),
            0b1000 => Ok(Hz100),
            0b1001 => Ok(Hz50),
            0b1010 => Ok(Hz25),
            0b1011 => Ok(Hz12_5),
            0b1100 => Ok(Hz6_25),
            0b1101 => Ok(Hz3_125),
            0b1110 => Ok(Hz1_5625),
            _ => Err(SensorError::InvalidDiscriminant),
        }
    }
}

/// Acceleration Low Power Averaging
#[derive(Clone, Copy, Default, Debug, PartialEq)]
pub enum AccLpAvg {
    #[default]
    X1  = 1,
    X16  = 6,
}

impl Bitfield for AccLpAvg {
    const BITMASK: u8 = 0b1111_0000;
    type Reg = Bank0;
    const REGISTER: Self::Reg = Self::Reg::GYRO_ACCEL_CONFIG0;

    fn bits(self) -> u8 {
        (self as u8) << 4
    }
}

/// Acceleration Digital Low Pass Filter options
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AccelDlpfBw {
    Bypassed = 0b000,
    Hz180    = 0b001,
    Hz121    = 0b010,
    Hz73     = 0b011,
    Hz53     = 0b100,
    Hz34     = 0b101,
    Hz25     = 0b110,
    Hz16     = 0b111,
}

impl Bitfield for AccelDlpfBw {
    const BITMASK: u8 = 0b0000_0111;
    type Reg = Bank0;
    const REGISTER: Self::Reg = Self::Reg::ACCEL_CONFIG1;

    fn bits(self) -> u8 {
        self as u8
    }
}

/// Temperature DLPF (Digital Low Pass Filter) Bandwidth
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TempDlpfBw {
    Bypassed = 0b000,
    Hz180    = 0b001,
    Hz72     = 0b010,
    Hz34     = 0b011,
    Hz16     = 0b100,
    Hz8      = 0b101,
    Hz4      = 0b110,
}
impl Bitfield for TempDlpfBw {
    const BITMASK: u8 = 0b1110_0000;
    type Reg = Bank0;
    const REGISTER: Self::Reg = Self::Reg::GYRO_CONFIG1;

    fn bits(self) -> u8 {
        (self as u8) << 5
    }
}

/// Gyroscope UI low pass filter bandwidth
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum GyroLpFiltBw {
    Bypassed = 0b000,
    Hz180    = 0b001,
    Hz121    = 0b010,
    Hz73     = 0b011,
    Hz53     = 0b100,
    Hz34     = 0b101,
    Hz25     = 0b110,
    Hz16     = 0b111,
}
impl Bitfield for GyroLpFiltBw {
    const BITMASK: u8 = 0b0000_0111;
    type Reg = Bank0;
    const REGISTER: Self::Reg = Self::Reg::GYRO_ACCEL_CONFIG0;

    fn bits(self) -> u8 {
        self as u8
    }
}
/// Gyroscope ODR selection values
///
/// Note that this enum is sorted from greatest to least
///  Hz, which is different to how these values are listed
///  in the datasheet.
#[derive(Clone, Copy, Default, Debug, PartialEq)]
pub enum GyroOdr {
    /// 8 kHz
    Hz8000 = 0b0011,
    /// 4 kHz
    Hz4000 = 0b0100,
    /// 2 kHz
    Hz2000 = 0b0101,
    /// 1 kHz
    #[default]
    Hz1000 = 0b0110,
    /// 500 Hz
    Hz500  = 0b1111,
    /// 200 Hz
    Hz200  = 0b0111,
    /// 100 Hz
    Hz100  = 0b1000,
    /// 50 Hz
    Hz50   = 0b1001,
    /// 25 Hz
    Hz25   = 0b1010,
    /// 12.5 Hz
    Hz12_5 = 0b1011,
}

impl GyroOdr {
    pub fn as_f32(self) -> f32 {
        use GyroOdr::*;

        match self {
            Hz8000 => 8000.0,
            Hz4000 => 4000.0,
            Hz2000 => 2000.0,
            Hz1000 => 1000.0,
            Hz500 => 500.0,
            Hz200 => 200.0,
            Hz100 => 100.0,
            Hz50 => 50.0,
            Hz25 => 25.0,
            Hz12_5 => 12.5,
        }
    }
}

impl Bitfield for GyroOdr {
    const BITMASK: u8 = 0b0000_1111;
    type Reg = Bank0;
    const REGISTER: Self::Reg = Self::Reg::GYRO_CONFIG0;

    fn bits(self) -> u8 {
        // `GYRO_ODR` occupies bits 3:0 in the register
        self as u8
    }
}

impl TryFrom<u8> for GyroOdr {
    type Error = SensorError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        use GyroOdr::*;

        match value {
            0b0011 => Ok(Hz8000),
            0b0100 => Ok(Hz4000),
            0b0101 => Ok(Hz2000),
            0b0110 => Ok(Hz1000),
            0b1111 => Ok(Hz500),
            0b0111 => Ok(Hz200),
            0b1000 => Ok(Hz100),
            0b1001 => Ok(Hz50),
            0b1010 => Ok(Hz25),
            0b1011 => Ok(Hz12_5),
            _ => Err(SensorError::InvalidDiscriminant),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct SoftReset;

impl Bitfield for SoftReset {
    const BITMASK: u8 = 0b0000_1000;
    type Reg = Bank0;
    const REGISTER: Self::Reg = Self::Reg::SIGNAL_PATH_RESET;

    fn bits(self) -> u8 {
        1 << 3
    }
}

#[allow(dead_code)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum SpiWireCount {
    ThreeWire = 0b0,
    FourWire  = 0b1,
}

impl Bitfield for SpiWireCount {
    const BITMASK: u8 = 0b0000_0010;
    type Reg = Bank1;
    const REGISTER: Self::Reg = Self::Reg::INTF_CONFIG4;

    fn bits(self) -> u8 {
        (self as u8) << 1
    }
}

#[allow(dead_code)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum SpiMode {
    Mode0And3 = 0b0,
    Mode1And2 = 0b1,
}

impl Bitfield for SpiMode {
    const BITMASK: u8 = 0b0000_0001;
    type Reg = Bank0;
    const REGISTER: Self::Reg = Self::Reg::DEVICE_CONFIG;

    fn bits(self) -> u8 {
        self as u8
    }
}

/// Controls slew rate for output pin 14 in I2C mode.
/// After device reset, the I2C_SLEW_RATE is used by default. If the 1st write
/// operation from host is an SPI transaction, the device automatically switches
/// to SPI_SLEW_RATE.
#[allow(dead_code)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum I2CSlewRate {
    /// Min 20ns; Max 60ns
    M20M60 = 0b000,
    /// Min 12ns; Max 36ns
    M12M36 = 0b001,
    /// Min 6ns; Max 18ns
    M6M18  = 0b010,
    /// Min 4ns; Max 12ns
    M4M12   = 0b011,
    /// Min 2ns; Max 6ns
    M2M6    = 0b100,
    /// Max 2ns
    M2      = 0b101,
}

impl Bitfield for I2CSlewRate {
    const BITMASK: u8 = 0b0011_1000;
    type Reg = Bank0;
    const REGISTER: Self::Reg = Self::Reg::DRIVE_CONFIG;

    fn bits(self) -> u8 {
        (self as u8) << 3
    }
}

/// Controls slew rate for output pin 14 in SPI 3-wire mode. In SPI 4-wire mode
/// this register controls the slew rate of pin 1 as it is used as an output in
/// SPI 4- wire mode only. After chip reset, the I2C_SLEW_RATE is used by
/// default for pin 14 pin. If the 1st write operation from the host is an
/// SPI3/4 transaction, the device automatically switches to SPI_SLEW_RATE.
///
/// This register field should not be programmed in I3C/DDR mode.
#[allow(dead_code)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum SpiSlewRate {
    /// Min 20ns; Max 60ns
    M20M60 = 0b000,
    /// Min 12ns; Max 36ns
    M12M36 = 0b001,
    /// Min 6ns; Max 18ns
    M6M18  = 0b010,
    /// Min 4ns; Max 12ns
    M4M12  = 0b011,
    /// Min 2ns; Max 6ns
    M2M6   = 0b100,
    /// Max 2ns
    M2     = 0b101,
}

impl Bitfield for SpiSlewRate {
    const BITMASK: u8 = 0b0000_0111;
    type Reg = Bank0;
    const REGISTER: Self::Reg = Self::Reg::DRIVE_CONFIG;

    fn bits(self) -> u8 {
        self as u8
    }
}
