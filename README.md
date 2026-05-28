# icm40627

An `embedded-hal` driver for the ICM-40627 6-axis IMU.

Forked from [icm42670]: https://github.com/jessebraham/icm42670 to support this similar IMU instead.

While this device supports communication via I²C, SPI, and I3C, presently only I²C is supported. In its current state we are able to read the accelerometer, gyroscope, and temperature sensor data and perform basic configuration of the device. Reading packets from the FIFO is not currently supported.

If there is a feature which has not yet been implemented and which you are interested in, please feel free to open an issue and/or a pull request!

## Examples

Examples demonstrating how to use this driver can be found in the [icm42670-examples] repository.

[icm42670-examples]: https://github.com/jessebraham/icm42670-examples

## Resources

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
