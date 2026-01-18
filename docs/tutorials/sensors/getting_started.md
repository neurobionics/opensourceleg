# Getting Started with Sensors

This guide introduces the `opensourceleg.sensors` module and provides essential information for all sensor tutorials.

## Prerequisites

- Raspberry Pi
- Python environment with `opensourceleg` package installed
- Supported sensors:

    - AS5048B Encoder
    - Lord Microstrain IMU
    - Dephy Loadcell Amplifier

## Hardware Setup

Each sensor has specific connection requirements:

1. Encoder (AS5048B):

    - Connects via I2C interface
    - Uses GPIO pins

2. IMU (Lord Microstrain):

    - Connects via serial (typically at `/dev/ttyUSB0`)
    - Keep note of the orientation of the IMU

3. Loadcell (Dephy Amplifier):

    - Connects via I2C interface
    - Requires proper calibration matrix of your loadcell
    - Needs correct amplifier gain settings

## Best Practices

### 1. Initialization

- Verify proper connections before powering on
- Check communication interfaces (I2C, USB)
- Ensure correct device addresses
- Initialize sensors with appropriate parameters

### 2. Data Collection

- Use appropriate sampling frequencies
- Implement proper error handling
- Log data for analysis
- Monitor sensor readings for anomalies

### 3. Maintenance

- Regular calibration checks
- Check connections periodically
- Monitor for drift or inconsistencies

## Troubleshooting Guide

Common issues and solutions:

- I2C Communication:

  ```bash
  # List I2C devices
  i2cdetect -y 1
  ```
  also make sure i2c is enabled in raspi-config if you are using a raspberry pi

- USB Device Detection:

  ```bash
  # Check USB devices
  ls /dev/ttyUSB*
  ls /dev/ttyACM*
  ```

- Permission Issues:

  ```bash
  # Add user to required groups
  sudo usermod -a -G dialout $USER
  ```

## Getting Help

If you encounter any issues:

1. Check the specific sensor's documentation
2. Review the [API documentation](../../api/sensors/sensors_base.md)
3. Post questions on the [Open Source Leg community forum](https://opensourceleg.discourse.group)

## Next Steps

Choose a sensor tutorial to get started:

- [Reading Encoder Data](reading_encoder_data.md)
- [Reading IMU Data](reading_microstrain_imu_data.md)
- [Reading Loadcell Data](reading_loadcell_data.md)

Each tutorial includes specific setup instructions and code samples.
