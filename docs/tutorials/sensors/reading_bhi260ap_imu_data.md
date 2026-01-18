# Using the BHI260AP IMU

This tutorial demonstrates how to use the BHI260AP IMU with the Open Source Leg platform.

## Hardware Setup

1. Connect the BHI260AP IMU to your device via an SPI port
2. Verify that the SPI port and chip-select is available

This example shows how to:

- Initialize and configure a BHI260AP IMU
- Read gyroscope and acceleration data
- Log IMU measurements

## Code Structure

The [tutorial script](https://github.com/neurobionics/opensourceleg/blob/main/tutorials/sensors/reading_bhi260ap_imu_data.py) is organized into several main sections:

### 1. Initialization

```python
--8<-- "tutorials/sensors/reading_bhi260ap_imu_data.py:1:29"
```

This section:

- Creates a data logger for recording measurements
- Sets up a real-time loop for consistent timing
- Initializes the BHI260AP with gyroscope and raw accelerometer enabled
- Configures variable tracking for x,y,z gyroscope and raw accelerometer values

### 2. Main Loop

```python
--8<-- "tutorials/sensors/reading_bhi260ap_imu_data.py:31:38"
```

The main loop:

1. Updates the IMU to get the latest reading
2. Logs the time and current gyroscope and accelerometer data
3. Updates the logger

## IMU Parameters

When initializing the BHI260AP, several important parameters can be configured:

```python
--8<-- "tutorials/sensors/reading_bhi260ap_imu_data.py:14:23"
```

### Parameter Details

1. **tag** (str):
      - Unique identifier for the IMU instance
      - Useful when using multiple sensors
      - Defaults to "BHI260AP"

2. **spi_bus** (int):
      - Specifies the SPI port where the IMU is connected
      - Default is 0
      - Use `ls /dev/spi*` to see available SPI ports

3. **spi_cs** (int):
      - Specifies SPI chip-select where the IMU is connected
      - Default is 2
      - Use `ls /dev/spi*` to see available SPI ports

4. **clock_freq** (int):
      - SPI clock frequency
      - Default is 2 MHz

5. **data_rate** (int):
      - Data streaming frequency in Hz
      - Default is 200 Hz
      - Higher rates provide more data but increase processing load

6. **firmware_path** (str):
      - Specifies file path to BHI260AP.fw firmware file

## Available Properties

The BHI260AP IMU has several useful properties. Each sensor must first be enabled before accessing property.

1. **Gyroscope** (x,y,z):
      - Enable gyroscope sensor via `imu.enable_gyroscope()`
      - Current angular velocity in radians per second
      ```python
      imu.gyro_x   # Angular velocity around X-axis
      imu.gyro_y   # Angular velocity around Y-axis
      imu.gyro_z   # Angular velocity around Z-axis
      imu.gyro    # (x,y,z) angular velocities
      ```

2. **Raw Acceleration** (x,y,z):
      - Enable raw accelerometer sensor via `imu.enable_accelerometer()`
      - Current raw accelerations in m/s²
      ```python
      imu.accel    # (x,y,z) accelerations
      imu.acc_x    # Acceleration along X-axis
      imu.acc_y    # Acceleration along Y-axis
      imu.acc_z    # Acceleration along Z-axis
      ```

3. **Gravity** (x,y,z):
      - Enable gravity sensor via `imu.enable_gravity()`
      - Current acceleration due to gravity in m/s²
      ```python
      imu.gravity    # (x,y,z) accelerations
      ```

3. **Linear Accelerations** (x,y,z):
      - Enable linear acceleration sensor via `imu.enable_linear_acceleration()`
      - Current linear acceleration in m/s²
      ```python
      imu.lin_accel    # (x,y,z) accelerations
      ```

## Running the Example

1. Navigate to the tutorial directory:
   ```bash
   cd tutorials/sensors
   ```

2. Run the script:
   ```bash
   python reading_bhi260ap_imu_data.py
   ```

3. Expected behavior:
      - IMU begins reading gyroscope and accelerometer data continuously at 200Hz
      - Data is logged to `./logs/reading_bhi260ap_data.csv`
      - Gyroscope and accelerometer values update as you rotate the IMU

## Common Issues

- **Device Not Found**: Verify connections and run `ls /dev/spi*`
- **Firmware Not Found**: Download the Bosch BHI260AP.fw firmware file

## Additional Resources

- [Bosch Documentation](https://www.bosch-sensortec.com/products/smart-sensor-systems/bhi260ap/#documents)

If you have any questions or need further assistance, please post on the [Open Source Leg community forum](https://opensourceleg.discourse.group).
