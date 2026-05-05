# Using the Lord Microstrain IMU

This tutorial demonstrates how to use the Lord Microstrain IMU with the Open Source Leg platform.

## Hardware Setup

1. Connect the Lord Microstrain IMU to your computer via USB
2. Note the device port (typically `/dev/ttyUSB0`)
3. Mount the IMU securely with proper orientation
4. Verify the LED indicators are active

This example shows how to:

- Initialize and configure a Lord Microstrain IMU
- Read orientation, angular velocity, and acceleration data
- Log IMU measurements

## Code Structure

The [tutorial script](https://github.com/neurobionics/opensourceleg/blob/main/tutorials/sensors/reading_microstrain_imu_data.py) is organized into several main sections:

### 1. Initialization

```python
--8<-- "tutorials/sensors/reading_microstrain_imu_data.py:1:26"
```

This section:

- Creates a data logger for recording measurements
- Sets up a real-time loop for consistent timing
- Initializes the LordMicrostrainIMU with specified parameters
- Configures variable tracking for roll, pitch, and yaw angles

### 2. Main Loop

```python
--8<-- "tutorials/sensors/reading_microstrain_imu_data.py:28:32"
```

The main loop:

1. Updates the IMU to get the latest reading
2. Logs the time and current orientation data
3. Updates the logger

## IMU Parameters

When initializing the LordMicrostrainIMU, several important parameters can be configured:

```python
--8<-- "tutorials/sensors/reading_microstrain_imu_data.py:14:23"
```

### Parameter Details

1. **tag** (str):
      - Unique identifier for the IMU instance
      - Useful when using multiple sensors
      - Defaults to "LordMicrostrainIMU"

2. **port** (str):
      - Specifies the serial port where the IMU is connected
      - Default is `/dev/ttyUSB0`
      - Use `ls /dev/ttyUSB*` to see available devices

3. **baud_rate** (int):
      - Communication speed for serial connection
      - Default is 921600 baud
      - Must match IMU's configured baud rate

4. **frequency** (int):
      - Data streaming frequency in Hz
      - Default is 200 Hz
      - Higher rates provide more data but increase processing load

5. **update_timeout** (int):
      - Timeout for data packet retrieval in milliseconds
      - Default is 500ms
      - Increase if data retrieval is unreliable

6. **max_packets** (int):
      - Maximum number of data packets to retrieve per update
      - Default is 1
      - Higher values can buffer more data

7. **return_packets** (bool):
      - If True, returns the raw data packets in the update() method
      - Default is False
      - Useful for advanced processing

## Available Properties

The LordMicrostrainIMU provides several useful properties:

1. **Orientation** (roll, pitch, yaw):
      - Current angular orientation in radians
      - Returned as Euler angles
      ```python
      imu.roll    # Rotation around X-axis
      imu.pitch   # Rotation around Y-axis
      imu.yaw     # Rotation around Z-axis
      ```

2. **Angular Velocity** (vel_x, vel_y, vel_z):
      - Angular rates in radians per second
      ```python
      imu.vel_x   # Angular velocity around X-axis
      imu.vel_y   # Angular velocity around Y-axis
      imu.vel_z   # Angular velocity around Z-axis
      ```

3. **Linear Acceleration** (acc_x, acc_y, acc_z):
      - Linear accelerations in m/s²
      ```python
      imu.acc_x   # Acceleration along X-axis
      imu.acc_y   # Acceleration along Y-axis
      imu.acc_z   # Acceleration along Z-axis
      ```

4. **Raw Data**:
      - Access to complete data packet
      ```python
      imu.data    # Dictionary with all available channels
      ```

## Running the Example

1. Navigate to the tutorial directory:
   ```bash
   cd tutorials/sensors
   ```

2. Run the script:
   ```bash
   python imu.py
   ```

3. Expected behavior:
      - IMU begins reading orientation data continuously at 200Hz
      - Data is logged to `./logs/reading_imu_data.csv`
      - Roll, pitch, and yaw values update as you rotate the IMU

## Common Issues

- **Device Not Found**: Verify connections and run `ls /dev/ttyUSB*`
- **Permission Denied**: Add user to dialout group: `sudo usermod -a -G dialout $USER`
- **Incorrect Readings**: Check IMU orientation and mounting
- **Missing Data**: Verify baud rate and connection quality
- **MSCL Import Error**: Install the MSCL library from Lord Microstrain

## Additional Resources

- [MSCL Documentation](https://lord-microstrain.github.io/MSCL/Documentation/MSCL%20API%20Documentation/index.html)

If you have any questions or need further assistance, please post on the [Open Source Leg community forum](https://opensourceleg.discourse.group).
