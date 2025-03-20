# Using the Lord Microstrain IMU

This tutorial demonstrates how to use the Lord Microstrain IMU with the Open Source Leg platform.

## Hardware Setup

1. Connect the Lord Microstrain IMU to your computer via USB
2. Note the device port (typically `/dev/ttyUSB0`)
3. Mount the IMU securely with proper orientation
4. Verify the LED indicators are active

## Code Example

The following example demonstrates reading orientation data from the IMU. You can find the complete code in ```1:39:tutorials/sensors/imu.py```.

```python
from opensourceleg.logging.logger import Logger
from opensourceleg.sensors.imu import LordMicrostrainIMU
from opensourceleg.time import SoftRealtimeLoop

FREQUENCY = 200  # 200Hz sampling rate
DT = 1 / FREQUENCY
```

## Code Explanation

Let's break down the key components:

1. **Configuration** (```1:8:tutorials/sensors/imu.py```):
   - We import necessary modules
   - Set sampling frequency to 200Hz (typical for IMU applications)

2. **Logger Setup** (```10:15:tutorials/sensors/imu.py```):
   ```python
   imu_logger = Logger(
       log_path="./logs",
       file_name="reading_imu_data",
   )
   ```

3. **IMU Initialization** (```16:21:tutorials/sensors/imu.py```):
   ```python
   imu = LordMicrostrainIMU(
       port="/dev/ttyUSB0",
       frequency=FREQUENCY,
   )
   ```

4. **Data Tracking** (```22:24:tutorials/sensors/imu.py```):
   - Track roll, pitch, and yaw values
   - Each value is logged separately for analysis

5. **Main Loop** (```26:31:tutorials/sensors/imu.py```):
   - Uses context manager for safe resource handling
   - Continuously reads and logs IMU data
   - Updates at specified frequency

## Expected Output

The script outputs three orientation angles in radians:
- Roll (rotation around X-axis)
- Pitch (rotation around Y-axis)
- Yaw (rotation around Z-axis)

## Common Issues

1. **Device Not Found**
   - Verify the USB port:
     ```bash
     ls /dev/ttyUSB*
     ```
   - Check if device is properly connected
   - Try a different USB port

2. **Permission Denied**
   - Add user to dialout group:
     ```bash
     sudo usermod -a -G dialout $USER
     ```
   - Log out and log back in for changes to take effect

3. **Incorrect Readings**
   - Verify IMU mounting orientation
   - Check for magnetic interference
   - Ensure IMU is properly calibrated

## Advanced Usage

- Adjust sampling frequency based on your application needs
- Implement filtering for noise reduction
- Consider using quaternion output for more precise orientation

## Next Steps

- Try modifying the sampling frequency
- Implement data filtering
- Add error handling for robustness
- Integrate with other sensors

For more detailed information, refer to:
- [IMU API Documentation](../../api/sensors/imu.md)
- [Lord Microstrain Documentation](https://lord-microstrain.github.io/MSCL/Documentation/Getting%20Started/index.html)
