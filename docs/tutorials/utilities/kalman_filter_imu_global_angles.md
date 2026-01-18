# Using the 2D Kalman Filter

This tutorial demonstrates how to use the 2D Kalman Filter with the BHI260AP IMU to estimate global roll and pitch angles with the Open Source Leg platform.

## Hardware Setup

1. Connect the BHI260AP IMU
2. Identify which IMU physical axes correspond to the global roll, pitch, and yaw axes for your desired IMU configuration

This example shows how to:

- Initialize and configure the 2D Kalman Filter
- Estimate global 2D orientation
- Log orientation measurements

## Code Structure

The [tutorial script](https://github.com/neurobionics/opensourceleg/blob/main/tutorials/utilities/kalman_filter_imu_global_angles.py) is organized into several main sections:

### 1. Initialization

```python
--8<-- "tutorials/utilities/kalman_filter_imu_global_angles.py:1:37"
```

This section:

- Creates a data logger for recording measurements
- Sets up a real-time loop for consistent timing
- Initializes the BHI260AP with gyroscope and gravity sensors enabled
- Configures variable tracking for global angles
- Configures axis transformation for physical IMU axes that correspond to global roll, pitch, and yaw axes

### 2. Main Loop

```python
--8<-- "tutorials/utilities/kalman_filter_imu_global_angles.py:39:54"
```

The main loop:

1. Updates the IMU to get the latest reading
2. Transforms IMU gyroscope and gravity readings into global roll, pitch, yaw axis convention
3. Updates Kalman filter to estimate global roll and pitch orientation
2. Logs the time and current global orientation data
3. Updates the logger

## IMU Parameters

When initializing the 2D Kalman Filter, several important parameters can be configured:

```python
--8<-- "tutorials/utilities/kalman_filter_imu_global_angles.py:25:30"
```

### Parameter Details

1. **tag** (str):
      - Unique identifier for the 2D Kalman Filter instance
      - Useful when using multiple sensors
      - Defaults to "KalmanFilter2D"

2. **Q_bias** (float):
      - Specifies variance in gyroscope drift rate

3. **Q_angle** (float):
      - Specifies uncertainty in gyroscope angle prediction

4. **R_var** (float):
      - Specifies accelerometer angle measurement noise


## Running the Example

1. Navigate to the tutorial directory:
   ```bash
   cd tutorials/utilities
   ```

2. Run the script:
   ```bash
   python kalman_filter_imu_global_angles.py
   ```

3. Expected behavior:
      - Kalman filter begins estimating global orientation continuously at 200Hz
      - Data is logged to `./logs/kalman_filter.csv`
      - Roll and pitch values update as you rotate the IMU

## Common Issues
Let us know if you find any.


## Additional Resources

If you have any questions or need further assistance, please post on the [Open Source Leg community forum](https://opensourceleg.discourse.group).
