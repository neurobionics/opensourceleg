# Using the Dephy Loadcell Amplifier

This tutorial demonstrates how to use the Dephy Loadcell Amplifier with the Open Source Leg platform to measure forces and moments.

## Hardware Setup

1. Connect the loadcell to the Dephy Loadcell Amplifier and the amplifier to the Raspberry Pi via I2C
2. Verify proper power supply connections
3. Ensure proper grounding
4. Mount the loadcell securely

This example shows how to:

- Initialize and configure a Dephy Loadcell Amplifier
- Read forces and moments (6-axis measurements)
- Log loadcell measurements

## Code Structure

The [tutorial script](https://github.com/neurobionics/opensourceleg/blob/main/tutorials/sensors/reading_loadcell_data.py) is organized into several main sections:

### 1. Initialization

```python
--8<-- "tutorials/sensors/reading_loadcell_data.py:1:40"
```

This section:

- Sets up constants and configuration parameters
- Defines the calibration matrix
- Creates a data logger for recording measurements
- Sets up a real-time loop for consistent timing
- Initializes the DephyLoadcellAmplifier with specified parameters

### 2. Main Loop

```python
--8<-- "tutorials/sensors/reading_loadcell_data.py:48:55"
```

The main loop:

1. Updates the loadcell to get the latest reading
2. Logs the time and current force/torque values
3. Updates the logger

## Loadcell Parameters

When initializing the DephyLoadcellAmplifier, several important parameters can be configured:

```python
--8<-- "tutorials/sensors/reading_loadcell_data.py:32:40"
```

### Parameter Details

1. **calibration_matrix** (np.array):
      - 6x6 matrix that converts raw sensor values to physical units
      - Specific to each loadcell and must be provided for accurate measurements
      - Obtained from manufacturer after calibration procedure

2. **tag** (str):
      - Unique identifier for the loadcell instance
      - Useful when using multiple sensors

3. **amp_gain** (float):
      - Amplifier gain setting
      - Typically 125 for the Dephy amplifier
      - Affects sensitivity and measurement range

4. **exc** (float):
      - Excitation voltage in volts
      - Typically 5V for the Dephy amplifier
      - Must match the hardware configuration

5. **bus** (int):
      - I2C bus number
      - Typically 1 on Raspberry Pi
      - Use `i2cdetect -y 1` to verify

6. **i2c_address** (int):
      - Device address on the I2C bus
      - Default is 102 (0x66 in hexadecimal)
      - Can be configured on some amplifiers

## Available Properties

The DephyLoadcellAmplifier provides six measurement properties:

1. **Forces** (fx, fy, fz):
      - Linear forces in Newtons (N)
      - Three orthogonal directions
      ```python
      loadcell.fx  # Force in X direction
      loadcell.fy  # Force in Y direction
      loadcell.fz  # Force in Z direction
      ```

2. **Moments** (mx, my, mz):
      - Torques/moments in Newton-meters (Nm)
      - Rotation around three orthogonal axes
      ```python
      loadcell.mx  # Moment around X axis
      loadcell.my  # Moment around Y axis
      loadcell.mz  # Moment around Z axis
      ```

## Running the Example

1. Navigate to the tutorial directory:
   ```bash
   cd tutorials/sensors
   ```

2. Run the script:
   ```bash
   python loadcell.py
   ```

3. Expected behavior:
      - Loadcell begins reading force/torque data continuously at 200Hz
      - Data is logged to `./logs/reading_loadcell_data.csv`
      - Force and moment values update as you apply loads to the sensor

## Common Issues

- **I2C Communication Errors**: Verify connections with `i2cdetect -y 1`
- **Permission Denied**: Add user to i2c group: `sudo usermod -a -G i2c $USER`
- **Incorrect Readings**: Check calibration matrix and amplifier settings
- **Noise in Measurements**: Verify grounding and power supply stability
- **Drift in Readings**: The loadcell might have to be re-calibrated by the manufacturer

## Calibration

The calibration matrix is crucial for accurate measurements. The provided matrix:

```python
LOADCELL_CALIBRATION_MATRIX = np.array([
    (-38.72600, -1817.74700, 9.84900, 43.37400, -44.54000, 1824.67000),
    # ... additional rows ...
])
```
Should be replaced with your specific loadcell's calibration values.

## Best Practices

1. **Before Each Use**

      - Verify all connections

2. **Maintenance**

      - Regular calibration checks
      - Clean connections
      - Monitor for drift

If you have any questions or need further assistance, please post on the [Open Source Leg community forum](https://opensourceleg.org/community).
