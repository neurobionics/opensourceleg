# Using the Dephy Loadcell Amplifier

This tutorial demonstrates how to use the Dephy Loadcell Amplifier with the Open Source Leg platform to measure forces and moments.

## Hardware Setup

1. Connect the Dephy Loadcell Amplifier via I2C
2. Verify proper power supply connections
3. Ensure proper grounding
4. Mount the loadcell securely

## Code Example

The complete code can be found in ```1:63:tutorials/sensors/loadcell.py```. Here's how to use the loadcell:

## Code Explanation

Let's break down the key components:

1. **Configuration and Constants** (```1:9:tutorials/sensors/loadcell.py```):
   ```python
   FREQUENCY = 200  # 200Hz sampling rate
   DT = 1 / FREQUENCY
   ```

2. **Calibration Matrix** (```10:23:tutorials/sensors/loadcell.py```):
   ```python
   LOADCELL_CALIBRATION_MATRIX = np.array([
       (-38.72600, -1817.74700, 9.84900, 43.37400, -44.54000, 1824.67000),
       (-8.61600, 1041.14900, 18.86100, -2098.82200, 31.79400, 1058.6230),
       # ... additional rows ...
   ])
   ```
   This matrix converts raw sensor readings to calibrated force/torque measurements.

3. **Logger and Device Setup** (```25:35:tutorials/sensors/loadcell.py```):
   ```python
   loadcell = DephyLoadcellAmplifier(
       calibration_matrix=LOADCELL_CALIBRATION_MATRIX,
       amp_gain=125,
       exc=5,
       bus=1,
       i2c_address=102,
   )
   ```
   Key parameters:
   - `amp_gain`: Amplifier gain (typically 125)
   - `exc`: Excitation voltage (5V)
   - `bus`: I2C bus number
   - `i2c_address`: Device address on I2C bus

4. **Data Tracking** (```37:44:tutorials/sensors/loadcell.py```):
   - Tracks six measurements:
     - Forces (Fx, Fy, Fz)
     - Moments (Mx, My, Mz)

5. **Main Loop** (```46:63:tutorials/sensors/loadcell.py```):
   - Uses context manager for safe resource handling
   - Continuously reads and logs force/torque data
   - Updates at specified frequency

## Expected Output

The script outputs six values:
- Forces (N): Fx, Fy, Fz
- Moments (Nm): Mx, My, Mz

## Common Issues

1. **I2C Communication Errors**
   - Verify connections:
     ```bash
     i2cdetect -y 1
     ```
   - Check address matches configuration
   - Verify proper wiring

2. **Incorrect Readings**
   - Verify calibration matrix
   - Check amplifier gain setting
   - Ensure proper excitation voltage
   - Zero/tare the loadcell when unloaded

3. **Noise in Measurements**
   - Check grounding
   - Verify power supply stability
   - Consider adding mechanical isolation
   - Implement digital filtering

## Advanced Usage

### Calibration

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
   - Zero the loadcell when unloaded
   - Verify all connections
   - Check mounting security

2. **During Operation**
   - Monitor for overload conditions
   - Watch for temperature effects
   - Log any anomalies

3. **Maintenance**
   - Regular calibration checks
   - Clean connections
   - Monitor for drift

For more detailed information, refer to:
- [Loadcell API Documentation](../../api/sensors/loadcell.md)
- [Dephy Documentation](https://dephy.com/documentation)
