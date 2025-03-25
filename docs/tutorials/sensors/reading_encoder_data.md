# Using the AS5048B Encoder

This tutorial demonstrates how to use the AS5048B magnetic encoder with the Open Source Leg platform.

## Hardware Setup

1. Connect the AS5048B encoder to your Raspberry Pi's I2C pins
2. Make sure the encoder is powered on and the I2C pins are connected correctly
3. Note the encoder's address pin configuration (A1 and A2)

This example shows how to:

- Initialize and configure an AS5048B encoder
- Read angular position data

## Code Structure

The [tutorial script](https://github.com/neurobionics/opensourceleg/blob/main/tutorials/sensors/encoder.py) is organized into several main sections:

### 1. Initialization

```python
--8<-- "tutorials/sensors/encoder.py:1:21"
```

This section:
   - Creates a data logger for recording measurements
   - Sets up a real-time loop for consistent timing
   - Initializes the AS5048B encoder with specified parameters

### 2. Main Loop

```python
--8<-- "tutorials/sensors/encoder.py:24:28"
```

The main loop:
1. Updates the encoder to get the latest reading
2. Logs the time and current encoder angle
3. Updates the logger

## Encoder Parameters

When initializing the AS5048B encoder, several important parameters can be configured:

```python
--8<-- "tutorials/sensors/encoder.py:14:19"
```

### Parameter Details

1. **tag** (str):
      - Unique identifier for the encoder instance
      - Useful when using multiple encoders

2. **bus** (str):
      - Specifies the I2C bus to use
      - Should be a path (e.g., `"/dev/i2c-1"`)
      - Default bus on Raspberry Pi is typically `1`

3. **A1_adr_pin** and **A2_adr_pin** (bool):
      - Controls the I2C address of the encoder
      - Corresponds to the physical address pins on the AS5048B
      - Used to connect multiple encoders on the same I2C bus
      - The encoder's address is calculated as:
      ```python
      --8<-- "opensourceleg/sensors/encoder.py:39:39"
      ```
      - Base address is `0b1000000` (0x40)

4. **zero_position** (int):
      - Sets the zero position reference
      - Value between 0 and 16383 (2¹⁴-1)
      - Affects the returned `position` value

5. **enable_diagnostics** (bool):
      - When enabled, performs additional checks during updates
      - Verifies data validity and magnetic field strength
      - May raise exceptions for invalid data or show warnings

## Available Properties

The AS5048B encoder provides several useful properties:

1. **position**:
      - Current angular position in radians
      - Calculated from raw encoder output with proper scaling

2. **velocity**:
      - Angular velocity in radians per second
      - Calculated from consecutive position readings

3. **abs_ang**:
      - Absolute angle tracking multiple rotations
      - Keeps track of full rotations for continuous tracking

4. **zero_position**:
      - Gets/sets the zero position offset
      - Use `set_zero_position()` method to calculate midpoint automatically

5. **Diagnostic Properties**:
      - `diag_compH`: High magnetic field compensation
      - `diag_compL`: Low magnetic field compensation
      - `diag_COF`: CORDIC overflow flag
      - `diag_OCF`: Offset compensation finished flag

## Running the Example

1. Navigate to the tutorial directory:
   ```bash
   cd tutorials/sensors
   ```

2. Run the script:
   ```bash
   python encoder.py
   ```

3. Expected behavior:
      - Encoder begins reading angular position continuously at 1000Hz
      - Data is logged to `./logs/reading_encoder_data.csv`
      - Position values will change as you rotate the encoder

## Advanced Usage

### Setting Zero Position

You can set the zero position in two ways:

1. **Manual Setting**:
   ```python
   encoder.zero_position = new_zero_value  # Sets specific zero offset
   ```

2. **Interactive Setting**:
   ```python
   encoder.set_zero_position()  # Interactive process to set midpoint
   ```
   This method prompts you to position the joint at minimum and maximum angles, then calculates the midpoint.
   Read more about this method [here](https://github.com/neurobionics/opensourceleg/blob/main/opensourceleg/sensors/encoder.py#283-300).

## Common Issues

- **I2C Communication Errors**: Verify connections and run `i2cdetect -y 1`
- **Permission Denied**: Add user to i2c group: `sudo usermod -a -G i2c $USER`
- **Incorrect Readings**: Check encoder orientation and magnetic field strength
- **Unstable Values**: Check for mechanical vibration or electromagnetic interference

If you have any questions or need further assistance, please post on the [Open Source Leg community forum](https://opensourceleg.org/community).
