# Using the AS5048B Encoder

This tutorial demonstrates how to use the AS5048B magnetic encoder with the Open Source Leg platform.

## Hardware Setup

1. Connect the AS5048B encoder to your Raspberry Pi's I2C pins
2. Make sure the encoder is powered on and the I2C pins are connected correctly
3. Note the encoder's address pin configuration (A1 and A2)

## Code Example

The complete code can be found in ```1:20:tutorials/sensors/encoder.py```.

## Encoder Parameters

When initializing the AS5048B encoder, several important parameters can be configured:

```python
encoder = AS5048B(
    bus=1,                    # I2C bus number
    A1_adr_pin=False,         # State of A1 address pin
    A2_adr_pin=True,          # State of A2 address pin
    name="encoder1",          # Unique identifier
    zero_position=0,          # Initial zero position
    enable_diagnostics=False, # Enable diagnostic checks
)
```

### Parameter Details

1. **bus** (int or str):
   - Specifies the I2C bus to use
   - Can be a number (e.g., `1`) or a path (e.g., `"/dev/i2c-1"`)
   - Default bus on Raspberry Pi is typically `1`

2. **A1_adr_pin** and **A2_adr_pin** (bool):
   - Controls the I2C address of the encoder
   - Corresponds to the physical address pins on the AS5048B
   - Used to connect multiple encoders on the same I2C bus
   - The encoder's address is calculated as: ```39:39:opensourceleg/sensors/encoder.py```
     ```python
     self.addr = AS5048B.I2C_BASE_ADR_7BIT | ((bool(A2_adr_pin)) << 1) | ((bool(A1_adr_pin)) << 0)
     ```
   - Base address is `0b1000000` (0x40)

3. **name** (str):
   - Identifier for the encoder instance
   - Useful when using multiple encoders

4. **zero_position** (int):
   - Sets the zero position reference
   - Value between 0 and 16383 (2¹⁴-1)
   - Affects the returned `position` value
   - ```179:186:opensourceleg/sensors/encoder.py``` shows the valid range

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

## Code Usage Example

```python
# Initialize and configure the encoder
encoder = AS5048B(
    bus=1,
    A1_adr_pin=False,
    A2_adr_pin=True,
    name="encoder1",
    zero_position=0,
    enable_diagnostics=False,
)

# Track the position data
encoder_logger.track_variable(lambda: encoder.position, "Encoder Position")

# Use context manager for safe handling
with encoder:
    for t in clock:
        encoder.update()  # Read latest data
        encoder_logger.info(f"Time: {t}; Encoder Angle: {encoder.position};")
```

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

### Multiple Encoders

The example below shows how to use multiple encoders simultaneously:

```python
knee_enc = AS5048B(
    name="knee",
    bus="/dev/i2c-1",
    A1_adr_pin=True,
    A2_adr_pin=False,
)

ankle_enc = AS5048B(
    name="ankle",
    bus="/dev/i2c-1",
    A1_adr_pin=False,
    A2_adr_pin=True,
)

with knee_enc, ankle_enc:
    knee_enc.update()
    ankle_enc.update()
    # Use both encoders...
```

## Common Issues

- **I2C Communication Errors**: Verify connections and run `i2cdetect -y 1`
- **Permission Denied**: Add user to i2c group: `sudo usermod -a -G i2c $USER`
- **Incorrect Readings**: Check encoder orientation and magnetic field strength
- **Unstable Values**: Check for mechanical vibration or electromagnetic interference
