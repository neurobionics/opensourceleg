# TMotor Servo Mode Setup Guide

## Prerequisites

Before using TMotor actuators in servo mode, you need to configure the CAN interface on your system.

## CAN Interface Configuration

### Initial Setup

Run the following commands to configure the CAN interface for TMotor servo mode:

```bash
# Bring down the CAN interface
sudo /sbin/ip link set can0 down

# Configure CAN interface with 1MHz bitrate
sudo /sbin/ip link set can0 up type can bitrate 1000000
```

### Verification

To verify that the CAN interface is properly configured:

```bash
# Check CAN interface status
ip link show can0

# Monitor CAN traffic (optional)
candump can0
```

## Motor Configuration

### Supported Motors

- **AK80-9**:
  - Torque constant (Kt): 0.095 Nm/A (driver-compatible value for Delta winding)
  - Gear ratio: 9:1
  - Pole pairs: 21

- **AK10-9**:
  - Torque constant (Kt): 0.206 Nm/A
  - Gear ratio: 9:1
  - Pole pairs: 21

### Understanding Torque Constant (Kt) for Delta-Wound Motors

#### Background

The TMotor actuators use **Delta-wound motors** with CubeMars FOC drivers. There is a discrepancy between different torque constant (Kt) values:

- **0.095 Nm/A** - Driver-compatible Kt (what you should use in your code for AK80-9)
- **0.14 Nm/A** - CubeMars specification (power-invariant transform)
- **0.163 Nm/A** - Physical Kt from Back-EMF testing (amplitude-invariant transform)

#### The Root Cause: Delta vs Wye Windings

The CubeMars FOC driver defines current as follows:

> "The definition of CubeMars' current: Amplitude of the line currents, which using the amplitude-invariant Clarke-Park transform is equivalent to the q-axis current."

**This definition only holds true for Wye-wound motors**, where line current equals phase current. However, these motors use **Delta winding**, where:

```
Line Current (I_l) = √3 × Phase Current (I_φ)
```

The standard amplitude-invariant Clarke-Park transform is built on phase current (I_φ), but CubeMars sets their I_q equal to the line current amplitude. This causes the I_q value in their FOC driver to be inflated by a factor of √3.

#### Mathematical Analysis

For a Delta winding motor:

1. **Physical torque equation** (from Back-EMF testing):
   ```
   τ = Kt^φ × I^φ
   where Kt^φ = 0.163 Nm/A (true physical constant)
   ```

2. **Driver torque equation** (what CubeMars uses):
   ```
   τ = Kt^drv × I_q^drv
   where I_q^drv = I_l = √3 × I^φ
   ```

3. **Derivation of the relationship**:
   ```
   Kt^drv × (√3 × I^φ) = Kt^φ × I^φ
   Kt^φ = √3 × Kt^drv
   √3 × 0.095 ≈ 0.165 Nm/A ≈ 0.163 Nm/A ✓
   ```

This calculation matches the Back-EMF measurement.

#### Why Use 0.095 Nm/A?

**Use Kt = 0.095 Nm/A** when working with CubeMars drivers and Delta-wound motors because:

1. It matches the driver's current definition (line current amplitude)
2. The FOC driver settings cannot be easily modified
3. It provides the correct torque calculation when combined with the driver's current interpretation

#### Practical Implications

Due to the CubeMars FOC driver settings:

- The driver's I_q value is √3 times larger than the true phase current
- The maximum current limit is also √3 times higher than the physical phase current
- The motor can be commanded with current values that are √3 times higher than would be expected from the physical Kt (0.163 Nm/A)

#### Summary of Three Kt Values

| Kt Value | Transform Type | Usage |
|----------|----------------|-------|
| 0.163 Nm/A | Amplitude-invariant (I_q = phase current) | Physical/theoretical analysis |
| 0.14 Nm/A | Power-invariant (I_q^pwr = √(3/2) × I^φ) | CubeMars specification |
| 0.095 Nm/A | Driver-compatible (I_q = line current) | **Use this in your code** |

The three values are related by:
```
Kt^φ = √3 × Kt^drv ≈ 1.732 × 0.095 ≈ 0.165 Nm/A
Kt^pwr = √2 × Kt^drv ≈ 1.414 × 0.095 ≈ 0.134 Nm/A
```

### Control Modes

The TMotor servo mode supports the following control modes:

1. **Position Control** (Mode 4): Control motor position in degrees
2. **Velocity Control** (Mode 3): Control motor velocity in ERPM
3. **Current Control** (Mode 1): Control motor current in Amps
4. **Idle Mode** (Mode 7): Motor idle state

## Usage Example

```python
from opensourceleg.actuators.tmotor import TMotorServoActuator

# Initialize motor
motor = TMotorServoActuator(
    motor_id=104, #the default CAN ID is 104
    gear_ratio=9.0,
    motor_type="AK80-9"
)

# Start motor
motor.start()

# Home the motor
motor.home()

# Set control mode
motor.set_control_mode(mode=CONTROL_MODES.POSITION)

# Command position
motor.set_motor_position(position_rad=1.57)  # 90 degrees

# Stop motor
motor.stop()
```

## Troubleshooting

### CAN Bus Initialization Failed

If you encounter a "CAN bus initialization failed" error:

1. Ensure the CAN interface is properly configured (see CAN Interface Configuration above)
2. Check that you have the necessary permissions (may require sudo)
3. Verify the CAN hardware is connected
4. Check for conflicting CAN bus processes

### Permission Denied

If you get permission errors when configuring the CAN interface:

```bash
# Add your user to the dialout group
sudo usermod -a -G dialout $USER

# Logout and login again for changes to take effect
```

### CAN Interface Not Found

If `can0` is not found:

1. Check if CAN drivers are loaded:
   ```bash
   lsmod | grep can
   ```

2. Load CAN drivers if needed:
   ```bash
   sudo modprobe can
   sudo modprobe can_raw
   sudo modprobe vcan  # For virtual CAN (testing)
   ```

## Notes

- The CAN interface configuration must be done before initializing the TMotorServoActuator
- The interface configuration is not persistent across reboots
- For production use, consider adding the configuration to system startup scripts
- Always ensure proper grounding and shielding for CAN communication
- Unlike some motors, in this type the current reading only becomes negative when the velocity and torque directions are opposite.
