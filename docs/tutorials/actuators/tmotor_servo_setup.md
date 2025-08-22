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

# Set transmission queue length for optimal performance
sudo ifconfig can0 txqueuelen 1000
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
  - Torque constant (Kt): 0.115 Nm/A
  - Gear ratio: 9:1
  - Pole pairs: 21

- **AK10-9**:
  - Torque constant (Kt): 0.206 Nm/A
  - Gear ratio: 9:1
  - Pole pairs: 21

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
    motor_id=1,
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
