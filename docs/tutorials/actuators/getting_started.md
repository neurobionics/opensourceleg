# Getting Started with Actuators

This guide introduces the `opensourceleg.actuators` module and provides essential safety information for all actuator tutorials.

## Prerequisites

- Raspberry Pi
- Python environment with `opensourceleg` package installed
- Supported actuator (Dephy actuator used in examples)
- External power supply

## Hardware Setup

1. Connect the actuator to your computer via USB (typically at `/dev/ttyACM0`)
2. Connect and verify the external power supply
3. Ensure proper mounting of the actuator before operation

## Safety Guidelines

Please follow these guidelines when following the `opensourceleg.actuators` tutorials.

### 1. Mechanical Safety

- Securely mount the actuator before operation
- Maintain clear space around moving parts
- Verify sufficient range of motion for commands
- Be aware of mechanical limits and stops

- Have a clear emergency stop plan
- Know how to quickly terminate scripts (`Ctrl+C`)
- Keep emergency stop button accessible

### 2. Electrical Safety

- Never exceed rated voltage/current limits
- Verify proper power supply connection
- Ensure proper grounding
- Check all electrical connections before powering on

### 3. Operational Safety

- Start with conservative values:

    - Low voltages in voltage control
    - Low currents in current control
    - Small steps in position control

- Continuously monitor sensor readings
- Watch for unexpected behavior:
    - Oscillations
    - Excessive current draw
    - Unusual sounds or vibrations
    - Unexpected resistance

### 4. Best Practices

- Verify all connections
- Check mounting security
- Clear workspace of obstacles
- Review emergency procedures

- Monitor system behavior
- Keep hands clear
- Be ready to terminate
- Log any unusual behavior

- Power down safely
- Document any issues
- Verify data logging
- Check actuator temperature

## Troubleshooting Guide

- Check device presence: `ls /dev/ttyACM*`
- Verify USB permissions, if you see `Permission denied` error, it is likely because the current user does not have permission to access the serial ports. You can add your user to the `dialout` group to fix this. If you are on a Linux system, you can run `sudo usermod -a -G dialout $USER` to add your user to the `dialout` group. Then you need to restart your system for the changes to take effect.

- Try physical reconnection:

    1.  Unplug USB
    2.  Wait a few seconds
    3.  Reconnect

- Verify power supply connection
- Check voltage levels
- Confirm power LED indicators
- Monitor current draw

## Getting Help

If you encounter any issues or need assistance:

1. Check the troubleshooting section in the specific tutorial
2. Review the [API documentation](../../api/actuators/actuators.md)
3. Post questions on the [Open Source Leg community forum](https://opensourceleg.org/community)

## Next Steps

Choose a tutorial to get started:

- [Reading Sensor Data](reading_sensor_data.md)
- [Commanding Voltage](commanding_voltage.md)
- [Commanding Current](commanding_current.md)
- [Commanding Position](commanding_position.md)
- [Commanding Impedance](commanding_impedance.md)

Each tutorial includes specific setup instructions and code examples.
