# Getting Started with Robots and the OSL

This guide introduces the `opensourceleg.robots` module and provides essential safety information for robot tutorials. To preface, the robot class is a combination of actuator and sensor instances that are implemented and organized for you. The class is adapted from the `OpenSourceLeg` class to provide a generalized framework for any robotics platform. The class used for non OSL applications is called `RobotBase`, and more documentation can be found [here](https://github.com/neurobionics/opensourceleg/blob/main/opensourceleg/robots/base.py).

## Prerequisites

- Raspberry Pi
- Python environment with `opensourceleg` package installed
- Open source leg or robot platform with sensors and/or actuators
- External power supply

## Hardware Setup
The hardware setup will be unique to each robot platform. If you are having trouble with your setup, please refer to the specific documentation regarding actuators and sensors below.

- [Actuator Setup Guide](https://github.com/neurobionics/opensourceleg/blob/main/tutorials/actuators/getting_started.md)
- [Sensor Setup Guide](https://github.com/neurobionics/opensourceleg/blob/main/tutorials/sensors/getting_started.md)

## Safety Guidelines

Please follow these guidelines when following the `opensourceleg.robots` tutorials.

### 1. Mechanical Safety

- Securely mount the OSL or robot platform before operation
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
2. Review the [API documentation](../../api/robots/robots.md)
3. Post questions on the [Open Source Leg community forum](https://opensourceleg.org/community)

## Next Steps

Choose a tutorial to get started:

- [Homing OSL Joints](homing_joints.md)

Each tutorial includes specific setup instructions and code examples.
