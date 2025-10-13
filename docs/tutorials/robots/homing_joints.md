# Homing OSL Joints

This tutorial demonstrates how to home the knee and ankle joints on the Open Source Leg (OSL) using the `opensourceleg` Python library. You’ll learn how to initialize actuators and encoders, configure the robot platform, and use the built-in `home()` method to calibrate the joints by driving them into physical stops and applying offsets.

## Overview

This example shows how to:

- Initialize `DephyActuator` instances for the knee and ankle joints
- Configure `AS5048B` magnetic absolute encoders via I2C
- Instantiate the `OpenSourceLeg` platform with the defined components
- Use the `home()` method to detect joint limits and assign calibrated offsets
- Use callback functions to execute code at the homed positions
- Log joint and encoder positions to the terminal in real-time

## Hardware Setup

- **Actuators**: Two Dephy Actuators are connected to `/dev/ttyACM0` and `/dev/ttyACM1`. These control the knee and ankle respectively.
- **Encoders**: Two AS5048B encoders are connected to I2C bus on the host controller.

For detailed setup instructions, refer to:

- [Actuator Setup Guide](https://github.com/neurobionics/opensourceleg/blob/main/tutorials/actuators/getting_started.md)
- [Sensor Setup Guide](https://github.com/neurobionics/opensourceleg/blob/main/tutorials/sensors/getting_started.md)

## Software Setup

Ensure the `opensourceleg` Python package is installed. Before running the full integration script, verify that each actuator and encoder is working individually. This helps isolate configuration issues.

## Code Structure

The [tutorial script](https://github.com/neurobionics/opensourceleg/blob/main/tutorials/robots/osl/homing_joints.py) is structured into seven key sections:

### 1. Configuration

```python
--8<-- "tutorials/robots/osl/homing_joints.py:1:14"
```

Key parameters:

- `FREQUENCY`: Control loop rate (200 Hz)
- `DT`: Timestep per loop cycle, defined as `1 / FREQUENCY`
- `GEAR_RATIO`: Compound gear ratio applied to each joint (≈41.5:1)

### 2. Real-time loop and Logger

```python
--8<-- "tutorials/robots/osl/homing_joints.py:16:22"
```

This section:

- Initializes a `Logger` instance to write homing data to the `./logs` directory
- Starts a `SoftRealtimeLoop` for accurate timing

### 3. Actuator Initialization

```python
--8<-- "tutorials/robots/osl/homing_joints.py:24:39"
```

This section:

- Instantiates two `DephyActuator` objects for the knee and ankle
- Assigns ports, frequency, gear ratio, and disables internal logging

### 4. Sensor Initialization

```python
--8<-- "tutorials/robots/osl/homing_joints.py:41:58"
```

This section:

- Instantiates two `AS5048B` encoders using the I2C interface
- Each encoder is tagged and assigned its A1/A2 pin configuration
- Zero position is temporarily set to 0

### 5. Using Callbacks for Homing Completion

The `home()` method also supports an optional `callback_functions` argument. This allows you to specify a list of functions (one per actuator) that will be called when each actuator finishes its homing routine. This is useful for custom notifications, logging, or triggering additional actions like zeroing joint encoders.

For example, to print a message when each joint completes homing:

```python
--8<-- "tutorials/robots/osl/homing_joints.py:60:73"
```

If you do not wish to use callbacks, you can omit the `callback_functions` argument or pass a list of `None` values.

### 6. Initialize OSL Platform

```python
--8<-- "tutorials/robots/osl/homing_joints.py:75:79"
```

This section:

- Creates an instance of `OpenSourceLeg` and passes in the actuator and sensor dictionaries

### 7. Run Homing Routine

```python
--8<-- "tutorials/robots/osl/homing_joints.py:81:90"
```

This section:

- Calls `osl.home()` with user-defined voltages, direction, and thresholds
- Each actuator rotates in a specified direction until one of the following is true:
  - Velocity falls below `0.001 rad/s`
  - Current exceeds `5000 mA`
- Once stopped, the motor is zeroed based on its position, and a calibrated offset is added (30° for ankle, 0° for knee)

### 8. Reset Torque and Start Logging

```python
--8<-- "tutorials/robots/osl/homing_joints.py:92:110"
```

This section:

- Switches both actuators to `CURRENT` control mode
- Sets joint torques to 0 so they can be moved freely
- Logs the following information continuously:
  - Elapsed time
  - Output positions of the knee and ankle (in degrees)
  - Raw encoder readings of the knee and ankle (in degrees)

## Running the Script

1. Navigate to the tutorial directory:

    ```bash
    cd tutorials/robots/osl
    ```

2. Run the script:

    ```bash
    python homing_joints.py
    ```

3. Expected behavior:

    - Both motors rotate slowly in a negative direction
    - Movement stops once physical limits are detected (via current or velocity thresholds)
    - Homing offsets are applied to calibrate the zero position
    - Control mode switches to current control, torque is zeroed
    - Joint and encoder positions print continuously in the terminal

> Note: You can use Ctrl+C to terminate the loop at any time.
