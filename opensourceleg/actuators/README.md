# Actuators Module Customization Guide

## Overview

The actuators library supports multiple controllers for the Open-Source Leg Project, and an API as [base.py](./base.py) for quick development.

The documentation mainly focus on how to develop a custom **actuator module**. Structures for the example implementations ([`dephy.py`](./dephy.py), etc) can be found in Reference.

---

## 1. Import base module

```python
from opensourceleg.actuators.base import (
    CONTROL_MODE_CONFIGS,
    CONTROL_MODES,
    MOTOR_CONSTANTS,
    ActuatorBase,
    ControlGains,
    ControlModeConfig,
)
```

---

## 2. Configuring and Customizing Predefined Control Modes

Actuator control modes (IDLE, POSITION, CURRENT, VOLTAGE, IMPEDANCE, VELOCITY, TORQUE) are predefined using the `CONTROL_MODES` enum. These can be configured per actuator using `ControlModeConfig` and `CONTROL_MODE_CONFIGS`. Each mode can specify entry/exit callbacks and gain limits.

**Example: Define and Configure Modes for Your Actuator**

```python
def entry_position_mode(actuator):
    # You can execute any actions here, such as logging, hardware setup, or custom checks.
    print(f"[{actuator.tag}] Entering POSITION mode.")

def exit_position_mode(actuator):
    print(f"[{actuator.tag}] Exiting POSITION mode.")

MY_CONTROL_MODE_CONFIGS = CONTROL_MODE_CONFIGS(
    POSITION=ControlModeConfig(
        entry_callback=entry_position_mode,
        exit_callback=exit_position_mode,
        has_gains=True,
        max_gains=ControlGains(kp=100, ki=10, kd=1, k=0, b=0, ff=0),
    ),
    # Add other modes similarly...
)
```

**Link these configs to your actuator class using the `_CONTROL_MODE_CONFIGS` property.**

---

## 2. Defining and Integrating Custom Control Modes

> **Note:** > **Python's `enum.Enum` does _not_ support extension by subclassing.**
> To add custom modes, define a new Enum containing all the original modes plus your custom ones.

**Example: Adding a CUSTOM Mode**

```python
 from enum import Enum

 class MY_CONTROL_MODES(Enum):
     IDLE = -1
     POSITION = 0
     VOLTAGE = 1
     CURRENT = 2
     IMPEDANCE = 3
     VELOCITY = 4
     TORQUE = 5
     CUSTOM = 100  # Your custom mode -- assign a unique integer
```

Make sure your control logic, configs, and actuator class use your custom Enum everywhere a mode is referenced.

```python
def entry_custom_mode(actuator):
    print(f"[{actuator.tag}] Entering CUSTOM mode.")

def exit_custom_mode(actuator):
    print(f"[{actuator.tag}] Exiting CUSTOM mode.")

MY_CONTROL_MODE_CONFIGS = CONTROL_MODE_CONFIGS(
    # ... other modes ...
    CUSTOM=ControlModeConfig(
        entry_callback=entry_custom_mode,
        exit_callback=exit_custom_mode,
        has_gains=False,
        max_gains=None,
    ),
)
```

---

## 3. Implementing a Custom Actuator Class with `ActuatorBase`

To add a new actuator, **subclass `ActuatorBase`** and implement all required abstract methods and properties. The class should also expose the `_CONTROL_MODE_CONFIGS` property that returns your mode configuration.

**Example: Minimal Template for a Custom Actuator**

```python
# 1. Define motor constants for your actuator
MY_MOTOR_CONSTANTS = MOTOR_CONSTANTS(
    MOTOR_COUNT_PER_REV=4096,
    NM_PER_AMP=0.05,
    NM_PER_RAD_TO_K=0.001,
    NM_S_PER_RAD_TO_B=0.0001,
    MAX_CASE_TEMPERATURE=75.0,
    MAX_WINDING_TEMPERATURE=100.0
)

# 2. Define mode configs as in the previous section and create a class

class MyActuator(ActuatorBase):
    """
    Example actuator implementation.

    Example usage:
        >>> actuator = MyActuator(tag="motor1", port="/dev/ttyUSB0", gear_ratio=100.0)
        >>> actuator.start()
        >>> actuator.set_motor_voltage(1500)
        >>> print(f"Motor position: {actuator.motor_position:.2f} rad")
        >>> actuator.stop()
    """
    def __init__(
        self,
        tag: str = "MyActuator",
        port: str = "/dev/ttyUSB0",
        gear_ratio: float = 100.0,
        motor_constants: MOTOR_CONSTANTS = MY_MOTOR_CONSTANTS,
        frequency: int = 500,
    ):
        super().__init__(
            tag=tag,
            gear_ratio=gear_ratio,
            motor_constants=motor_constants,
            frequency=frequency,
        )
        # Implement your own hardware initialization steps here

    @property
    def _CONTROL_MODE_CONFIGS(self):
        return MY_CONTROL_MODE_CONFIGS
    def start(self):
        # Implement your own steps to start communication with the actuator hardware
        pass
    def stop(self):
        # Implement your own steps to safely stop and disconnect hardware
        pass
    def update(self):
        # Implement your own steps to update actuator state from hardware
        pass
    def set_motor_voltage(self, value: float):
        # Implement your own steps as per the hardware
        pass
    @property
    def motor_position(self) -> float:
        # Implement your own steps to return position from hardware
        pass
    # Add other required methods and properties as needed for your application
```

---

## 4. Restricting Methods by Control Mode (`MethodWithRequiredModes` and `@requires`)

The actuators framework allows you to restrict which methods can be called in each control mode.
This is implemented using the `@requires` decorator and the `MethodWithRequiredModes` protocol.

### How It Works

- **@requires decorator**: Add `@requires(CONTROL_MODES.POSITION, CONTROL_MODES.TORQUE)` above a method to specify that it is only allowed in those modes.
- **Enforcement**: The base class automatically enforces these mode restrictions at runtime.
- **Protocol**: Any method with a `_required_modes` attribute is recognized as a `MethodWithRequiredModes`.

### Example Usage

```python
from opensourceleg.actuators.base import requires, CONTROL_MODES

class MyActuator(ActuatorBase):
    @requires(CONTROL_MODES.POSITION)
    def set_motor_position(self, value):
        # Implementation here
        pass

    @requires(CONTROL_MODES.CURRENT)
    def set_motor_current(self, value):
        # Implementation here
        pass
```

If you try to call `set_motor_position` when the actuator is not in `POSITION` mode, a `ControlModeException` will be raised.

---

## 5. Reference

### Structure of [base.py](./base.py) with Example Applications

![](./images/Class%20Diagram%20Base%20Lib.svg)

[DephyActpack](./dephy.py) Support

[Moteus Controller](./moteus.py) Support

[TMotor](./tmotor.py) Support

### Issues:

Found a bug or have a suggestion? Please [open an issue](https://github.com/neurobionics/opensourceleg/issues).
