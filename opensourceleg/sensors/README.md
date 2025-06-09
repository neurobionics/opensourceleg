# [Sensor Module Customization](./base.py) Guide

## Overview

The sensor library contains a [`base.py`](./base.py), an abstract base for customizing your own sensor instance, and examples of applications such as [`imu.py`](./imu.py), [`encoder.py`](./encoder.py), [`adc.py`](./adc.py), etc.

## 1. Import base module

```python
  from opensourceleg.sensors.base import (
    SensorBase,
    SensorNotStreamingException,
    check_sensor_stream
  )
```

## 2. Initialize sensor class

```python
  class yourSensor(SensorBase):
    def __init__(
      self,
      tag="your_sensor",
      offline=False):

      """
      Add other variables here
      """

    def start(self):
    """ Initialize hardware connection and start sensor """
    self._streaming = True
    print(f"{self.tag} started.")

    def stop(self):
      """ Stop sensor and raise SensorNotStreamingException: If the sensor is not currently streaming."""
      self._streaming = False
      print(f"{self.tag} stopped.")

    @requirement
    def update(self):
    """ Read from sensor and do something with the data """
```

## 3. Check if sensor is streaming data

2.  Customize the sensor classes from `class SensorIMU`
    For example:

    ```Python
    class DephyIMU(base.SensorIMU):
    def __init__(self, Sensor: "DephyIMU") -> None:
        super.__init__()
        <!-- Should implement your own steps below -->

    def start_streaming(self):
        <!-- Should implement your own steps below -->

    def stop_streaming(self):
        <!-- Should implement your own steps below -->

    def update(self):
        <!-- Should implement your own steps below -->

    <!-- Should implement your own methods below -->
    ```

3.  Link it to your `class Actuator` in the actuator module

        For example:
        ```Python
        class DephyActuator(base.Actuator, device)

            <!-- Irrelevant methods / members neglected -->

            def update(self) -> None:
                super().update()

                if self.is_streaming:
                    self._data = self.read()
                    self.dephyIMU.update(self._data)

                <!-- Should implement your own steps below -->
        ```

    <!--

## Reference: Base Class Structure Diagram

````mermaid
classDiagram
  direction RL
  class ControlGains {
    kp: int
    ki: int
    kd: int
    K: int
    B: int
    ff: int
  }
  class MechanicalConstants{
    MOTOR_COUNT_PER_REV: float
    NM_PER_AMP: float
    IMPEDANCE_A: float
    IMPEDANCE_C: float
    MAX_CASE_TEMPERATURE: float
    M_PER_SEC_SQUARED_ACCLSB: float
    NM_PER_MILLIAMP()
    RAD_PER_COUNT()
    RAD_PER_DEG()
    RAD_PER_SEC_GYROLSB()
    NM_PER_RAD_TO_K()
    NM_S_PER_RAD_TO_B()
  }
  class ActuatorMode{
    _control_mode
    _has_gains
    _gains
    _entry_callback
    _exit_callback
    has_gains()
    mode()
    __init__()
  }
  class VoltageMode{
    set_voltage()
  }
  class CurrentMode{
    set_current()
    set_gains()
  }
  class PositionMode{
    set_position()
    set_gains()
  }
  class ImpedanceMode{
    set_position()
    set_gains()
  }
 class Actuator{
    __init__(Gains, MecheSpecs, *args, **kwargs)
    _gains: Any = Gains
    _MecheConsts: MechanicalConstants = MecheSpecs
    _mode: Any = None
    start()
    stop()
    update()
    set_mode()
    set_voltage()
    set_current()
    set_motor_position()
 }
VoltageMode <-- ActuatorMode
CurrentMode <-- ActuatorMode
PositionMode <-- ActuatorMode
ImpedanceMode <-- ActuatorMode
ControlGains ..> Actuator
MechanicalConstants ..> Actuator

Actuator <.. VoltageMode
Actuator <.. CurrentMode
Actuator <.. PositionMode
Actuator <.. ImpedanceMode
``` -->
````
