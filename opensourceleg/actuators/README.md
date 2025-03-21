# Actuators Library Guide

## Introduction

The actuators library supports multiple controllers for the Open-Source Leg Project, and an API as [base.py](./base.py) for quick development.

The documentation mainly focus on how to develop a custom **actuator module**. Structures for the example implementations ([`dephy.py`](./dephy.py), etc) can be found in Reference.

## Customization Guide of Actuators Module

1. Import base module

   ```Python
   import opensourceleg.actuators.base as base
   ```

2. Customize the Modes of Actuators from `base.ActuatorMode`

   `VoltageMode`, `CurrentMode`, `PositionMode`, and `ImpedanceMode` are predefined for common use.

   For example, when customizing `class VoltageMode`:

   ```Python
   class VoltageMode(base.VoltageMode):
       def __init__(self, device: "DephyActuator") -> None:

           super().__init__(mode_pass, device=device)       # Inherit the steps from the template are suggested, as they connects to class Actuator
           # Should implement your own steps below
           pass


       def set_voltage(self, voltage_value: int) -> None:
           # Should implement your own steps below
           pass
   ```

   Or customizing from `class ControlModeBase` if new modes are needed:

   ```Python
   class TorqueMode(base.ControlModeBase)
       def __init__(self, device: "DephyActuator") -> None:

           super().__init__(mode_pass, device=device)       # Inherit the steps from the template are suggested, as they connects to class Actuator
           # Should implement your own steps below
           pass

       # Should implement your own steps below
       pass
   ```

   Index from `class ControlModesBase` is needed if constructing the Actuator Object from `base.ActuatorBase`. For example:

   ```Python
   @dataclass(init=False)
   class ActuatorControlModes(ControlModesBase):

   def __init__(self, actuator: "Actuator") -> None:

       self.VOLTAGE = VoltageMode(actuator=actuator)
       self.CURRENT = CurrentMode(actuator=actuator)
       self.POSITION = PositionMode(actuator=actuator)
       self.IMPEDANCE = ImpedanceMode(actuator=actuator)
   ```

3. Customize the actuator object from `base.ActuatorBase`

   ```Python
   class Motor(base.ActuatorBase):

       def __init__(
           self,
           tag: str = "Motor1"
           port: str = "/dev/ttM0",
           baud_rate: int = 230400,
           frequency: int = 200,
           logger: Logger = Logger(),
           debug_level: int = 0,
           dephy_log: bool = False,
           *args,
           **kwargs,
       ) -> None:
           base.Actuator.__init__(
               self,
               Gains=base.ControlGains(),
               MecheSpecs=base.MecheConsts(),
           )                                                               # Inherit the steps from the template are suggested, as they offer common member definitions

           # Should implement your own steps below
           pass

       def __repr__(self) -> str:
           return f"DephyActuator[{self._name}]"

       def start(self) -> None:
           super().start()
           # Should implement your own steps below
           pass

       def stop(self) -> None:
           super().stop()
           # Should implement your own steps below
           pass

       def update(self) -> None:
           super().update()
           # Should implement your own steps below
           pass

       def set_mode(self, mode: base.ActuatorMode) -> None:
           # Should implement your own steps below
           pass

       def set_motor_voltage(self, voltage_value: float):
           # Should implement your own steps below
           pass

       def set_motor_current(
           self,
           current_value: float,
       ):
           # Should implement your own steps below
           pass

       # Should implement your own steps below
       pass

   ```

## Reference

### Structure of [base.py](./base.py) with Example Applications

![](./images/Class%20Diagram%20Base%20Lib.svg)

### [DephyActpack](./dephy.py) Support

#### Known Issues

- Support for DephyActpack new firmware not finished

### [Moteus Controller](./moteus.py) Support

#### Known Issues [Check [Issue Page](https://github.com/neurobionics/opensourceleg/issues) for Details]

- Missing Joint Properties (e.g. `homing`)

- [Multiple actuators sharing same cycle not supported, manual access to `MoteusInterface` Required](https://github.com/neurobionics/opensourceleg/issues)
