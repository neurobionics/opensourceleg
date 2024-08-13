# Actuators Library Guide

## Introduction

The actuators library supports multiple controllers for the Open-Source Leg Project, and an API as [base.py](./base.py) for quick development.

The documentation mainly focus on how to develop a custom **actuator module**. Structures for the example implementations ([`dephy.py`](./dephy.py), etc) can be found in Section Reference. 

## Customization Guide of Actuators Module

1. Import base module

    ```Python
    import opensourceleg.hardware.actuators.base as base
    ```

2. Customize the Modes of Actuators from `base.ActuatorMode`
    
    `VoltageMode`, `CurrentMode`, `PositionMode`, and `ImpedanceMode` are predefined for common use. 

    For example, when customizing `class VoltageMode`: 

    ```Python
    class VoltageMode(base.VoltageMode):
        def __init__(self, device: "DephyActpack") -> None:

            super().__init__(mode_pass, device=device)       # Inherit the steps from the template are suggested, as they connects to class Actuator
            # Should implement your own steps below
            pass
        

        def set_voltage(self, voltage_value: int) -> None:
            # Should implement your own steps below
            pass
    ```

    Or customizing from `class ActuatorMode` if new modes are needed:
    ```Python
    class TorqueMode(base.ActuatorMode)
        def __init__(self, device: "DephyActpack") -> None:

            super().__init__(mode_pass, device=device)       # Inherit the steps from the template are suggested, as they connects to class Actuator
            # Should implement your own steps below
            pass
        
        # Should implement your own steps below
        pass
    ```

3. Customize the actuator object from `base.Actuator`

    For example, when customizing `class VoltageMode`: 

    ```Python
    class Motor(base.Actuator):
    
        def __init__(
            self,
            name: str = "DephyActpack",
            port: str = "/dev/ttyACM0",
            baud_rate: int = 230400,
            frequency: int = 500,
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
            return f"DephyActpack[{self._name}]"
    
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
    
        def set_voltage(self, voltage_value: float):
            # Should implement your own steps below
            pass
    
        def set_current(
            self,
            current_value: float,
        ):
            # Should implement your own steps below
            pass
    
        # Should implement your own steps below
        pass
    
    ```

## Reference

### Structure of [base.py](./base.py)

```mermaid
classDiagram
  namespace Mode_Switch{
    class ControlModesMapping{
        POSITION
        VOLTAGE
        CURRENT
        IMPEDANCE
    }
    class ControlModesMeta{
        
    }
    class ControlModeBase{
        self._control_mode_map: ControlModesMapping = control_mode_map
        self._has_gains: bool = False
        self._gains: Any = None
        self._entry_callbacks: list[Callable[[], None]] = entry_callbacks
        self._exit_callbacks: list[Callable[[], None]] = exit_callbacks
        self._actuator: "ActuatorBase" = actuator
        self._max_command: Union[float, int] = max_command
        self._max_gains: ControlGains = max_gains
        enter()
        exit()
        transition()
        add_actuator()
        set_gains()
        set_command()
    }
    class ControlModesBase{

    }
  }
  class ActuatorBase{
        self._CONTROL_MODES: ControlModesBase = control_modes
        self._MOTOR_CONSTANTS: MotorConstants = motor_constants
        self._gear_ratio: float = gear_ratio
        self._actuator_name: str = actuator_name
        self._frequency: int = frequency
        self._data: Any = None
        self._mode: ControlModeBase = default_control_mode
        self._is_offline: bool = offline
        start()
        stop()
        update()
        set_control_mode()

  }
  namespace Data{
    class ControlGains{
        kp
        ki
        kd
        k
        b
        ff
    }
    class MotorConstants{
        MOTOR_COUNT_PER_REV
        NM_PER_AMP
        IMPEDANCE_A
        IMPEDANCE_C
        MAX_CASE_TEMPERATURE
        M_PER_SEC_SQUARED_ACCLSB
    }
  }
  
  ActuatorBase <.. ControlModesBase: self._CONTROL_MODES
  ActuatorBase <.. ControlGains: self._max_gains
  ControlModesMeta ..> ControlModeBase: ControlModesBase(metaclass=ControlModesMeta)
  ActuatorBase <.. MotorConstants: self._MOTOR_CONSTANTS
```

### [DephyActpack](./dephy.py)



### [Moteus Drivers](./moteus.py)

#### Known Issues

* Missing Joint Properties (e.g. `homing`)

* Multiple actuators sharing same cycle not supported, manual access to `MoteusInterface` Required
