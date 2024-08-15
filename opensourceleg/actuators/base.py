from typing import Any, Callable, List, Union

from abc import ABC, abstractmethod
from ctypes import c_int
from dataclasses import dataclass
from enum import Enum

import numpy as np

from opensourceleg.actuators.exceptions import VoltageModeMissingException
from opensourceleg.logging.logger import LOGGER


class ControlModesMapping(Enum):
    """Control Modes Dictionary

    Attributes
    ----------
    VOLTAGE (c_int): Voltage control mode
    CURRENT (c_int): Current control mode
    POSITION (c_int): Position control mode
    IMPEDANCE (c_int): Impedance control mode

    Extending
    ---------
    Adding new modes to the control mode dictionary:
        Access opensourceleg.actuators.base.ControlModesMapping and add the new mode

    """

    POSITION = c_int(0), "position"
    VOLTAGE = c_int(1), "voltage"
    CURRENT = c_int(2), "current"
    IMPEDANCE = c_int(3), "impedance"
    VELOCITY = c_int(4), "velocity"
    TORQUE = c_int(5), "torque"
    IDLE = c_int(6), "IDLE"

    def __new__(cls, c_int_value, str_value):
        obj = object.__new__(cls)
        obj._value_ = c_int_value
        obj.str_value = str_value
        return obj

    @property
    def flag(self) -> c_int:
        return self._value_

    def __int__(self):
        return self.value.value

    def __str__(self):
        return str.upper(self.str_value)


class ControlModesMeta(type):
    def __init__(cls, name, bases, attrs):
        # TODO: Enforce a default mode for all actuators, this could be voltage mode or a separate idle mode
        # if bases and "VOLTAGE" not in attrs:
        #     print(f"Class: {cls}")
        #     print(f"Name: {name}")
        #     print(f"Bases: {bases}")
        #     print(f"Attrs: {attrs}")
        #     raise VoltageModeMissingException(cls.__name__)
        super().__init__(name, bases, attrs)


class ControlModesBase(metaclass=ControlModesMeta):
    pass


@dataclass
class ControlGains:
    """Dataclass for controller gains

    Attributes
    ----------
    kp (int): Proportional gain
    ki (int): Integral gain
    kd (int): Derivative gain
    k (int): Stiffness of the impedance controller
    b (int): Damping of the impedance controller
    ff (int): Feedforward gain
    """

    kp: int = 0
    ki: int = 0
    kd: int = 0
    k: int = 0
    b: int = 0
    ff: int = 0

    def __repr__(self) -> str:
        return f"kp={self.kp}, ki={self.ki}, kd={self.kd}, k={self.k}, b={self.b}, ff={self.ff}"


@dataclass
class MotorConstants:
    """Dataclass for mechanical constants

    Attributes
    ----------
    MOTOR_COUNT_PER_REV: float = 16384
    NM_PER_AMP: float = 0.1133
    IMPEDANCE_A: float = 0.00028444
    IMPEDANCE_C: float = 0.0007812
    MAX_CASE_TEMPERATURE: float = 80
    M_PER_SEC_SQUARED_ACCLSB: float = 9.80665 / 8192

    Properties
    ----------
    NM_PER_MILLIAMP(self): float = NM_PER_AMP / 1000

    RAD_PER_COUNT(self): float = return 2 * pi / MOTOR_COUNT_PER_REV

    RAD_PER_DEG(self): float  = pi / 180

    RAD_PER_SEC_GYROLSB(self): float = pi / 180 / 32.8

    NM_PER_RAD_TO_K(self): float = RAD_PER_COUNT / IMPEDANCE_C * 1e3 / NM_PER_AMP

    NM_S_PER_RAD_TO_B(self): float = RAD_PER_DEG / IMPEDANCE_A * 1e3 / NM_PER_AMP
    """

    def __repr__(self) -> str:
        return f"MotorConstants"

    MOTOR_COUNT_PER_REV: float = 16384
    NM_PER_AMP: float = 0.1133
    IMPEDANCE_A: float = 0.00028444
    IMPEDANCE_C: float = 0.0007812
    MAX_CASE_TEMPERATURE: float = 80
    MAX_WINDING_TEMPERATURE: float = 110
    M_PER_SEC_SQUARED_ACCLSB: float = 9.80665 / 8192

    # Should be only containing the members above. Others directly implemented in the Actuators class
    @property
    def NM_PER_MILLIAMP(self) -> float:
        return self.NM_PER_AMP / 1000

    @property
    def RAD_PER_COUNT(self) -> float:
        return 2 * np.pi / self.MOTOR_COUNT_PER_REV

    @property
    def RAD_PER_DEG(self) -> float:
        return np.pi / 180

    @property
    def RAD_PER_SEC_GYROLSB(self) -> float:
        return np.pi / 180 / 32.8

    @property
    def NM_PER_RAD_TO_K(self) -> float:
        return self.RAD_PER_COUNT / self.IMPEDANCE_C * 1e3 / self.NM_PER_AMP

    @property
    def NM_S_PER_RAD_TO_B(self) -> float:
        return self.RAD_PER_DEG / self.IMPEDANCE_A * 1e3 / self.NM_PER_AMP

    def set_max_winding_temperature(self, temperature: float) -> None:
        self.MAX_WINDING_TEMPERATURE = temperature

    def set_max_case_temperature(self, temperature: float) -> None:
        self.MAX_CASE_TEMPERATURE = temperature


class ControlModeBase(ABC):
    """Base Class for new actuator mode definition

    Args
    ----
    control_mode_index (c_int): Mode pass / flag for the actuator
    device (Actuator): Actuator instance

    Properties
    ----------
    mode (c_int): Control Mode
    has_gains (bool): Whether the mode has gains

    Methods
    -------
    enter() -> None:

    Calls the entry callback

    exit() → None:

    Calls the exit callback

    transition(
        to_mode: opensourceleg.hardware.actuators.base.ControlModeBase
    ) → None:

    Transition to another mode. Calls the exit callback of the current mode and the entry callback of the new mode

    """

    def __init__(
        self,
        control_mode_map: ControlModesMapping,
        actuator: Union["ActuatorBase", None] = None,
        entry_callbacks: list[Callable[[], None]] = [lambda: None],
        exit_callbacks: list[Callable[[], None]] = [lambda: None],
        max_gains: ControlGains = None,
    ) -> None:
        self._control_mode_map: ControlModesMapping = control_mode_map
        self._has_gains: bool = False
        self._gains: Any = None
        self._entry_callbacks: list[Callable[[], None]] = entry_callbacks
        self._exit_callbacks: list[Callable[[], None]] = exit_callbacks
        self._actuator: "ActuatorBase" = actuator
        self._max_gains: ControlGains = max_gains

    def __eq__(self, __o: object) -> bool:
        if isinstance(__o, ControlModeBase):
            return self.index == __o.index
        return False

    def __str__(self) -> str:
        return str(object=self.index)

    def __repr__(self) -> str:
        return f"ControlModeBase[{self.name}]"

    def enter(self) -> None:
        """
        Calls the entry callback
        """
        for _callback in self._entry_callbacks:
            _callback()

    def exit(self) -> None:
        """
        Calls the exit callback
        """
        for _callback in self._exit_callbacks:
            _callback()

    def transition(self, to_mode: "ControlModeBase") -> None:
        """
        Transition to another mode. Calls the exit callback of the current mode
        and the entry callback of the new mode.

        Args:
            to_mode (ControlModeBase): Mode to transition to
        """
        self.exit()
        to_mode.enter()

    def set_actuator(self, actuator: "ActuatorBase") -> None:
        self._actuator = actuator

    @abstractmethod
    def set_gains(self, gains: ControlGains) -> None:
        """set_gains method for the control mode.
        Please use the super method only if applicable to the child class if not override this method

        Args:
            Gains (ControlGains): control gains applied
        """
        if self.max_gains is not None:
            assert 0 <= gains.kp <= self._max_gains.kp
            assert 0 <= gains.ki <= self._max_gains.ki
            assert 0 <= gains.kd <= self._max_gains.kd
            assert 0 <= gains.k <= self._max_gains.k
            assert 0 <= gains.b <= self._max_gains.b
            assert 0 <= gains.ff <= self._max_gains.ff

        self._gains = gains
        self._has_gains = True

    @abstractmethod
    def set_current(self, value: Union[float, int]):
        """
        Set current for the control mode

        Args:
            value (Union[float, int]): Current value in mA
        """
        pass

    @abstractmethod
    def set_position(self, value: Union[float, int]):
        """
        Set position for the control mode

        Args:
            value (Union[float, int]): Position value in radians
        """
        pass

    @abstractmethod
    def set_voltage(self, value: Union[float, int]):
        """
        Set voltage for the control mode

        Args:
            value (Union[float, int]): Voltage value in mV
        """
        pass

    def set_max_gains(self, gains: ControlGains) -> None:
        self._max_gains = gains

    @property
    def index(self) -> int:
        """
        Control Mode

        Returns:
            c_int: Control mode pass / flag
        """
        return int(self._control_mode_map)

    @property
    def name(self) -> str:
        """
        Control Mode Name

        Returns:
            str: Control mode name
        """
        return str(self._control_mode_map)

    @property
    def flag(self) -> c_int:
        """
        Control Mode Flag

        Returns:
            c_int: Control mode flag
        """
        return self._control_mode_map.flag

    @property
    def has_gains(self) -> bool:
        """
        Whether the mode has gains (Read Only)

        Returns:
            bool: True if the mode has gains, False otherwise
        """
        return self._has_gains

    @property
    def max_gains(self) -> ControlGains:
        """
        Maximum gains for the mode (Read Only)

        Returns:
            ControlGains, None: Maximum gains for the mode
        """
        return self._max_gains

    @property
    def actuator(self) -> "ActuatorBase":
        """
        Actuator (Read Only)

        Returns:
            ActuatorBase: Actuator instance
        """
        return self._actuator


class ActuatorBase(ABC):
    """Base class for the Actuator

    Args
    ----
    Gains (ControlGains): control gains applied
    motor_constants (MotorConstants): mechanical constants applied

    Methods
    -------
    start() -> None:
        Start method for the actuator

    stop() -> None:
        Stop method for the actuator

    update() -> None:
        Update method for the actuator

    set_control_mode(mode: ControlModeBase) -> None:
        set_control_mode method for the actuator

    set_voltage(voltage_value: float) -> None:
        set_voltage method for the actuator

    set_current(current_value: float) -> None:
        set_current method for the actuator

    set_motor_position(PositionCount: int) -> None:
        set_motor_position method for the actuator

    """

    def __init__(
        self,
        tag: str,
        control_modes: ControlModesBase,
        default_control_mode: ControlModeBase,
        gear_ratio: float,
        motor_constants: MotorConstants,
        frequency: int = 1000,
        offline: bool = False,
        *args,
        **kwargs,
    ) -> None:
        self._CONTROL_MODES: ControlModesBase = control_modes
        self._MOTOR_CONSTANTS: MotorConstants = motor_constants
        self._gear_ratio: float = gear_ratio
        self._tag: str = tag
        self._frequency: int = frequency
        self._data: Any = None
        self._mode: ControlModeBase = default_control_mode
        self._is_offline: bool = offline
        self._is_homed: bool = False

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, exc_traceback):
        self.stop()

    @abstractmethod
    def start(self) -> None:
        """Start method for the actuator"""
        pass

    @abstractmethod
    def stop(self) -> None:
        """Stop method for the actuator"""
        pass

    @abstractmethod
    def update(self) -> None:
        """update method for the actuator"""
        pass

    @abstractmethod
    def set_control_mode(self, mode: ControlModeBase) -> None:
        """set_control_mode method for the actuator

        Args:
            mode (ControlModeBase): mode applied to the actuator
        """

        self._mode.transition(to_mode=mode)
        self._mode = mode

    @abstractmethod
    def set_motor_voltage(self, value: Union[float, int]) -> None:
        """set_voltage method for the actuator

        Args:
            voltage_value (float): voltage value applied
        """
        pass

    @abstractmethod
    def set_motor_current(
        self,
        value=Union[float, int],
    ):
        """set_current method for the actuator

        Args:
            current_value (float): current value applied
        """
        pass

    @abstractmethod
    def set_motor_position(
        self,
        value=Union[float, int],
    ):
        """set_motor_position method for the actuator

        Args:
            PositionCount (int): encoder count for the motor position
        """
        pass

    @abstractmethod
    def set_current_gains(
        self,
        kp: int,
        ki: int,
        kd: int,
        ff: int,
    ):
        """set_current_gains method for the actuator

        Args:
            kp (int): Proportional gain
            ki (int): Integral gain
            kd (int): Derivative gain
            ff (int): Feedforward gain
        """
        pass

    @abstractmethod
    def set_position_gains(
        self,
        kp: int,
        ki: int,
        kd: int,
        ff: int,
    ):
        """set_position_gains method for the actuator

        Args:
            kp (int): Proportional gain
            ki (int): Integral gain
            kd (int): Derivative gain
            ff (int): Feedforward gain
        """
        pass

    @abstractmethod
    def set_impedance_gains(
        self,
        kp: int,
        ki: int,
        kd: int,
        k: int,
        b: int,
        ff: int,
    ):
        """set_impedance_gains method for the actuator

        Args:
            kp (int): Proportional gain
            ki (int): Integral gain
            kd (int): Derivative gain
            k (int): Stiffness of the impedance controller
            b (int): Damping of the impedance controller
            ff (int): Feedforward gain
        """
        pass

    @abstractmethod
    def home(self) -> None:
        """Homing method for the joint, should be implemented if the actuator has a homing sequence"""
        pass

    def set_max_case_temperature(self, temperature: float) -> None:
        self._MOTOR_CONSTANTS.set_max_case_temperature(temperature)

    def set_max_winding_temperature(self, temperature: float) -> None:
        self._MOTOR_CONSTANTS.set_max_winding_temperature(temperature)

    @property
    def CONTROL_MODES(self) -> ControlModesBase:
        """Control Modes (Read Only)

        Returns:
            List[ControlModeBase]: List of control modes
        """
        return self._CONTROL_MODES

    @property
    def MOTOR_CONSTANTS(self) -> MotorConstants:
        """Motor Constants (Read Only)

        Returns:
            MotorConstants: Motor Constants
        """
        return self._MOTOR_CONSTANTS

    @property
    def mode(self) -> ControlModeBase:
        """Current Control Mode (Read Only)

        Returns:
            ControlModeBase: Current control mode
        """
        return self._mode

    @property
    def tag(self) -> str:
        """Actuator Name (Read Only)

        Returns:
            str: Actuator name
        """
        return self._tag

    @property
    def is_homed(self) -> bool:
        """Homed (Read Only)

        Returns:
            bool: Homed
        """
        return self._is_homed

    @property
    def frequency(self) -> int:
        """Frequency (Read Only)

        Returns:
            int: Frequency
        """
        return self._frequency

    @property
    def is_offline(self) -> bool:
        """Offline (Read Only)

        Returns:
            bool: Offline
        """
        return self._is_offline

    @property
    def gear_ratio(self) -> float:
        """Gear Ratio (Read Only)

        Returns:
            float: Gear Ratio
        """
        return self._gear_ratio

    @property
    def max_case_temperature(self) -> float:
        """Max Case Temperature (Read Only)

        Returns:
            float: Max Case Temperature
        """
        return self._MOTOR_CONSTANTS.MAX_CASE_TEMPERATURE

    @property
    def max_winding_temperature(self) -> float:
        """Max Winding Temperature (Read Only)

        Returns:
            float: Max Winding Temperature
        """
        return self._MOTOR_CONSTANTS.MAX_WINDING_TEMPERATURE


if __name__ == "__main__":
    pass
