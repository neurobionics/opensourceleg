"""
Actuators Interface Generalized
05/2024
"""

from typing import Any, Callable, List, Union

from abc import ABC, ABCMeta, abstractmethod
from ctypes import c_int
from dataclasses import dataclass
from enum import Enum, EnumMeta
from functools import wraps

import numpy as np

from opensourceleg.tools.logger import LOGGER, Logger

"""_summary_

    Returns:
        _type_: _description_
"""


def verify_control_mode(func):
    @wraps(func)
    def wrapper(self, *args, **kwargs):
        if self.actuator.mode != self:
            raise ControlModeException(
                actuator_name=self.actuator.actuator_name,
                expected_mode=self.name,
                current_mode=self.actuator.mode.name,
            )
        return func(self, *args, **kwargs)

    return wrapper


class ControlModeException(Exception):
    """Control Mode Exception

    Attributes
    ----------
    message (str): Error message

    """

    def __init__(
        self,
        actuator_name: str,
        expected_mode: str,
        current_mode: str,
    ) -> None:
        super().__init__(
            f"Expected the {actuator_name} to be in {expected_mode} mode but it was in {current_mode} mode"
        )


class VoltageModeMissingException(Exception):
    """Voltage Mode Missing Exception

    Attributes
    ----------
    message (str): Error message

    """

    def __init__(self, actuator_name: str) -> None:
        super().__init__(f"{actuator_name} must have a voltage mode")


class ControlModesMapping(Enum):
    """Control Modes Dictionary

    Attributes
    ----------
    VOLTAGE (c_int): Voltage control mode
    CURRENT (c_int): Current control mode
    POSITION (c_int): Position control mode
    IMPEDANCE (c_int): Impedance control mode

    """

    VOLTAGE = c_int(0), "voltage"
    CURRENT = c_int(1), "current"
    POSITION = c_int(2), "position"
    IMPEDANCE = c_int(3), "impedance"

    def __new__(cls, c_int_value, str_value):
        obj = object.__new__(cls)
        obj._value_ = c_int_value
        obj.str_value = str_value
        return obj

    def __int__(self):
        return self.value.value

    def __str__(self):
        return str.upper(self.str_value)


class ControlModesMeta(type):
    def __init__(cls, name, bases, attrs):
        if bases and "VOLTAGE" not in attrs:
            print(f"Class: {cls}")
            print(f"Name: {name}")
            print(f"Bases: {bases}")
            print(f"Attrs: {attrs}")
            raise VoltageModeMissingException(cls.__name__)
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
    K (int): Stiffness of the impedance controller
    B (int): Damping of the impedance controller
    ff (int): Feedforward gain

    """

    kp: int = 0
    ki: int = 0
    kd: int = 0
    K: int = 0
    B: int = 0
    ff: int = 0

    def __repr__(self) -> str:
        return f"kp={self.kp}, ki={self.ki}, kd={self.kd}, K={self.K}, B={self.B}, ff={self.ff}"


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
        control_mode_index: c_int,
        control_mode_name: str,
        actuator: Union["ActuatorBase", None] = None,
        entry_callbacks: list[Callable[[], None]] = [lambda: None],
        exit_callbacks: list[Callable[[], None]] = [lambda: None],
        max_command: Union[float, int] = None,
        max_gains: ControlGains = None,
    ) -> None:
        self._control_mode_index: c_int = control_mode_index
        self._control_mode_name: str = control_mode_name
        self._has_gains: bool = False
        self._gains: Any = None
        self._entry_callbacks: list[Callable[[], None]] = entry_callbacks
        self._exit_callbacks: list[Callable[[], None]] = exit_callbacks
        self._actuator: "ActuatorBase" = actuator
        self._max_command: Union[float, int] = max_command
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

    def add_actuator(self, actuator: "ActuatorBase") -> None:
        """
        Assign actuator to the mode

        Args:
            actuator (ActuatorBase): Actuator instance
        """
        self._actuator = actuator

    @abstractmethod
    @verify_control_mode
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
            assert 0 <= gains.K <= self._max_gains.K
            assert 0 <= gains.B <= self._max_gains.B
            assert 0 <= gains.ff <= self._max_gains.ff

        self._gains = gains
        self._has_gains = True

        if self.actuator is not None:
            self.actuator.set_gains(**vars(gains))

    @abstractmethod
    @verify_control_mode
    def set_command(self, value: Union[float, int], expected_mode: c_int) -> None:
        """set_command method for the control mode. Please use the super method only if applicable to the child class if not override this method
        Args:
            command (Any): command applied
        """
        if self.max_command is not None:
            assert 0 <= value <= self._max_command

        if self.actuator is not None:
            # TODO: Modify this for new flexsea API, send_motor_command is deprecated
            self.actuator.send_motor_command(
                ctrl_mode=self.index,
                value=value,
            )

    def set_max_command(self, value: Union[float, int]) -> None:
        self._max_command = value

    def set_max_gains(self, gains: ControlGains) -> None:
        self._max_gains = gains

    @property
    def index(self) -> int:
        """
        Control Mode

        Returns:
            c_int: Control mode pass / flag
        """
        return int(self._control_mode_index)

    @property
    def name(self) -> str:
        """
        Control Mode Name

        Returns:
            str: Control mode name
        """
        return self._control_mode_name

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
    def max_command(self) -> Union[float, int]:
        """
        Maximum command for the mode (Read Only)

        Returns:
            float, int: Maximum command for the mode
        """
        return self._max_command

    @property
    def actuator(self) -> "ActuatorBase":
        """
        Actuator (Read Only)

        Returns:
            ActuatorBase: Actuator instance
        """
        return self.actuator


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
        actuator_name: str,
        control_modes: ControlModesBase,
        motor_constants: MotorConstants,
        frequency: int = 1000,
        offline: bool = False,
        *args,
        **kwargs,
    ) -> None:
        self._control_modes: ControlModesBase = control_modes
        self._motor_constants: MotorConstants = motor_constants

        self._actuator_name: str = actuator_name
        self._frequency: int = frequency
        self._data: Any = None
        self._mode: ControlModeBase = None
        self._is_offline: bool = offline

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
        if mode in self.control_modes:
            self._mode.transition(to_mode=mode)
            self._mode = mode
        else:
            LOGGER.warning(msg=f"[{self.__repr__()}] Mode {mode} not found")

    @abstractmethod
    def set_motor_voltage(self, value: Union[float, int]) -> None:
        """set_voltage method for the actuator

        Args:
            voltage_value (float): voltage value applied
        """
        self.mode.set_command(value=value)

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
        K: int,
        B: int,
        ff: int,
    ):
        """set_impedance_gains method for the actuator

        Args:
            kp (int): Proportional gain
            ki (int): Integral gain
            kd (int): Derivative gain
            K (int): Stiffness of the impedance controller
            B (int): Damping of the impedance controller
            ff (int): Feedforward gain
        """
        pass

    @property
    def control_modes(self) -> ControlModesBase:
        """Control Modes (Read Only)

        Returns:
            List[ControlModeBase]: List of control modes
        """
        return self._control_modes

    @property
    def motor_constants(self) -> MotorConstants:
        """Motor Constants (Read Only)

        Returns:
            MotorConstants: Motor Constants
        """
        return self._motor_constants

    @property
    def mode(self) -> ControlModeBase:
        """Current Control Mode (Read Only)

        Returns:
            ControlModeBase: Current control mode
        """
        return self._mode

    @property
    def actuator_name(self) -> str:
        """Actuator Name (Read Only)

        Returns:
            str: Actuator name
        """
        return self._actuator_name

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


if __name__ == "__main__":
    pass
