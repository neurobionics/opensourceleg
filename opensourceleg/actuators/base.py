from typing import Any, Callable, List, NamedTuple, Optional, Union

from abc import ABC, abstractmethod
from ctypes import c_int
from dataclasses import dataclass
from enum import Enum
from functools import wraps

import numpy as np

from opensourceleg.actuators.exceptions import VoltageModeMissingException
from opensourceleg.logging.logger import LOGGER


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

    kp: float = 0
    ki: float = 0
    kd: float = 0
    k: float = 0
    b: float = 0
    ff: float = 0


@dataclass
class ControlModeConfig:
    """Dataclass for control mode configuration

    Attributes
    ----------
    max_gains (ControlGains): maximum gains that can be set by the user
    entry_callback (Callable): callback function to be called when entering the mode
    exit_callback (Callable): callback function to be called when exiting the mode
    """

    flag: c_int
    max_gains: ControlGains
    entry_callback: Callable[[Any], None]
    exit_callback: Callable[[Any], None]


DEFAULT_POSITION_CONTROL_MODE_CONFIG = ControlModeConfig(
    flag=0,
    max_gains=ControlGains(kp=1000, ki=100, kd=100, ff=100),
    entry_callback=lambda _: LOGGER.info("Entering position control mode"),
    exit_callback=lambda _: LOGGER.info("Exiting position control mode"),
)

DEFAULT_CURRENT_CONTROL_MODE_CONFIG = ControlModeConfig(
    flag=1,
    max_gains=ControlGains(kp=1000, ki=100, kd=100, ff=100),
    entry_callback=lambda _: LOGGER.info("Entering current control mode"),
    exit_callback=lambda _: LOGGER.info("Exiting current control mode"),
)

DEFAULT_VOLTAGE_CONTROL_MODE_CONFIG = ControlModeConfig(
    flag=2,
    max_gains=ControlGains(kp=1000, ki=100, kd=100, ff=100),
    entry_callback=lambda _: LOGGER.info("Entering voltage control mode"),
    exit_callback=lambda _: LOGGER.info("Exiting voltage control mode"),
)

DEFAULT_IMPEDANCE_CONTROL_MODE_CONFIG = ControlModeConfig(
    flag=3,
    max_gains=ControlGains(kp=1000, ki=100, kd=100, k=100, b=100, ff=100),
    entry_callback=lambda _: LOGGER.info("Entering impedance control mode"),
    exit_callback=lambda _: LOGGER.info("Exiting impedance control mode"),
)

DEFAULT_VELOCITY_CONTROL_MODE_CONFIG = ControlModeConfig(
    flag=4,
    max_gains=ControlGains(kp=1000, ki=100, kd=100, ff=100),
    entry_callback=lambda _: LOGGER.info("Entering velocity control mode"),
    exit_callback=lambda _: LOGGER.info("Exiting velocity control mode"),
)

DEFAULT_TORQUE_CONTROL_MODE_CONFIG = ControlModeConfig(
    flag=5,
    max_gains=ControlGains(kp=1000, ki=100, kd=100, ff=100),
    entry_callback=lambda _: LOGGER.info("Entering torque control mode"),
    exit_callback=lambda _: LOGGER.info("Exiting torque control mode"),
)

DEFAULT_IDLE_CONTROL_MODE_CONFIG = ControlModeConfig(
    flag=6,
    max_gains=ControlGains(kp=0, ki=0, kd=0, ff=0),
    entry_callback=lambda _: LOGGER.info("Entering idle control mode"),
    exit_callback=lambda _: LOGGER.info("Exiting idle control mode"),
)


class CONTROL_MODES(NamedTuple):
    POSITION: Optional[ControlModeConfig] = None
    CURRENT: Optional[ControlModeConfig] = None
    VOLTAGE: Optional[ControlModeConfig] = None
    IMPEDANCE: Optional[ControlModeConfig] = None
    VELOCITY: Optional[ControlModeConfig] = None
    TORQUE: Optional[ControlModeConfig] = None
    IDLE: Optional[ControlModeConfig] = None


def require_control_mode(required_mode: CONTROL_MODES):
    def decorator(func):
        @wraps(func)
        def wrapper(self: ActuatorBase, *args, **kwargs):
            if self.mode != required_mode:
                raise ValueError(
                    f"Function {func.__name__} requires control mode {required_mode.name}"
                )
            return func(self, *args, **kwargs)

        return wrapper

    return decorator


def require_control_gains():
    def decorator(func):
        @wraps(func)
        def wrapper(self: ActuatorBase, *args, **kwargs):
            if not self.mode.has_gains:
                raise ValueError(f"Function {func.__name__} requires control gains")
            return func(self, *args, **kwargs)

        return wrapper

    return decorator


@dataclass(frozen=True)
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
        gear_ratio: float,
        motor_constants: MotorConstants,
        frequency: int = 1000,
        offline: bool = False,
        *args,
        **kwargs,
    ) -> None:
        self._MOTOR_CONSTANTS: MotorConstants = motor_constants
        self._gear_ratio: float = gear_ratio
        self._tag: str = tag
        self._frequency: int = frequency
        self._data: Any = None
        self._is_offline: bool = offline
        self._is_homed: bool = False

        self._motor_zero_position: float = 0.0
        self._motor_position_offset: float = 0.0

        self._joint_zero_position: float = 0.0
        self._joint_position_offset: float = 0.0
        self._joint_direction: int = 1

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, exc_traceback):
        self.stop()

    @property
    @abstractmethod
    def CONTROL_MODES(self) -> CONTROL_MODES:
        """
        Supported control modes for your actuator
        Please create a _CONTROL_MODES class variable in your actuator class and return it here
        """
        pass

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
    def set_control_mode(self, mode: CONTROL_MODES) -> None:
        """set_control_mode method for the actuator

        Args:
            mode (ControlModeBase): mode applied to the actuator
        """

    @abstractmethod
    def set_motor_voltage(self, value: float) -> None:
        """set_voltage method for the actuator

        Args:
            voltage_value (float): voltage value applied
        """
        pass

    @abstractmethod
    def set_motor_current(
        self,
        value=float,
    ):
        """set_current method for the actuator

        Args:
            current_value (float): current value applied
        """
        pass

    @abstractmethod
    def set_motor_position(
        self,
        value=float,
    ):
        """set_motor_position method for the actuator

        Args:
            PositionCount (int): encoder count for the motor position
        """
        pass

    @abstractmethod
    def set_current_gains(
        self,
        kp: float,
        ki: float,
        kd: float,
        ff: float,
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
        kp: float,
        ki: float,
        kd: float,
        ff: float,
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
        kp: float,
        ki: float,
        kd: float,
        k: float,
        b: float,
        ff: float,
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
    def CONTROL_MODES(self) -> CONTROL_MODES:
        """Control Modes (Read Only)

        Returns:
            List[ControlModeBase]: List of control modes
        """
        return CONTROL_MODES

    @property
    def MOTOR_CONSTANTS(self) -> MotorConstants:
        """Motor Constants (Read Only)

        Returns:
            MotorConstants: Motor Constants
        """
        return self._MOTOR_CONSTANTS

    @property
    def mode(self) -> CONTROL_MODES:
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

    @property
    def motor_zero_position(self) -> float:
        """Motor encoder zero position in radians."""
        return self._motor_zero_position

    @property
    def motor_position_offset(self) -> float:
        """Motor encoder offset in radians."""
        return self._motor_position_offset

    @property
    def joint_zero_position(self) -> float:
        """Joint encoder zero position in radians."""
        return self._joint_zero_position

    @property
    def joint_position_offset(self) -> float:
        """Joint encoder offset in radians."""
        return self._joint_position_offset

    @property
    def joint_direction(self) -> int:
        """Joint direction: 1 or -1"""
        return self._joint_direction


if __name__ == "__main__":
    pass
