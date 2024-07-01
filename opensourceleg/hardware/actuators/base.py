"""
Actuators Interface Generalized
05/2024
"""

from typing import Any, Callable, List

from abc import ABC, abstractmethod
from ctypes import c_int
from dataclasses import dataclass
from enum import Enum

import numpy as np

"""_summary_

    Returns:
        _type_: _description_
"""


class ControlModeIndices(Enum):
    """ModeIndices Enum for the actuator modes

    This class shares the same values as Dephy's flexSEA fxEnums

    Attributes
    ----------
    POSITION (c_int): Position Mode
    VOLTAGE (c_int): Voltage Mode
    CURRENT (c_int): Current Mode
    IMPEDANCE (c_int): Impedance Mode
    NONE (c_int): No Mode

    """

    POSITION = c_int(0)
    VOLTAGE = c_int(1)
    CURRENT = c_int(2)
    IMPEDANCE = c_int(3)
    NONE = c_int(4)


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


class ActuatorMode(ABC):
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
        to_mode: opensourceleg.hardware.actuators.base.ActuatorMode
    ) → None:

    Transition to another mode. Calls the exit callback of the current mode and the entry callback of the new mode

    """

    def __init__(
        self,
        control_mode_index: c_int,
        control_mode_name: str,
        actuator: "Actuator",
        entry_callbacks: list[Callable[[], None]] = [lambda: None],
        exit_callbacks: list[Callable[[], None]] = [lambda: None],
        max_command: float | int | None = None,
        max_gains: ControlGains | None = None,
    ) -> None:
        self._control_mode_index: c_int = control_mode_index
        self._control_mode_name: str = control_mode_name
        self._has_gains: bool = False
        self._gains: Any = None
        self._entry_callbacks: list[Callable[[], None]] = entry_callbacks
        self._exit_callbacks: list[Callable[[], None]] = exit_callbacks
        self._actuator: "Actuator" = actuator
        self._max_command: float | int | None = max_command
        self._max_gains: ControlGains | None = max_gains

    def __eq__(self, __o: object) -> bool:
        if isinstance(__o, ActuatorMode):
            return self.index == __o.index
        return False

    def __str__(self) -> str:
        return str(object=self.index)

    def __repr__(self) -> str:
        return f"ActpackMode[{self.name}]"

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

    # To be modified
    def transition(self, to_mode: "ActuatorMode") -> None:
        """
        Transition to another mode. Calls the exit callback of the current mode
        and the entry callback of the new mode.

        Args:
            to_mode (ActpackMode): Mode to transition to
        """
        self.exit()
        to_mode.enter()

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
            assert 0 <= gains.K <= self._max_gains.K
            assert 0 <= gains.B <= self._max_gains.B
            assert 0 <= gains.ff <= self._max_gains.ff

        self._gains = gains
        self._has_gains = True
        self._actuator.set_gains(**vars(gains))

    @abstractmethod
    def set_command(self, command: float | int) -> None:
        """set_command method for the control mode. Please use the super method only if applicable to the child class if not override this method
        Args:
            command (Any): command applied
        """
        if self.max_command is not None:
            assert 0 <= command <= self._max_command

        # TODO: Modify this for new flexsea API, send_motor_command is deprecated
        self._actuator.send_motor_command(
            ctrl_mode=self.index,
            value=command,
        )

    @property
    def index(self) -> c_int:
        """
        Control Mode

        Returns:
            c_int: Control mode pass / flag
        """
        return self._control_mode_index

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
    def max_gains(self) -> ControlGains | None:
        """
        Maximum gains for the mode (Read Only)

        Returns:
            ControlGains | None: Maximum gains for the mode
        """
        return self._max_gains

    @property
    def max_command(self) -> float | int | None:
        """
        Maximum command for the mode (Read Only)

        Returns:
            float | int | None: Maximum command for the mode
        """
        return self._max_command


class Actuator(ABC):
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

    set_control_mode(mode: ActuatorMode) -> None:
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
        gains: ControlGains = ControlGains(),
        motor_constants: MotorConstants = MotorConstants(),
        *args,
        **kwargs,
    ) -> None:
        self._gains: ControlGains = gains
        self._MotorConstants: MotorConstants = motor_constants
        self._mode: Any = None

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
    def set_control_mode(self, mode: ActuatorMode) -> None:
        """set_control_mode method for the actuator

        Args:
            mode (ActuatorMode): mode applied to the actuator
        """
        pass

    @abstractmethod
    def set_motor_voltage(self, value: float | int) -> None:
        """set_voltage method for the actuator

        Args:
            voltage_value (float): voltage value applied
        """
        pass

    @abstractmethod
    def set_motor_current(
        self,
        value=float | int,
    ):
        """set_current method for the actuator

        Args:
            current_value (float): current value applied
        """
        pass

    @abstractmethod
    def set_motor_position(
        self,
        value=float | int,
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


if __name__ == "__main__":
    pass
