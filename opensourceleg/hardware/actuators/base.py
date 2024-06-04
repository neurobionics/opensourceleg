"""
Actuators Interface Generalized
05/2024
"""

from typing import Any, Callable, Protocol, Union, overload

from abc import ABC, abstractmethod
from ctypes import c_int
from dataclasses import dataclass

# To be removed after Generalization
import numpy as np

# To be removed after Generalization

"""_summary_

    Returns:
        _type_: _description_
"""


@dataclass
# Mandatory
class ControlGains:
    """Dataclass for controller gains
    Args:
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
class MecheConsts:
    """Dataclass for mechanical constants
    Args:
    MOTOR_COUNT_PER_REV: float = 16384
    NM_PER_AMP: float = 0.1133
    IMPEDANCE_A: float = 0.00028444
    IMPEDANCE_C: float = 0.0007812
    MAX_CASE_TEMPERATURE: float = 80
    M_PER_SEC_SQUARED_ACCLSB: float = 9.80665 / 8192
    """

    def __repr__(self) -> str:
        return f"MecheConsts"

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


@dataclass
class SafetySpec:
    """
    `SafetySpec` class for safety specifications of the actuator.
    To be linked to safety module
    """

    MAX_VOLTAGE = 0
    MAX_CURRENT = 0
    MAX_POSITION_COUNT = 0
    MAX_IMPEDANCE_KP = 0
    MAX_IMPEDANCE_KI = 0
    MAX_IMPEDANCE_KD = 0
    MAX_IMPEDANCE_K = 0
    MAX_IMPEDANCE_B = 0
    MAX_IMPEDANCE_FF = 0
    MAX_POSITION_KP = 0
    MAX_POSITION_KI = 0
    MAX_POSITION_KD = 0
    MAX_CURRENT_KP = 0
    MAX_CURRENT_KI = 0
    MAX_CURRENT_KD = 0


class ActuatorMode:
    """Base mode class for the actuator mode transfer
    Args:
        mode_pass (c_int): Mode pass / flag for the actuator
        device (Actuator): Actuator instance
    """

    def __init__(self, mode_pass: c_int, device: "Actuator") -> None:

        self._control_mode: c_int = mode_pass
        self._has_gains: bool = False
        self._gains: Any = None
        self._entry_callback: Callable[[], None] = lambda: None
        self._exit_callback: Callable[[], None] = lambda: None

    def __eq__(self, __o: object) -> bool:
        if isinstance(__o, ActuatorMode):
            return self._control_mode == __o._control_mode
        return False

    def __str__(self) -> str:
        return str(object=self._control_mode)

    def __repr__(self) -> str:
        return f"ActpackMode[{self._control_mode}]"

    @property
    def mode(self) -> c_int:
        """
        Control Mode (Read Only)

        Returns:
            c_int: Control mode pass / flag
        """
        return self._control_mode

    @property
    def has_gains(self) -> bool:
        """
        Whether the mode has gains (Read Only)

        Returns:
            bool: True if the mode has gains, False otherwise
        """
        return self._has_gains

    def enter(self) -> None:
        """
        Calls the entry callback
        """
        self._entry_callback()

    def exit(self) -> None:
        """
        Calls the exit callback
        """
        self._exit_callback()

    # To be modified
    def transition(self, to_state: "ActuatorMode") -> None:
        """
        Transition to another mode. Calls the exit callback of the current mode
        and the entry callback of the new mode.

        Args:
            to_state (ActpackMode): Mode to transition to
        """
        self.exit()
        to_state.enter()

    def set_voltage(self, voltage: int) -> None:
        """
        This method should be implemented by the child class. It should set the q axis voltage.
        """
        pass

    def set_current(self, current: int) -> None:
        """
        This method should be implemented by the child class. It should set the q axis current.
        """
        pass

    def set_position(self, counts: int) -> None:
        """
        This method should be implemented by the child class. It should set the motor position.
        """
        pass


class VoltageMode(ActuatorMode, ABC):
    """Base Mode for Voltage Mode of Actuator

    Args:
        mode_pass (c_int): Mode pass / flag for the actuator
        device (Actuator): Actuator instance
    """

    def __init__(self, mode_pass: c_int, device: "Actuator") -> None:
        super().__init__(mode_pass=mode_pass, device=device)
        pass

    @abstractmethod
    def set_voltage(self, voltage_value: int):
        """set_voltage method for the voltage mode

        Args:
            voltage_value (int): voltage value applied
        """
        pass


class CurrentMode(ActuatorMode):
    """Base Mode for Current Mode of Actuator

    Args:
        mode_pass (c_int): Mode pass / flag for the actuator
        device (Actuator): Actuator instance
    """

    def __init__(self, mode_pass: c_int, device: "Actuator") -> None:
        super().__init__(mode_pass=mode_pass, device=device)
        pass

    @abstractmethod
    def set_current(
        self,
        current_value: int,
    ) -> None:
        """set_current method for the current mode

        Args:
            current_value (int): current value applied
        """
        pass

    @abstractmethod
    def set_gains(self, Gains: ControlGains) -> None:
        """set_gains method for the current mode
        Args:
            Gains (ControlGains): control gains applied
        """
        self._gains = Gains
        self._has_gains = True


class PositionMode(ActuatorMode):
    """Base Mode for Position Mode of Actuator
    Args:
        mode_pass (c_int): Mode pass / flag for the actuator
        device (Actuator): Actuator instance
    """

    def __init__(self, mode_pass: c_int, device: "Actuator") -> None:
        super().__init__(mode_pass=mode_pass, device=device)
        pass

    @abstractmethod
    def set_position(
        self,
        encoder_count: int,
    ) -> None:
        """set_position method for the position mode

        Args:
            encoder_count (int): encoder count applied
        """
        pass

    def set_gains(self, Gains: ControlGains) -> None:
        """set_gains method for the position mode

        Args:
            Gains (ControlGains): gain values applied
        """
        self._gains = Gains
        self._has_gains = True


class ImpedanceMode(ActuatorMode):
    """Base Mode for Impedance Mode of Actuator

    Args:
        mode_pass (c_int): Mode pass / flag for the actuator
        device (Actuator): Actuator instance
    """

    def __init__(self, mode_pass: c_int, device: "Actuator") -> None:
        super().__init__(mode_pass=mode_pass, device=device)
        pass

    @abstractmethod
    def set_gains(self, Gains: ControlGains):
        """set_gains method for the impedance mode
        Args:
            Gains (ControlGains): control gains applied
        """
        self._gains = Gains
        self._has_gains = True
        pass

    @abstractmethod
    def set_position(
        self,
        encoder_count: int,
    ) -> None:
        """set_position method for the impedance mode

        Args:
            encoder_count (int): encoder count applied
        """
        pass


class Actuator(ABC):
    """Base class for the Actuator

    Args:
        Gains (ControlGains): control gains applied
        MecheSpecs (MecheConsts): mechanical constants applied
    """

    def __init__(
        self,
        Gains: ControlGains = ControlGains(),
        MecheSpecs: MecheConsts = MecheConsts(),
        *args,
        **kwargs,
    ) -> None:
        self._gains: Any = Gains
        self._MecheConsts: MecheConsts = MecheSpecs
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
    def set_mode(self, mode: ActuatorMode) -> None:
        """set_mode method for the actuator

        Args:
            mode (ActuatorMode): mode applied to the actuator
        """
        pass

    @property
    def mode(self) -> Any:
        return self._mode

    @abstractmethod
    def set_voltage(self, voltage_value: float):
        """set_voltage method for the actuator

        Args:
            voltage_value (float): voltage value applied
        """
        pass

    @abstractmethod
    def set_current(
        self,
        current_value: float,
    ):
        """set_current method for the actuator

        Args:
            current_value (float): current value applied
        """
        pass

    @abstractmethod
    def set_motor_position(
        self,
        PositionCount: int,
    ):
        """set_motor_position method for the actuator

        Args:
            PositionCount (int): encoder count for the motor position
        """
        pass


if __name__ == "__main__":
    pass
