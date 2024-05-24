"""
Actuators Interface Generalized
05/2024
"""

from typing import Any, Callable, Protocol, Union, overload

import ctypes
import os
import time
from abc import ABC, abstractmethod
from ctypes import c_int
from dataclasses import dataclass

# To be removed after Generalization
import flexsea.fx_enums as fxe
import numpy as np

# To be removed after Generalization
from flexsea.device import Device

from ...tools.logger import Logger
from ..thermal import ThermalModel

# from opensourceleg.hardware.actuators_base import ControlModes
# from opensourceleg.tools import safety


"""
    User Guide to opensourceleg.hardware.actuators
    
    ```
    ```
    
    Returns:
        _type_: _description_
"""


@dataclass
# Mandatory
class ControlGains:
    """
    Dataclass for controller gains

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
# Mandatory
class MecheConsts:
    """
    Imports necessary constants, compute the rest and protects them by @property

    Returns:
        _type_: _description_
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


# @dataclass
# class ActuatorComm:
#     """
#     Configuration of Actuator Definition & Communication.

#     Args:
#         ABC (_type_): _description_
#     """

#     name: str = "DephyActpack"

#     port: str = "/dev/ttyACM0"
#     baud_rate: int = 230400

#     frequency: int = 500
#     logger: Logger = Logger()

#     debug_level: int = 0

#     log: bool = False


@dataclass
class SafetySpec:
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


# Don't need to be imported by user
class ActuatorMode:

    def __init__(self, mode_pass: c_int, device: "Actuator") -> None:

        # self._which_mode: str = which_mode
        self._control_mode: c_int = mode_pass
        # self._device: Actuator = device
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
        Control mode

        Returns:
            c_int: Control mode
        """
        return self._control_mode

    @property
    def has_gains(self) -> bool:
        """
        Whether the mode has gains

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
        # self._which_mode = to_which_mode
        # self._control_mode = to_mode_pass
        # self.enter()

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
    def __init__(self, mode_pass: c_int, device: "Actuator") -> None:
        super().__init__(mode_pass=mode_pass, device=device)
        pass

    @abstractmethod
    def set_voltage(self, voltage_value: int):
        pass


class CurrentMode(ActuatorMode):
    def __init__(self, mode_pass: c_int, device: "Actuator") -> None:
        super().__init__(mode_pass=mode_pass, device=device)
        pass

    @abstractmethod
    def set_current(
        self,
        current_value: int,
        # Gains: ControlGains,
    ) -> None:
        # self._gains = Gains
        pass

    @abstractmethod
    def set_gains(self, Gains: ControlGains) -> None:
        # if self._mode == CurrentMode:
        #     return ControlGains(
        #         kp=Gains.kp, ki=Gains.ki, kd=0, K=0, B=0, ff=Gains.ff
        #     )
        #     self._has_gains = True
        # if self._mode == ImpedanceMode:
        #     return ControlGains(
        #         kp=Gains.kp, ki=Gains.ki, kd=0, K=Gains.K, B=Gains.B, ff=Gains.ff
        #     )
        #     self._has_gains = True
        # if self._mode == PositionMode:
        #     return ControlGains(
        #         kp=Gains.kp, ki=Gains.ki, kd=Gains.kd, K=0, B=0, ff=Gains.ff
        #     )
        #     self._has_gains = True
        # else:
        #     # TODO: should raise an error here
        #     pass
        self._gains = Gains
        self._has_gains = True


class PositionMode(ActuatorMode):
    def __init__(self, mode_pass: c_int, device: "Actuator") -> None:
        super().__init__(mode_pass=mode_pass, device=device)
        pass

    @abstractmethod
    def set_position(
        self,
        encoder_count: int,
        #  Gains: ControlGains,
    ) -> None:
        # self._gains = Gains
        pass

    def set_gains(self, Gains: ControlGains) -> None:
        # if self._mode == CurrentMode:
        #     return ControlGains(
        #         kp=Gains.kp, ki=Gains.ki, kd=0, K=0, B=0, ff=Gains.ff
        #     )
        #     self._has_gains = True
        # if self._mode == ImpedanceMode:
        #     return ControlGains(
        #         kp=Gains.kp, ki=Gains.ki, kd=0, K=Gains.K, B=Gains.B, ff=Gains.ff
        #     )
        #     self._has_gains = True
        # if self._mode == PositionMode:
        #     return ControlGains(
        #         kp=Gains.kp, ki=Gains.ki, kd=Gains.kd, K=0, B=0, ff=Gains.ff
        #     )
        #     self._has_gains = True
        # else:
        #     # TODO: should raise an error here
        #     pass
        self._gains = Gains
        self._has_gains = True


class ImpedanceMode(ActuatorMode):
    def __init__(self, mode_pass: c_int, device: "Actuator") -> None:
        super().__init__(mode_pass=mode_pass, device=device)
        pass

    @abstractmethod
    def set_gains(self, Gains: ControlGains):
        self._gains = Gains
        self._has_gains = True
        # self._set_gains(Mode=self, Gains=ImpedanceGains)
        pass

    @abstractmethod
    def set_position(
        self,
        encoder_count: int,
        #  Gains: ControlGains
    ) -> None:
        # self._gains = Gains
        pass


# @dataclass(init=False)
# class ActuatorControlModes:
#     """
#     Actpack modes

#     Args:
#         voltage (VoltageMode): Voltage mode
#         current (CurrentMode): Current mode
#         position (PositionMode): Position mode
#         impedance (ImpedanceMode): Impedance mode
#     """

#     voltage: VoltageMode
#     current: CurrentMode
#     position: PositionMode
#     impedance: ImpedanceMode

#     def __init__(self, device: "Actuator") -> None:
#         self.voltage = VoltageMode(device=device)
#         self.current = CurrentMode(device=device)
#         self.position = PositionMode(device=device)
#         self.impedance = ImpedanceMode(device=device)

#     def __repr__(self) -> str:
#         return f"ActuatorControlModes"


# Mandatory
class Actuator(ABC):

    def __init__(
        self,
        # port: str,
        # baud_rate: int,
        Gains: ControlGains = ControlGains(),
        MecheSpecs: MecheConsts = MecheConsts(
            # MOTOR_COUNT_PER_REV=16384,
            # NM_PER_AMP=0.1133,
            # IMPEDANCE_A=0.00028444,
            # IMPEDANCE_C=0.0007812,
            # MAX_CASE_TEMPERATURE=80,
            # M_PER_SEC_SQUARED_ACCLSB=9.80665 / 8192,
        ),
        # Communication: ActuatorComm,
        # Safety: SafetySpec,
        *args,
        **kwargs,
    ) -> None:
        # super().__init__()
        # self._port: str = port
        # self._baud_rate: int = baud_rate
        # ActuatorAction.__init__(self, device=self)
        self._gains: Any = Gains
        # self._actuator_mode_manager =
        self._MecheConsts: MecheConsts = MecheSpecs
        # self._Communication: ActuatorComm = Communication
        # self._frequency = self._Communication.frequency
        self._mode: Any = None

        # self._which_mode: str = self._mode._which_mode
        # self._mode_pass: c_int
        # self._safety: SafetySpec = Safety
        # self._log: Logger = Logger()

        # self._has_gains: bool = self._mode._has_gains

        # self._sensor: Any = None

        # self._encoder_map = None

        # self._motor_zero_position = 0.0
        # self._joint_zero_position = 0.0

        # self._joint_offset = 0.0
        # self._motor_offset = 0.0

        # self._joint_direction = 1.0

        # self._thermal_model: ThermalModel = ThermalModel(
        #     temp_limit_windings=80,
        #     soft_border_C_windings=10,
        #     temp_limit_case=70,
        #     soft_border_C_case=10,
        # )
        # self._thermal_scale: float = 1.0

        # self.port = kwargs["port"]
        # self.baud_rate = kwargs["baud_rate"]
        # self._frequency = kwargs["frequency"]
        # TODO: use *argv and **kwargs to allow passing additional info

    @abstractmethod
    def start(self) -> None:
        # self.set_mode(which_mode="voltage", mode_pass=self._mode_pass)
        # self.set_voltage(voltage_value=0, mode_pass=self._mode_pass)
        # try:
        #     self.open(
        #         freq=self._frequency,
        #         log_level=self._debug_level,
        #         log_enabled=self._dephy_log,
        #     )
        # except OSError as e:
        #     print("\n")
        #     self._log.error(
        #         msg=f"[{self.__repr__()}] Need admin previleges to open the port '{self.port}'. \n\nPlease run the script with 'sudo' command or add the user to the dialout group.\n"
        #     )
        #     os._exit(status=1)
        # time.sleep(0.1)
        # # self._sensor = self.read()
        # self._mode.enter()
        pass

    @abstractmethod
    def stop(self) -> None:
        """
        This is the stop method for the actuator. It should be called before the program exits.

        A few best practices to follow:
        1. Set the actuator to voltage mode and set the voltage to 0.
        2. Close the serial port.
        3. Sleep for a short duration to allow the actuator to stop.

        """
        # self.set_mode(mode=ActuatorControlModes.voltage)
        # self.set_voltage(voltage_value=0, mode_pass=self._mode_pass)

        # time.sleep(0.1)
        # self.close()
        pass

    @abstractmethod
    def update(self) -> None:
        # self._gains = self._mode._gains
        pass

    @abstractmethod
    def set_mode(self, mode: ActuatorMode) -> None:
        # if type(mode) in ActuatorControlModes:
        #     self._mode.transition(to_state=mode)
        #     self._mode = mode
        # else:
        #     # TODO: check method of raising error
        #     # raise ValueError("Invalid mode")
        #     pass
        pass

    # @property
    # def frequency(self) -> int:
    #     return self._frequency

    @property
    def mode(self) -> Any:
        return self._mode

    @abstractmethod
    def set_voltage(self, voltage_value: float):
        # self._mode.set_voltage(voltage_value=voltage_value)
        # take motor steps here
        pass

    @abstractmethod
    def set_current(
        self,
        current_value: float,
    ):
        # self._mode._set_current(current_value=current_value, Gains=CurrentGain)
        # take motor steps here
        pass

    # @abstractmethod
    # def set_impedance(
    #     self, PositionCount: int, ImpedanceGains: ControlGains, mode_pass: c_int
    # ):
    #     # self._mode._set_impedance(ImpedanceGains=ImpedanceGains)
    #     # take motor steps here
    #     pass

    @abstractmethod
    def set_motor_position(
        self,
        PositionCount: int,
    ):
        # self._mode._set_position(encoder_count=PositionCount, Gains=PositionGains)
        # take motor steps here
        pass


if __name__ == "__main__":
    pass
