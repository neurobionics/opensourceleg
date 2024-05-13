"""
Actuators Interface Generalized
05/2024
"""

from typing import Any, Callable, Union, overload

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

from opensourceleg.hardware.actuators import ControlModes

from ..tools.logger import Logger
from .thermal import ThermalModel

"""
    User Guide to opensourceleg.hardware.actuators
    
    ```
    ```
    
    Returns:
        _type_: _description_
"""


# @dataclass
# class ControlModesInput:
#     """
#     Control modes definition.

#     All members are in ctypes.c_int type. Will Change later?

#     Available modes are voltage, current, position, impedance.
#     """

#     voltage: ctypes.c_int
#     current: ctypes.c_int
#     position: ctypes.c_int
#     impedance: ctypes.c_int

#     def __repr__(self) -> str:
#         return f"ControlModes"


@dataclass
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
class MECHECONSTS:
    """
    Imports necessary constants, compute the rest and protects them by @property

    Returns:
        _type_: _description_
    """

    MOTOR_COUNT_PER_REV: float = 0
    NM_PER_AMP: float = 0
    IMPEDANCE_A: float = 0
    IMPEDANCE_C: float = 0
    MAX_CASE_TEMPERATURE: float = 0

    @property
    def NM_PER_MILLIAMP(self) -> float:
        return self._NM_PER_MILLIAMP

    @NM_PER_MILLIAMP.setter
    @classmethod
    def NM_PER_MILLIAMP(cls) -> None:
        cls._NM_PER_MILLIAMP: float = cls.NM_PER_AMP / 1000

    @property
    def RAD_PER_COUNT(self) -> float:
        return self._RAD_PER_COUNT

    @RAD_PER_COUNT.setter
    @classmethod
    def RAD_PER_COUNT(cls) -> None:
        cls._RAD_PER_COUNT: float = 2 * np.pi / cls.MOTOR_COUNT_PER_REV

    @property
    def RAD_PER_DEG(self) -> float:
        return self._RAD_PER_DEG

    @RAD_PER_DEG.setter
    @classmethod
    def RAD_PER_DEG(cls) -> None:
        cls._RAD_PER_DEG: float = np.pi / 180

    @property
    def RAD_PER_SEC_GYROLSB(self) -> float:
        return self._RAD_PER_SEC_GYROLSB

    @RAD_PER_SEC_GYROLSB.setter
    @classmethod
    def RAD_PER_SEC_GYROLSB(cls) -> None:
        cls._RAD_PER_SEC_GYROLSB: float = np.pi / 180 / 32.8

    @property
    def NM_PER_RAD_TO_K(self) -> float:
        return self._NM_PER_RAD_TO_K

    @NM_PER_RAD_TO_K.setter
    @classmethod
    def NM_PER_RAD_TO_K(cls) -> None:
        cls._NM_PER_RAD_TO_K: float = (
            cls.RAD_PER_COUNT / cls.IMPEDANCE_C * 1e3 / cls.NM_PER_AMP
        )

    @property
    def NM_S_PER_RAD_TO_B(self) -> float:
        return self._NM_S_PER_RAD_TO_B

    @NM_S_PER_RAD_TO_B.setter
    @classmethod
    def NM_S_PER_RAD_TO_B(cls) -> None:
        cls._NM_S_PER_RAD_TO_B: float = (
            cls.RAD_PER_DEG / cls.IMPEDANCE_A * 1e3 / cls.NM_PER_AMP
        )

    def __repr__(self) -> str:
        return f"MECHECONSTS"


class ActuatorMode:
    """
    Base class for Actpack modes

    Args:
        control_mode (c_int): Control mode
        device (DephyActpack): Dephy Actpack
    """

    def __init__(self, mode_command: str, device: "ActuatorObject") -> None:

        # self._control_mode: c_int = control_mode
        self._device: ActuatorObject = device
        self._entry_callback: Callable[[], None] = lambda: None
        self._exit_callback: Callable[[], None] = lambda: None
        self._control_mode: c_int
        self._has_gains = False
        self._mode: str = mode_command
        self._entry_callback = self._entry
        self._exit_callback = self._exit

        if mode_command == "voltage":
            # To be generalized
            self._control_mode = fxe.FX_VOLTAGE
        elif mode_command == "current":
            # To be generalized
            self._control_mode = fxe.FX_CURRENT
        elif mode_command == "position":
            # To be generalized
            self._control_mode = fxe.FX_POSITION
        elif mode_command == "impedance":
            # To be generalized
            self._control_mode = fxe.FX_IMPEDANCE
        else:
            # should raise an error here
            pass
        self._entry_callback = self._entry
        self._exit_callback = self._exit

    def __eq__(self, __o: object) -> bool:
        if isinstance(__o, ActuatorMode):
            return self._control_mode == __o._control_mode
        return False

    def __str__(self) -> str:
        return str(object=self._control_mode)

    def __repr__(self) -> str:
        return f"ActpackMode[{self._control_mode}]"

    @property
    def mode(self) -> str:
        """
        Control mode

        Returns:
            c_int: Control mode
        """
        return self._mode

    @property
    def has_gains(self) -> bool:
        """
        Whether the mode has gains

        Returns:
            bool: True if the mode has gains, False otherwise
        """
        return self._has_gains

    def _entry(self) -> None:
        self._device._log.debug(
            msg=f"[{self.__repr__()}] Entering {self._control_mode} mode."
        )
        if self._mode == "voltage":
            # To be generalized
            pass
        elif self._mode == "current":
            # To be generalized
            pass
        elif self._mode == "position":
            # To be generalized
            pass
        elif self._mode == "impedance":
            # To be generalized
            pass
        else:
            # should raise an error here
            pass
        time.sleep(0.1)

    def _exit(self) -> None:
        # self._device._log.debug(msg=f"[Actpack] Exiting Voltage mode.")
        # self._set_voltage(voltage=0)
        self._device._log.debug(msg=f"[{self.__repr__()}] Exiting {self._mode} mode.")
        if self._mode == "voltage":
            self._set_voltage(voltage=0)
            pass
        elif self._mode == "current":
            # To be generalized
            pass
        elif self._mode == "position":
            # To be generalized
            pass
        elif self._mode == "impedance":
            # To be generalized
            pass
        else:
            # should raise an error here
            pass
        time.sleep(0.1)

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

    def transition(self, to_state: "ActuatorMode") -> None:
        """
        Transition to another mode. Calls the exit callback of the current mode
        and the entry callback of the new mode.

        Args:
            to_state (ActpackMode): Mode to transition to
        """
        self.exit()
        to_state.enter()

    def _set_voltage(self, voltage: int) -> None:
        """
        This method should be implemented by the child class. It should set the q axis voltage.
        """

        # self._device.send_motor_command(
        #     ctrl_mode=self.mode,
        #     value=voltage,
        # )
        if self._mode == "voltage":
            # To be generalized
            pass
        else:
            # should raise an error here
            pass

    def _set_current(self, current: int) -> None:
        """
        This method should be implemented by the child class. It should set the q axis current.
        """
        if self._mode == "current":
            # To be generalized
            pass
        else:
            # should raise an error here
            pass

    def _set_motor_position(self, counts: int) -> None:
        """
        This method should be implemented by the child class. It should set the motor position.
        """
        if self._mode == "position":
            # To be generalized
            pass
        else:
            # should raise an error here
            pass

    def _set_gains(self, Gains: ControlGains) -> None:
        if self._mode == "voltage":
            # should raise an error here
            pass
        elif self._mode == "current":
            assert 0 <= Gains.kp <= 80, "kp must be between 0 and 80"
            assert 0 <= Gains.ki <= 800, "ki must be between 0 and 800"
            assert 0 <= Gains.ff <= 128, "ff must be between 0 and 128"
            # to be generalized
        elif self._mode == "position":
            assert 0 <= Gains.kp <= 1000, "kp must be between 0 and 1000"
            assert 0 <= Gains.ki <= 1000, "ki must be between 0 and 1000"
            assert 0 <= Gains.kd <= 1000, "kd must be between 0 and 1000"
        elif self._mode == "impedance":
            assert 0 <= Gains.kp <= 80, "kp must be between 0 and 80"
            assert 0 <= Gains.ki <= 800, "ki must be between 0 and 800"
            assert 0 <= Gains.ff <= 128, "ff must be between 0 and 128"
            assert 0 <= Gains.K, "K must be greater than 0"
            assert 0 <= Gains.B, "B must be greater than 0"

        # set gains to be generalized
        # self._device.set_gains(kp=kp, ki=ki, kd=kd, k=0, b=0, ff=ff)
        self._has_gains = True


# @dataclass(init=False)
# class CONSTANTVALS:
#     """
#     Passes through all the values and construct a unified class for easy use
#     """

#     def __init__(self) -> None:
#         self.CONTROL_MODE: ControlModesInput = ControlModesInput(
#             fxe.FX_VOLTAGE, fxe.FX_CURRENT, fxe.FX_POSITION, fxe.FX_IMPEDANCE
#         )
#         self.DEFAULT_POSITION_GAINS: ControlGains = ControlGains(
#             0,
#             0,
#             0,
#             0,
#             0,
#             0,
#         )
#         self.DEFAULT_CURRENT_GAINS: ControlGains = ControlGains(
#             0,
#             0,
#             0,
#             0,
#             0,
#             0,
#         )
#         self.DEFAULT_IMPEDANCE_GAINS: ControlGains = ControlGains(
#             0,
#             0,
#             0,
#             0,
#             0,
#             0,
#         )
#         self.DEFAULT_MECHANICAL_CONSTANTS: MECHECONSTS = MECHECONSTS(
#             0,
#             0,
#             0,
#             0,
#             0,
#             0,
#         )

#     def __repr__(self) -> str:
#         return f"CONSTANTVALS"


@dataclass
class ActuatorCommunication:
    """Data strcture for the Actuator Definition & Communication

    Args:
        ABC (_type_): _description_
    """

    name: str = "DephyActpack"
    port: str = "/dev/ttyACM0"
    baud_rate: int = 230400
    frequency: int = 500
    logger: Logger = Logger()
    debug_level: int = 0
    log: bool = False


@dataclass
class ActuatorSensor:
    batt_volt: int = 30
    batt_curr: int = 0
    mot_volt: int = 0
    mot_cur: int = 0
    mot_ang: int = 0
    ank_ang: int = 0
    mot_vel: int = 0
    mot_acc: int = 0
    ank_vel: int = 0
    temperature: int = 25
    genvar_0: int = 0
    genvar_1: int = 0
    genvar_2: int = 0
    genvar_3: int = 0
    genvar_4: int = 0
    genvar_5: int = 0
    accelx: int = 0
    accely: int = 0
    accelz: int = 0
    gyrox: int = 0
    gyroy: int = 0
    gyroz: int = 0

    def __repr__(self):
        return f"ActuatorSensorData"


@dataclass
class ActuatorInitialization:
    state = None

    encoder_map = None

    motor_zero_position = 0.0
    joint_zero_position = 0.0

    joint_offset = 0.0
    motor_offset = 0.0

    joint_direction = 1.0

    thermal_model: ThermalModel = ThermalModel(
        temp_limit_windings=80,
        soft_border_C_windings=10,
        temp_limit_case=70,
        soft_border_C_case=10,
    )
    thermal_scale: float = 1.0


@dataclass
class ActuatorData:
    Gains: ControlGains
    MecheConsts: MECHECONSTS
    Communication: ActuatorCommunication
    Sensor: ActuatorSensor
    InitValue: ActuatorInitialization


class ActuatorObject(ActuatorData, ActuatorMode, ABC):
    def __init__(self, OperationMode: str) -> None:
        self._OperationMode: str = OperationMode
        self._ActuatorData = ActuatorData(
            ControlGains(),
            MECHECONSTS(),
            ActuatorCommunication(),
            ActuatorSensor(),
            ActuatorInitialization(),
        )
        self._default_control_mode: ActuatorMode = ActuatorMode("voltage", device=self)
        self._log = self._ActuatorData.Communication.logger
        self._data = self._ActuatorData.Sensor

    # Overrides the open method to function without a device
    def __repr__(self) -> str:
        return f"DephyActpack[{self._ActuatorData.Communication.name}]"

    def open(self, freq, log_level, log_enabled):
        self._log.debug(
            msg=f"[{self.__repr__()}] Opening Device at {self._ActuatorData.Communication.port}"
        )
        self.is_streaming = True

    # Overrides the send_motor_command method to set the new _motor_command attribute
    # @abstractmethod
    # def send_motor_command(self, ctrl_mode, value):
    #     self._motor_command = (
    #         f"[{self.__repr__()}] Control Mode: {ctrl_mode}, Value: {value}"
    #     )

    # Overrides the set_gains method to set the gains in the new _gains attribute

    def set_gains(self, kp, ki, kd, k, b, ff):
        self._gains["kp"] = kp
        self._gains["ki"] = ki
        self._gains["kd"] = kd
        self._gains["k"] = k
        self._gains["b"] = b
        self._gains["ff"] = ff

    # Overrides the read method to modify the data incrementally instead of through a device data stream

    def read(self):
        if self._OperationMode == "test":
            small_noise = np.random.normal(0, 0.01)
            self._data.batt_volt += small_noise
            self._data.batt_curr += 0.0
            self._data.mot_volt += 0.0
            self._data.mot_cur += 0.0
            self._data.mot_ang += 0.0
            self._data.ank_ang += 0.0
            self._data.mot_vel += small_noise
            self._data.mot_acc += small_noise
            self._data.ank_vel += small_noise
            self._data.temperature += small_noise
            self._data.genvar_0 += 0.0
            self._data.genvar_1 += 0.0
            self._data.genvar_2 += 0.0
            self._data.genvar_3 += 0.0
            self._data.genvar_4 += 0.0
            self._data.genvar_5 += 0.0
            self._data.accelx += small_noise
            self._data.accely += small_noise
            self._data.accelz += small_noise
            self._data.gyrox += small_noise
            self._data.gyroy += small_noise
            self._data.gyroz += small_noise
            return self._data
        else:
            # Should raise an OS error here.
            pass

    # Overrides the close method to do nothing

    def close(self):
        pass


if __name__ == "__main__":
    # To be erased after all tests are finished
    class Actpack(ActuatorObject):
        def __init__(self, OperationMode: str) -> None:
            super().__init__(OperationMode)

    testround = Actpack(OperationMode="test")
    val = testround.read()
