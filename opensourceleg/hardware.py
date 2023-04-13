#!/usr/bin/python3
from typing import Any, Callable, Dict, List, Optional

import collections
import csv
import ctypes as c
import logging
import os
import sys
import threading
import time
import traceback
from dataclasses import dataclass
from enum import Enum
from logging.handlers import RotatingFileHandler
from math import isfinite

import flexsea.fx_enums as fxe
import numpy as np
from flexsea.device import Device
from smbus2 import SMBus

sys.path.append("../")

from opensourceleg.utilities import SoftRealtimeLoop

# TODO: Support for TMotor driver with similar structure
# TODO: Support for gRPC servers

MOTOR_COUNT_PER_REV = 16384
NM_PER_AMP = 0.1133
NM_PER_MILLIAMP = NM_PER_AMP / 1000
RAD_PER_COUNT = 2 * np.pi / MOTOR_COUNT_PER_REV
RAD_PER_DEG = np.pi / 180
MOTOR_COUNT_TO_RADIANS = lambda x: x * (np.pi / 180.0 / 45.5111)
RADIANS_TO_MOTOR_COUNTS = lambda q: q * (180 * 45.5111 / np.pi)

RAD_PER_SEC_GYROLSB = np.pi / 180 / 32.8
M_PER_SEC_SQUARED_ACCLSB = 9.80665 / 8192

IMPEDANCE_A = 0.00028444
IMPEDANCE_C = 0.0007812

NM_PER_RAD_TO_K = RAD_PER_COUNT / IMPEDANCE_C * 1e3 / NM_PER_AMP
NM_S_PER_RAD_TO_B = RAD_PER_DEG / IMPEDANCE_A * 1e3 / NM_PER_AMP

CURRENT_LIMIT = 8000

# Global Units Dictionary
ALL_UNITS = {
    "force": {
        "N": 1.0,
        "lbf": 4.4482216152605,
        "kgf": 9.80665,
    },
    "torque": {
        "N-m": 1.0,
        "lbf-in": 0.1129848290276167,
        "lbf-ft": 1.3558179483314004,
        "kgf-cm": 0.0980665,
        "kgf-m": 0.980665,
    },
    "stiffness": {
        "N/rad": 1.0,
        "N/deg": 0.017453292519943295,
        "lbf/rad": 0.224809,
        "lbf/deg": 0.003490659,
        "kgf/rad": 1.8518518518518519,
        "kgf/deg": 0.031746031746031744,
    },
    "damping": {
        "N/(rad/s)": 1.0,
        "N/(deg/s)": 0.017453292519943295,
        "lbf/(rad/s)": 0.224809,
        "lbf/(deg/s)": 0.003490659,
        "kgf/(rad/s)": 1.8518518518518519,
        "kgf/(deg/s)": 0.031746031746031744,
    },
    "length": {
        "m": 1.0,
        "cm": 0.01,
        "in": 0.0254,
        "ft": 0.3048,
    },
    "position": {
        "rad": 1.0,
        "deg": 0.017453292519943295,
    },
    "mass": {
        "kg": 1.0,
        "g": 0.001,
        "lb": 0.45359237,
    },
    "velocity": {
        "rad/s": 1.0,
        "deg/s": 0.017453292519943295,
        "rpm": 0.10471975511965977,
    },
    "acceleration": {
        "rad/s^2": 1.0,
        "deg/s^2": 0.017453292519943295,
    },
    "time": {
        "s": 1.0,
        "ms": 0.001,
        "us": 0.000001,
    },
    "current": {
        "mA": 1,
        "A": 1000,
    },
    "voltage": {
        "mV": 1,
        "V": 1000,
    },
    "gravity": {
        "m/s^2": 1.0,
        "g": 9.80665,
    },
}


class UnitsDefinition(dict):
    """
    UnitsDefinition class is a dictionary with set and get methods that checks if the keys are valid

    Methods:
        __setitem__(key: str, value: dict) -> None
        __getitem__(key: str) -> dict
        convert(value: float, attribute: str) -> None
    """

    def __setitem__(self, key: str, value: dict) -> None:
        if key not in self:
            raise KeyError(f"Invalid key: {key}")

        if value not in ALL_UNITS[key].keys():
            raise ValueError(f"Invalid unit: {value}")

        super().__setitem__(key, value)

    def __getitem__(self, key: str) -> dict:
        if key not in self:
            raise KeyError(f"Invalid key: {key}")
        return super().__getitem__(key)

    def convert_to_default_units(self, value: float, attribute: str) -> None:
        """
        convert a value from one unit to the default unit

        Args:
            value (float): Value to be converted
            attribute (str): Attribute to be converted

        Returns:
            float: Converted value in the default unit
        """
        return value * ALL_UNITS[attribute][self[attribute]]

    def convert_from_default_units(self, value: float, attribute: str) -> None:
        """
        convert a value from the default unit to another unit

        Args:
            value (float): Value to be converted
            attribute (str): Attribute to be converted

        Returns:
            float: Converted value in the default unit
        """
        return value / ALL_UNITS[attribute][self[attribute]]


DEFAULT_UNITS = UnitsDefinition(
    {
        "force": "N",
        "torque": "N-m",
        "stiffness": "N/rad",
        "damping": "N/(rad/s)",
        "length": "m",
        "position": "rad",
        "mass": "kg",
        "velocity": "rad/s",
        "acceleration": "rad/s^2",
        "time": "s",
        "current": "mA",
        "voltage": "mV",
        "gravity": "m/s^2",
    }
)


@dataclass
class Gains:
    """
    Dataclass for controller gains

    Attributes:
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


DEFAULT_POSITION_GAINS = Gains(kp=50, ki=0, kd=0, K=0, B=0, ff=0)

DEFAULT_CURRENT_GAINS = Gains(kp=40, ki=400, kd=0, K=0, B=0, ff=128)

DEFAULT_IMPEDANCE_GAINS = Gains(kp=40, ki=400, kd=0, K=200, B=400, ff=128)


@dataclass
class ControlModes:
    voltage = fxe.FX_VOLTAGE
    current = fxe.FX_CURRENT
    position = fxe.FX_POSITION
    impedance = fxe.FX_IMPEDANCE


CONTROL_MODE = ControlModes()


class Logger(logging.Logger):
    """
    Logger class is a class that logs attributes from a class to a csv file

    Methods:
        __init__(self, class_instance: object, file_path: str, logger: logging.Logger = None) -> None
        log(self) -> None
    """

    def __init__(
        self,
        file_path: str,
        log_format: str = "[%(asctime)s] %(levelname)s: %(message)s",
    ) -> None:

        self._file_path = file_path

        self._class_instances = []
        self._attributes = []

        self._file = open(self._file_path + ".csv", "w")
        self._writer = csv.writer(self._file)
        self._writer.writerow(self._attributes)

        self._log_levels = {
            "DEBUG": logging.DEBUG,
            "INFO": logging.INFO,
            "WARNING": logging.WARNING,
            "ERROR": logging.ERROR,
            "CRITICAL": logging.CRITICAL,
        }

        super().__init__(__name__)
        self.setLevel(logging.DEBUG)

        self._std_formatter = logging.Formatter(log_format)

        self._file_handler = RotatingFileHandler(
            self._file_path,
            mode="w",
            maxBytes=0,
            backupCount=10,
        )
        self._file_handler.setLevel(logging.DEBUG)
        self._file_handler.setFormatter(self._std_formatter)

        self._stream_handler = logging.StreamHandler()
        self._stream_handler.setLevel(logging.INFO)
        self._stream_handler.setFormatter(self._std_formatter)

        self.addHandler(self._stream_handler)
        self.addHandler(self._file_handler)

        self._is_logging = False

    def set_file_level(self, level: str = "DEBUG") -> None:
        """
        Sets the level of the logger

        Args:
            level (str): Level of the logger
        """
        if level not in self._log_levels.keys():
            self.warning(f"Invalid logging level: {level}")

        self._file_handler.setLevel(self._log_levels[level])

    def set_stream_level(self, level: str = "INFO") -> None:
        """
        Sets the level of the logger

        Args:
            level (str): Level of the logger
        """
        if level not in self._log_levels.keys():
            self.warning(f"Invalid logging level: {level}")

        self._stream_handler.setLevel(self._log_levels[level])

    def add_attributes(self, class_instance: object, attributes_str: list[str]) -> None:
        """
        Configures the logger to log the attributes of a class

        Args:
            class_instance (object): Class instance to log the attributes of
            attributes_str (list[str]): List of attributes to log
        """
        self._class_instances.append(class_instance)
        self._attributes.append(attributes_str)

    def data(self) -> None:
        """
        Logs the attributes of the class instance to the csv file
        """

        if not self._is_logging:
            for class_instance, attributes in zip(
                self._class_instances, self._attributes
            ):
                self._writer.writerow(
                    [
                        f"{class_instance.__class__.__name__}: {attribute}"
                        for attribute in attributes
                    ]
                )
            self._is_logging = True

        for class_instance, attributes in zip(self._class_instances, self._attributes):
            self._writer.writerow(
                [getattr(class_instance, attribute) for attribute in attributes]
            )

        self._file.flush()

    def close(self) -> None:
        """
        Closes the csv file
        """
        self._file.close()


class ActpackMode:
    def __init__(self, control_mode, device: "DephyActpack"):
        self._control_mode = control_mode
        self._device = device
        self._entry_callback: callable = lambda: None
        self._exit_callback: callable = lambda: None

        self._has_gains = False

    def __eq__(self, __o: object) -> bool:
        if isinstance(__o, ActpackMode):
            return self._control_mode == __o._control_mode
        return False

    def __str__(self) -> str:
        return str(self._control_mode)

    @property
    def mode(self):
        return self._control_mode

    @property
    def has_gains(self) -> bool:
        return self._has_gains

    def enter(self):
        self._entry_callback()

    def exit(self):
        self._exit_callback()

    def transition(self, to_state: "ActpackMode"):
        self.exit()
        to_state.enter()


class VoltageMode(ActpackMode):
    def __init__(self, device: "DephyActpack"):
        super().__init__(CONTROL_MODE.voltage, device)
        self._entry_callback = self._entry
        self._exit_callback = self._exit

    def _entry(self):
        self._device._log.debug(f"[Actpack] Entering Voltage mode.")

    def _exit(self):
        self._device._log.debug(f"[Actpack] Exiting Voltage mode.")
        self._set_voltage(0)
        time.sleep(0.1)

    def _set_voltage(self, voltage: int):
        self._device.send_motor_command(
            self.mode,
            voltage,
        )


class CurrentMode(ActpackMode):
    def __init__(self, device: "DephyActpack"):
        super().__init__(CONTROL_MODE.current, device)
        self._entry_callback = self._entry
        self._exit_callback = self._exit

    def _entry(self):
        self._device._log.debug(f"[Actpack] Entering Current mode.")

        if not self.has_gains:
            self._set_gains()

        self._set_current(0)

    def _exit(self):
        self._device._log.debug(f"[Actpack] Exiting Current mode.")
        self._device.send_motor_command(CONTROL_MODE.voltage, 0)
        time.sleep(1 / self._device.frequency)

    def _set_gains(
        self,
        kp: int = DEFAULT_CURRENT_GAINS.kp,
        ki: int = DEFAULT_CURRENT_GAINS.ki,
        ff: int = DEFAULT_CURRENT_GAINS.ff,
    ):

        assert 0 <= kp <= 80, "kp must be between 0 and 80"
        assert 0 <= ki <= 800, "ki must be between 0 and 800"
        assert 0 <= ff <= 128, "ff must be between 0 and 128"

        self._device.set_gains(kp=kp, ki=ki, kd=0, k=0, b=0, ff=ff)
        self._has_gains = True

        time.sleep(0.1)

    def _set_current(self, current: int):
        """Sets the Q-axis current of the motor

        Args:
            current (int): _description_
        """
        self._device.send_motor_command(
            self.mode,
            current,
        )


class PositionMode(ActpackMode):
    def __init__(self, device: "DephyActpack"):
        super().__init__(CONTROL_MODE.position, device)
        self._entry_callback = self._entry
        self._exit_callback = self._exit

    def _entry(self):
        self._device._log.debug(f"[Actpack] Entering Position mode.")

        if not self.has_gains:
            self._set_gains()

        self._set_motor_position(
            int(
                self._device._units.convert_to_default_units(
                    self._device.motor_position, "position"
                )
                / RAD_PER_COUNT
            )
        )

    def _exit(self):
        self._device._log.debug(f"[Actpack] Exiting Position mode.")
        self._device.send_motor_command(CONTROL_MODE.voltage, 0)
        time.sleep(0.1)

    def _set_gains(
        self,
        kp: int = DEFAULT_POSITION_GAINS.kp,
        ki: int = DEFAULT_POSITION_GAINS.ki,
        kd: int = DEFAULT_POSITION_GAINS.kd,
    ):

        assert 0 <= kp <= 1000, "kp must be between 0 and 1000"
        assert 0 <= ki <= 1000, "ki must be between 0 and 1000"
        assert 0 <= kd <= 1000, "kd must be between 0 and 1000"

        self._device.set_gains(kp=kp, ki=ki, kd=kd, k=0, b=0, ff=0)
        self._has_gains = True

        time.sleep(1 / self._device.frequency)

    def _set_motor_position(self, counts: int):
        """Sets the motor position

        Args:
            counts (int): position in counts
        """
        self._device.send_motor_command(
            self.mode,
            counts,
        )


class ImpedanceMode(ActpackMode):
    def __init__(self, device: "DephyActpack"):
        super().__init__(CONTROL_MODE.impedance, device)
        self._entry_callback = self._entry
        self._exit_callback = self._exit

    def _entry(self):
        self._device._log.debug(f"[Actpack] Entering Impedance mode.")
        if not self.has_gains:
            self._set_gains()

        self._set_motor_position(
            int(
                self._device._units.convert_to_default_units(
                    self._device.motor_position, "position"
                )
                / RAD_PER_COUNT
            )
        )

    def _exit(self):
        self._device._log.debug(f"[Actpack] Exiting Impedance mode.")
        self._device.send_motor_command(CONTROL_MODE.voltage, 0)
        time.sleep(1 / self._device.frequency)

    def _set_motor_position(self, counts: int):
        """Sets the motor position

        Args:
            counts (int): position in counts
        """
        self._device.send_motor_command(
            self.mode,
            counts,
        )

    def _set_gains(
        self,
        kp: int = DEFAULT_IMPEDANCE_GAINS.kp,
        ki: int = DEFAULT_IMPEDANCE_GAINS.ki,
        K: int = DEFAULT_IMPEDANCE_GAINS.K,
        B: int = DEFAULT_IMPEDANCE_GAINS.B,
        ff: int = DEFAULT_IMPEDANCE_GAINS.ff,
    ):

        assert 0 <= kp <= 80, "kp must be between 0 and 80"
        assert 0 <= ki <= 800, "ki must be between 0 and 800"
        assert 0 <= ff <= 128, "ff must be between 0 and 128"
        assert 0 <= K, "K must be greater than 0"
        assert 0 <= B, "B must be greater than 0"

        self._device.set_gains(kp=kp, ki=ki, kd=0, k=K, b=B, ff=ff)
        self._has_gains = True

        time.sleep(1 / self._device.frequency)


class DephyActpack(Device):
    """Class for the Dephy Actpack

    Args:
        Device (_type_): _description_

    Raises:
        KeyError: _description_
        ValueError: _description_
        KeyError: _description_

    Returns:
        _type_: _description_
    """

    def __init__(
        self,
        port: str = "/dev/ttyACM0",
        baud_rate: int = 230400,
        frequency: int = 500,
        logger: Logger = None,
        units: UnitsDefinition = DEFAULT_UNITS,
        debug_level: int = 0,
        dephy_log: bool = False,
    ) -> None:
        """
        Initializes the Actpack class

        Args:
            port (str): _description_
            baud_rate (int): _description_. Defaults to 230400.
            frequency (int): _description_. Defaults to 500.
            logger (Logger): _description_
            units (UnitsDefinition): _description_
            debug_level (int): _description_. Defaults to 0.
            dephy_log (bool): _description_. Defaults to False.
        """
        super().__init__(port, baud_rate)
        self._debug_level = debug_level
        self._dephy_log = dephy_log
        self._frequency = frequency
        self._data: dict = None
        self._log = logger
        self._state = None
        self._units = units if units else DEFAULT_UNITS

        self._motor_zero_position = 0.0
        self._joint_zero_position = 0.0

        self._modes: dict[str, ActpackMode] = {
            "voltage": VoltageMode(self),
            "position": PositionMode(self),
            "current": CurrentMode(self),
            "impedance": ImpedanceMode(self),
        }

        self._mode: ActpackMode = self._modes["voltage"]

    def start(self):
        self.open(self._frequency, self._debug_level, log_enabled=self._dephy_log)
        time.sleep(0.1)
        self._data = self.read()
        self._mode.enter()

    def stop(self):
        self.set_mode("voltage")
        self.set_voltage(0, force=True)

        time.sleep(0.1)
        self.close()

    def update(self):
        if self.is_streaming:
            self._data = self.read()
        else:
            self._log.warning(
                f"[Actpack] Please open() the device before streaming data."
            )

    def set_mode(self, mode: str):
        if mode in self._modes:
            self._mode.transition(self._modes[mode])
            self._mode = self._modes[mode]

        else:
            self._log.warning(f"Mode {mode} not found")
            return

    def set_motor_zero_position(self, position: float):
        self._motor_zero_position = position

    def set_joint_zero_position(self, position: float):
        self._joint_zero_position = position

    def set_position_gains(
        self,
        kp: int = DEFAULT_POSITION_GAINS.kp,
        ki: int = DEFAULT_POSITION_GAINS.ki,
        kd: int = DEFAULT_POSITION_GAINS.kd,
        force: bool = True,
    ):

        """
        Sets the position gains

        Args:
            kp (int): The proportional gain
            ki (int): The integral gain
            kd (int): The derivative gain
            force (bool): Force the mode transition. Defaults to False.
        """
        if self._mode != self._modes["position"]:
            self._log.warning(f"Cannot set position gains in mode {self._mode}")
            return

        self._mode._set_gains(kp=kp, ki=ki, kd=kd)

    def set_current_gains(
        self,
        kp: int = DEFAULT_CURRENT_GAINS.kp,
        ki: int = DEFAULT_CURRENT_GAINS.ki,
        ff: int = DEFAULT_CURRENT_GAINS.ff,
        force: bool = True,
    ):

        """
        Sets the current gains

        Args:
            kp (int): The proportional gain
            ki (int): The integral gain
            ff (int): The feedforward gain
            force (bool): Force the mode transition. Defaults to False.
        """
        if self._mode != self._modes["current"]:
            self._log.warning(f"Cannot set current gains in mode {self._mode}")
            return

        self._mode._set_gains(kp=kp, ki=ki, ff=ff)

    def set_impedance_gains(
        self,
        kp: int = DEFAULT_IMPEDANCE_GAINS.kp,
        ki: int = DEFAULT_IMPEDANCE_GAINS.ki,
        K: int = DEFAULT_IMPEDANCE_GAINS.K,
        B: int = DEFAULT_IMPEDANCE_GAINS.B,
        ff: int = DEFAULT_IMPEDANCE_GAINS.ff,
        force: bool = True,
    ):
        """
        Sets the impedance gains

        Args:
            kp (int): The proportional gain
            ki (int): The integral gain
            K (int): The spring constant
            B (int): The damping constant
            ff (int): The feedforward gain
            force (bool): Force the mode transition. Defaults to False.
        """
        if self._mode != self._modes["impedance"]:
            self._log.warning(f"Cannot set impedance gains in mode {self._mode}")
            return

        self._mode._set_gains(kp=kp, ki=ki, K=K, B=B, ff=ff)

    def set_voltage(self, value: float, force: bool = False):
        """
        Sets the q axis voltage

        Args:
            value (float): The voltage to set
            force (bool): Force the mode transition. Defaults to False.
        """
        if self._mode != self._modes["voltage"]:
            self._log.warning(f"Cannot set voltage in mode {self._mode}")
            return

        self._mode._set_voltage(
            int(self._units.convert_to_default_units(value, "voltage")),
        )

    def set_current(self, value: float, force: bool = False):
        """
        Sets the q axis current

        Args:
            value (float): The current to set
            force (bool): Force the mode transition. Defaults to False.
        """
        if self._mode != self._modes["current"]:
            self._log.warning(f"Cannot set current in mode {self._mode}")
            return

        self._mode._set_current(
            int(self._units.convert_to_default_units(value, "current")),
        )

    def set_motor_torque(self, torque: float, force: bool = False):
        """
        Sets the motor torque

        Args:
            torque (float): The torque to set
            force (bool): Force the mode transition. Defaults to False.
        """
        if self._mode != self._modes["current"]:
            self._log.warning(f"Cannot set motor_torque in mode {self._mode}")
            return

        self._mode._set_current(
            int(
                self._units.convert_to_default_units(torque, "torque") / NM_PER_MILLIAMP
            ),
        )

    def set_motor_position(self, position: float):
        """
        Sets the motor position

        Args:
            position (float): The position to set
        """
        if self._mode not in [self._modes["position"], self._modes["impedance"]]:
            self._log.warning(f"Cannot set motor position in mode {self._mode}")
            return

        self._mode._set_motor_position(
            int(
                self._units.convert_to_default_units(position, "position")
                / RAD_PER_COUNT
            ),
        )

    # Read only properties from the actpack

    @property
    def units(self):
        return self._units

    @property
    def frequency(self):
        return self._frequency

    @property
    def mode(self):
        return self._mode

    @property
    def motor_zero_position(self):
        return self._motor_zero_position

    @property
    def joint_zero_position(self):
        return self._joint_zero_position

    @property
    def battery_voltage(self):
        return self._units.convert_from_default_units(
            self._data.batt_volt,
            "voltage",
        )

    @property
    def batter_current(self):
        return self._units.convert_from_default_units(
            self._data.batt_curr,
            "current",
        )

    @property
    def motor_voltage(self):
        return self._units.convert_from_default_units(
            self._data.mot_volt,
            "voltage",
        )

    @property
    def motor_current(self):
        return self._units.convert_from_default_units(
            self._data.mot_cur,
            "current",
        )

    @property
    def motor_torque(self):
        return self._units.convert_from_default_units(
            self._data.mot_cur * NM_PER_MILLIAMP,
            "torque",
        )

    @property
    def motor_position(self):
        return self._units.convert_from_default_units(
            int(self._data.mot_ang) * RAD_PER_COUNT,
            "position",
        )

    @property
    def motor_velocity(self):
        return self._units.convert_from_default_units(
            self._data.mot_vel * RAD_PER_DEG,
            "velocity",
        )

    @property
    def motor_acceleration(self):
        return self._units.convert_from_default_units(
            self._data.mot_acc,
            "acceleration",
        )

    @property
    def motor_torque(self):
        return self._units.convert_from_default_units(
            self.motor_current * NM_PER_AMP,
            "torque",
        )

    @property
    def joint_position(self):
        return self._units.convert_from_default_units(
            self._data.ank_ang * RAD_PER_COUNT,
            "position",
        )

    @property
    def joint_velocity(self):
        return self._units.convert_from_default_units(
            self._data.ank_vel * RAD_PER_COUNT,
            "velocity",
        )

    @property
    def genvars(self):
        return np.array(
            [
                self._data.genvar_0,
                self._data.genvar_1,
                self._data.genvar_2,
                self._data.genvar_3,
                self._data.genvar_4,
                self._data.genvar_5,
            ]
        )

    @property
    def acc_x(self):
        return self._units.convert_from_default_units(
            self._data.accelx * M_PER_SEC_SQUARED_ACCLSB,
            "gravity",
        )

    @property
    def acc_y(self):
        return self._units.convert_from_default_units(
            self._data.accely * M_PER_SEC_SQUARED_ACCLSB,
            "gravity",
        )

    @property
    def acc_z(self):
        return self._units.convert_from_default_units(
            self._data.accelz * M_PER_SEC_SQUARED_ACCLSB,
            "gravity",
        )

    @property
    def gyro_x(self):
        return self._units.convert_from_default_units(
            self._data.gyrox * RAD_PER_SEC_GYROLSB,
            "velocity",
        )

    @property
    def gyro_y(self):
        return self._units.convert_from_default_units(
            self._data.gyroy * RAD_PER_SEC_GYROLSB,
            "velocity",
        )

    @property
    def gyro_z(self):
        return self._units.convert_from_default_units(
            self._data.gyroz * RAD_PER_SEC_GYROLSB,
            "velocity",
        )


class Joint(DephyActpack):
    def __init__(
        self,
        name: str = "knee",
        port: str = "/dev/ttyACM0",
        baud_rate: int = 230400,
        frequency: int = 500,
        gear_ratio: float = 1.0,
        has_loadcell: bool = False,
        logger: Logger = None,
        units: UnitsDefinition = DEFAULT_UNITS,
        debug_level: int = 0,
        dephy_log: bool = False,
    ) -> None:

        super().__init__(
            port=port,
            baud_rate=baud_rate,
            frequency=frequency,
            logger=logger,
            units=units,
            debug_level=debug_level,
            dephy_log=dephy_log,
        )

        self._gear_ratio: float = gear_ratio
        self._is_homed: bool = False
        self._has_loadcell: bool = has_loadcell
        self._encoder_map = None
        self._zero_pos = 0.0

        self._motor_zero_pos = 0.0
        self._joint_zero_pos = 0.0

        self._motor_voltage_sp = 0.0
        self._motor_current_sp = 0.0
        self._motor_position_sp = 0.0

        self._stiffness_sp: int = 200
        self._damping_sp: int = 400
        self._equilibrium_position_sp = 0.0

        self._control_mode_sp: str = "voltage"

        if "knee" in name.lower() or "ankle" in name.lower():
            self._name = name
        else:
            self._log.warning(f"Invalid joint name: {name}")
            return

        if os.path.isfile(f"./{self._name}_encoder_map.npy"):
            coefficients = np.load(f"./{self._name}_encoder_map.npy")
            self._encoder_map = np.polynomial.polynomial.Polynomial(coefficients)
        else:
            self._log.debug(
                f"[{self._name}] No encoder map found. Please run the calibration routine."
            )

    def home(
        self,
        homing_voltage: int = 2000,
        homing_frequency: int = 100,
    ):

        is_homing = True

        CURRENT_THRESHOLD = 6000
        VELOCITY_THRESHOLD = 0.001

        self.set_mode("voltage")
        self.set_voltage(-1 * homing_voltage)  # mV, negative for counterclockwise

        _motor_encoder_array = []
        _joint_encoder_array = []

        time.sleep(0.1)

        try:
            while is_homing:
                self.update()
                time.sleep(1 / homing_frequency)

                _motor_encoder_array.append(self.motor_position)
                _joint_encoder_array.append(self.joint_position)

                if (
                    abs(self.output_velocity) <= VELOCITY_THRESHOLD
                    or abs(self.motor_current) >= CURRENT_THRESHOLD
                ):
                    self.set_voltage(0)
                    is_homing = False

        except KeyboardInterrupt:
            self.set_voltage(0)
            self._log.warning("Homing interrupted.")
            return

        _motor_zero_pos = self.motor_position
        _joint_zero_pos = self.joint_position

        time.sleep(0.1)

        if np.std(_motor_encoder_array) < 1e-6:
            self._log.warning(
                f"[{self._name}] Motor encoder not working. Please check the wiring."
            )
            return

        elif np.std(_joint_encoder_array) < 1e-6:
            self._log.warning(
                f"[{self._name}] Joint encoder not working. Please check the wiring."
            )
            return

        if "ankle" in self._name.lower():
            self._zero_pos = np.deg2rad(30)
            self.set_motor_zero_position(_motor_zero_pos)
            self.set_joint_zero_position(_joint_zero_pos)

        else:
            self._zero_pos = 0.0
            self.set_motor_zero_position(_motor_zero_pos)
            self.set_joint_zero_position(_joint_zero_pos)

        self._is_homed = True

        # if self.encoder_map is None:
        #     if (
        #         input(f"[{self._name}] Would you like to make an encoder map? (y/n): ")
        #         == "y"
        #     ):
        #         self.make_encoder_map()

    def make_encoder_map(self):
        """
        This method makes a lookup table to calculate the position measured by the joint encoder.
        This is necessary because the magnetic output encoders are nonlinear.
        By making the map while the joint is unloaded, joint position calculated by motor position * gear ratio
        should be the same as the true joint position.

        Output from this function is a file containing a_i values parameterizing the map

        Eqn: position = sum from i=0^5 (a_i*counts^i)

        Author: Kevin Best, PhD
                U-M Neurobionics Lab
                Gitub: tkevinbest, https://github.com/tkevinbest
        """

        if not self.is_homed:
            self._log.warning(
                f"[{self._name}] Please home the joint before making the encoder map."
            )
            return

        self.set_mode("current")
        self.set_current_gains()
        time.sleep(0.1)
        self.set_current_gains()

        self.set_output_torque(0.0)
        time.sleep(0.1)
        self.set_output_torque(0.0)

        _joint_position_array = []
        _output_position_array = []

        self._log.info(
            f"[{self._name}] Please manually move the joint numerous times \
                through its full range of motion for 10 seconds.\
                   \n Press any key to continue."
        )

        _start_time = time.time()

        try:
            while time.time() - _start_time < 10:
                self.update()
                _joint_position_array.append(self.joint_position)
                _output_position_array.append(self.output_position)

                time.sleep(1 / self.frequency)

        except KeyboardInterrupt:
            self._log.warning("Encoder map interrupted.")
            return

        self._log.info(f"[{self._name}] You may now stop moving the joint.")

        _power = np.arange(4.0)
        _a_mat = np.array(_joint_position_array).reshape(-1, 1) ** _power
        _beta = np.linalg.lstsq(_a_mat, _output_position_array, rcond=None)[0]
        _coeffs = _beta[0]

        self._encoder_map = np.polynomial.polynomial.Polynomial(_coeffs)

        np.save(f"./{self._name}_encoder_map.npy", _coeffs)
        self._log.info(f"[{self._name}] Encoder map saved.")

    def set_output_torque(self, torque: float):
        """
        Set the output torque of the joint.
        This is the torque that is applied to the joint, not the motor.

        Args:
            torque (float): torque in user-defined units
        """
        self.set_motor_torque(torque / self.gear_ratio)

    def set_output_position(self, position: float):
        """
        Set the output position of the joint.
        This is the desired position of the joint, not the motor.

        Args:
            position (float): position in user-defined units
        """
        self.set_motor_position(position * self.gear_ratio)

    def set_motor_impedance(
        self,
        kp: int = 40,
        ki: int = 400,
        K: float = 0.08922,
        B: float = 0.0038070,
        ff: int = 128,
    ):
        """
        Set the impedance gains of the motor in real units: Nm/rad and Nm/rad/s.

        Args:
            kp (int): Proportional gain. Defaults to 40.
            ki (int): Integral gain. Defaults to 400.
            K (float): Spring constant. Defaults to 0.08922 Nm/rad.
            B (float): Damping constant. Defaults to 0.0038070 Nm/rad/s.
            ff (int): Feedforward gain. Defaults to 128.
        """
        self.set_impedance_gains(
            kp=kp,
            ki=ki,
            K=int(K * NM_PER_RAD_TO_K),
            B=int(B * NM_S_PER_RAD_TO_B),
            ff=ff,
        )

    def set_joint_impedance(
        self,
        kp: int = 40,
        ki: int = 400,
        K: float = 0.08922,
        B: float = 0.0038070,
        ff: int = 128,
    ):
        """
        Set the impedance gains of the joint in real units: Nm/rad and Nm/rad/s.

        Conversion:
            K_motor = K_joint / (gear_ratio ** 2)
            B_motor = B_joint / (gear_ratio ** 2)

        Args:
            kp (int): Proportional gain. Defaults to 40.
            ki (int): Integral gain. Defaults to 400.
            K (float): Spring constant. Defaults to 0.08922 Nm/rad.
            B (float): Damping constant. Defaults to 0.0038070 Nm/rad/s.
            ff (int): Feedforward gain. Defaults to 128.
        """
        self.set_motor_impedance(
            kp=kp,
            ki=ki,
            K=K / (self.gear_ratio**2),
            B=B / (self.gear_ratio**2),
            ff=ff,
        )

    def update_set_points(self):
        self.set_mode(self.control_mode_sp)

    @property
    def zero_position(self):
        return self._zero_pos

    @zero_position.setter
    def zero_position(self, value):
        self._zero_pos = value

    @property
    def name(self):
        return self._name

    @property
    def gear_ratio(self):
        return self._gear_ratio

    @property
    def is_homed(self):
        return self._is_homed

    @property
    def encoder_map(self):
        return self._encoder_map

    @property
    def output_position(self):
        return self.motor_position / self.gear_ratio

    @property
    def output_velocity(self):
        return self.motor_velocity / self.gear_ratio

    @property
    def joint_torque(self):
        return self.motor_torque * self.gear_ratio

    @property
    def motor_current_sp(self):
        return self._motor_current_sp

    @property
    def motor_voltage_sp(self):
        return self._motor_voltage_sp

    @property
    def motor_position_sp(self):
        return self._motor_position_sp

    @property
    def stiffness_sp(self):
        return self._stiffness_sp

    @property
    def damping_sp(self):
        return self._damping_sp

    @property
    def equilibirum_position_sp(self):
        return self._equilibrium_position_sp

    @property
    def control_mode_sp(self):
        return self._control_mode_sp


class StrainAmp:
    """
    A class to directly manage the 6ch strain gauge amplifier over I2C. GIVE MITRY CREDIT
    """

    # register numbers for the "ezi2c" interface on the strainamp
    # found in source code here: https://github.com/JFDuval/flexsea-strain/tree/dev
    MEM_R_CH1_H = 8
    MEM_R_CH1_L = 9
    MEM_R_CH2_H = 10
    MEM_R_CH2_L = 11
    MEM_R_CH3_H = 12
    MEM_R_CH3_L = 13
    MEM_R_CH4_H = 14
    MEM_R_CH4_L = 15
    MEM_R_CH5_H = 16
    MEM_R_CH5_L = 17
    MEM_R_CH6_H = 18
    MEM_R_CH6_L = 19

    def __init__(self, bus, I2C_addr=0x66) -> None:
        """Create a strainamp object, to talk over I2C"""
        # self._I2CMan = I2CManager(bus)
        self._SMBus = SMBus(bus)
        time.sleep(1)
        self.bus = bus
        self.addr = I2C_addr
        self.genvars = np.zeros((3, 6))
        self.indx = 0
        self.is_streaming = True
        self.data = []
        self.failed_reads = 0

    def read_uncompressed_strain(self):
        """Used for an older version of the strain amp firmware (at least pre-2017)"""
        data = []
        for i in range(self.MEM_R_CH1_H, self.MEM_R_CH6_L + 1):
            data.append(self._SMBus.read_byte_data(self.addr, i))

        return self.unpack_uncompressed_strain(data)

    def read_compressed_strain(self):
        """Used for more recent versions of strain amp firmware"""
        try:
            self.data = self._SMBus.read_i2c_block_data(self.addr, self.MEM_R_CH1_H, 10)
            self.failed_reads = 0
        except OSError as e:
            self.failed_reads += 1
            # print("\n read failed")
            if self.failed_reads >= 5:
                raise Exception("Load cell unresponsive.")
        # unpack them and return as nparray
        return self.unpack_compressed_strain(self.data)

    def update(self):
        """Called to update data of strain amp. Also returns data."""
        self.genvars[self.indx, :] = self.read_compressed_strain()
        self.indx = (self.indx + 1) % 3
        return np.median(self.genvars, axis=0)

    @staticmethod
    def unpack_uncompressed_strain(data):
        """Used for an older version of the strain amp firmware (at least pre-2017)"""
        ch1 = (data[0] << 8) | data[1]
        ch2 = (data[2] << 8) | data[3]
        ch3 = (data[4] << 8) | data[5]
        ch4 = (data[6] << 8) | data[7]
        ch5 = (data[8] << 8) | data[9]
        ch6 = (data[10] << 8) | data[11]
        return np.array([ch1, ch2, ch3, ch4, ch5, ch6])

    @staticmethod
    def unpack_compressed_strain(data):
        """Used for more recent versions of strainamp firmware"""
        # ch1 = (data[0] << 4) | ( (data[1] >> 4) & 0x0F)
        # ch2 = ( (data[1] << 8) & 0x0F00) | data[2]
        # ch3 = (data[3] << 4) | ( (data[4] >> 4) & 0x0F)
        # ch4 = ( (data[4] << 8) & 0x0F00) | data[5]
        # ch5 = (data[6] << 4) | ( (data[7] >> 4) & 0x0F)
        # ch6 = ( (data[7] << 8) & 0x0F00) | data[8]
        # moved into one line to save 0.02ms -- maybe pointless but eh
        return np.array(
            [
                (data[0] << 4) | ((data[1] >> 4) & 0x0F),
                ((data[1] << 8) & 0x0F00) | data[2],
                (data[3] << 4) | ((data[4] >> 4) & 0x0F),
                ((data[4] << 8) & 0x0F00) | data[5],
                (data[6] << 4) | ((data[7] >> 4) & 0x0F),
                ((data[7] << 8) & 0x0F00) | data[8],
            ]
        )

    @staticmethod
    def strain_data_to_wrench(
        unpacked_strain, loadcell_matrix, loadcell_zero, exc=5, gain=125
    ):
        """Converts strain values between 0 and 4095 to a wrench in N and Nm"""
        loadcell_signed = (unpacked_strain - 2048) / 4095 * exc
        loadcell_coupled = loadcell_signed * 1000 / (exc * gain)
        return np.reshape(
            np.transpose(loadcell_matrix.dot(np.transpose(loadcell_coupled)))
            - loadcell_zero,
            (6,),
        )

    @staticmethod
    def wrench_to_strain_data(measurement, loadcell_matrix, exc=5, gain=125):
        """Wrench in N and Nm to the strain values that would give that wrench"""
        loadcell_coupled = (np.linalg.inv(loadcell_matrix)).dot(measurement)
        loadcell_signed = loadcell_coupled * (exc * gain) / 1000
        return ((loadcell_signed / exc) * 4095 + 2048).round(0).astype(int)


class Loadcell:
    def __init__(
        self,
        dephy_mode: bool = False,
        joint: Joint = None,
        amp_gain: float = 125.0,
        exc: float = 5.0,
        loadcell_matrix: np.array = None,
        logger: "Logger" = None,
    ) -> None:
        self._is_dephy = dephy_mode
        self._joint = joint
        self._amp_gain = amp_gain
        self._exc = exc
        self._adc_range = 2**12 - 1
        self._offset = (2**12) / 2
        self._lc = None

        if not self._is_dephy:
            self._lc = StrainAmp(bus=1, I2C_addr=0x66)

        if not loadcell_matrix:
            self._loadcell_matrix = np.array(
                [
                    (-38.72600, -1817.74700, 9.84900, 43.37400, -44.54000, 1824.67000),
                    (-8.61600, 1041.14900, 18.86100, -2098.82200, 31.79400, 1058.6230),
                    (
                        -1047.16800,
                        8.63900,
                        -1047.28200,
                        -20.70000,
                        -1073.08800,
                        -8.92300,
                    ),
                    (20.57600, -0.04000, -0.24600, 0.55400, -21.40800, -0.47600),
                    (-12.13400, -1.10800, 24.36100, 0.02300, -12.14100, 0.79200),
                    (-0.65100, -28.28700, 0.02200, -25.23000, 0.47300, -27.3070),
                ]
            )
        else:
            self._loadcell_matrix = loadcell_matrix

        self._loadcell_data = None
        self._prev_loadcell_data = None

        self._loadcell_zero = np.zeros((1, 6), dtype=np.double)
        self._zeroed = False
        self._log = logger

    def reset(self):
        self._zeroed = False
        self._loadcell_zero = np.zeros((1, 6), dtype=np.double)

    def update(self, loadcell_zero=None):
        """
        Computes Loadcell data

        """
        if self._is_dephy:
            loadcell_signed = (self._joint.genvars - self._offset) / (
                self._adc_range * self._exc
            )
        else:
            loadcell_signed = (self._lc.update() - self._offset) / (
                self._adc_range * self._exc
            )

        loadcell_coupled = loadcell_signed * 1000 / (self._exc * self._amp_gain)

        if loadcell_zero is None:
            self._loadcell_data = (
                np.transpose(self._loadcell_matrix.dot(np.transpose(loadcell_coupled)))
                - self._loadcell_zero
            )
        else:
            self._loadcell_data = (
                np.transpose(self._loadcell_matrix.dot(np.transpose(loadcell_coupled)))
                - loadcell_zero
            )

    def initialize(self, number_of_iterations: int = 2000):
        """
        Obtains the initial loadcell reading (aka) loadcell_zero
        """
        ideal_loadcell_zero = np.zeros((1, 6), dtype=np.double)

        if not self._zeroed:

            if self._is_dephy:
                if self._joint.is_streaming:
                    self._joint.update()
                    self.update()
                else:
                    self._log.warning(
                        "[Loadcell] {self._joint.name} joint isn't streaming data. Please start streaming data before initializing loadcell."
                    )
                    return
            else:
                self.update()

            self._loadcell_zero = self._loadcell_data

            for _ in range(number_of_iterations):
                self.update(ideal_loadcell_zero)
                loadcell_offset = self._loadcell_data
                self._loadcell_zero = (loadcell_offset + self._loadcell_zero) / 2.0

            self._zeroed = True

        elif (
            input(f"[Loadcell] Would you like to re-initialize loadcell? (y/n): ")
            == "y"
        ):
            self.reset()
            self.initialize()

    @property
    def is_zeroed(self):
        return self._zeroed

    @property
    def fx(self):
        return self._loadcell_data[0][0]

    @property
    def fy(self):
        return self._loadcell_data[0][1]

    @property
    def fz(self):
        return self._loadcell_data[0][2]

    @property
    def mx(self):
        return self._loadcell_data[0][3]

    @property
    def my(self):
        return self._loadcell_data[0][4]

    @property
    def mz(self):
        return self._loadcell_data[0][5]

    @property
    def loadcell_data(self):
        return self._loadcell_data[0]


class OpenSourceLeg:
    """
    OSL class: This class is the main class for the Open Source Leg project. It
    contains all the necessary functions to control the leg.

    Returns:
        none: none
    """

    # This is a singleton class
    _instance = None

    @staticmethod
    def get_instance():
        if OpenSourceLeg._instance is None:
            OpenSourceLeg()
        return OpenSourceLeg._instance

    def __init__(self, frequency: int = 200, file_name: str = "./osl.log") -> None:

        self._frequency: int = frequency

        self._has_knee: bool = False
        self._has_ankle: bool = False
        self._has_loadcell: bool = False
        self._has_tui: bool = False

        self._knee: Joint = None
        self._ankle: Joint = None
        self._loadcell: Loadcell = None

        self.log = Logger(
            file_path=file_name, log_format="[%(asctime)s] %(levelname)s: %(message)s"
        )

        self.clock = SoftRealtimeLoop(dt=1.0 / self._frequency, report=False, fade=0.1)

        self._units: UnitsDefinition = DEFAULT_UNITS

        self.tui = None

    def __enter__(self):

        if self.has_knee:
            self._knee.start()

        if self.has_ankle:
            self._ankle.start()

        if self.has_loadcell:
            self._loadcell.initialize()

    def __exit__(self, type, value, tb):
        if self.has_knee:
            self._knee.stop()

        if self.has_ankle:
            self._ankle.stop()

        if self.has_tui:
            if self.tui.is_running:
                self.tui.quit()

    def __repr__(self) -> str:
        return f"OSL object with 0 joints"

    def get_attribute(self, object, attribute: str):
        return getattr(object, attribute)

    def add_tui(
        self,
        frequency: int = 30,
    ):
        from opensourceleg.tui import TUI

        self._has_tui = True
        self.tui = TUI(frequency=frequency)

        if self.has_loadcell:
            self.tui.set_loadcell(self._loadcell)

        self.initialize_tui()

    def add_joint(
        self,
        name: str = "knee",
        port: str = "/dev/ttyACM0",
        baud_rate: int = 230400,
        gear_ratio: float = 1.0,
        has_loadcell: bool = False,
        debug_level: int = 0,
        dephy_log: bool = False,
    ):

        if "knee" in name.lower():
            self._has_knee = True
            self._knee = Joint(
                name=name,
                port=port,
                baud_rate=baud_rate,
                frequency=self._frequency,
                gear_ratio=gear_ratio,
                has_loadcell=has_loadcell,
                logger=self.log,
                units=self.units,
                debug_level=debug_level,
                dephy_log=dephy_log,
            )

        elif "ankle" in name.lower():
            self._has_ankle = True
            self._ankle = Joint(
                name=name,
                port=port,
                baud_rate=baud_rate,
                frequency=self._frequency,
                gear_ratio=gear_ratio,
                has_loadcell=has_loadcell,
                logger=self.log,
                units=self.units,
                debug_level=debug_level,
                dephy_log=dephy_log,
            )
        else:
            self.log.warning("[OSL] Joint name is not recognized.")

    def add_loadcell(
        self,
        dephy_mode: bool = False,
        joint: Joint = None,
        amp_gain: float = 125.0,
        exc: float = 5.0,
        loadcell_matrix=None,
    ):
        self._has_loadcell = True
        self._loadcell = Loadcell(
            dephy_mode=dephy_mode,
            joint=joint,
            amp_gain=amp_gain,
            exc=exc,
            loadcell_matrix=loadcell_matrix,
            logger=self.log,
        )

    def update(
        self,
        current_limit: int = CURRENT_LIMIT,
    ):
        if self.has_knee:
            self._knee.update()

            if self.knee.motor_current > current_limit:
                self.knee.set_mode("voltage")
                self.knee.set_voltage(0, force=True)
                time.sleep(0.1)

        if self.has_ankle:
            self._ankle.update()

            if self.ankle.motor_current > current_limit:
                self.ankle.set_mode("voltage")
                self.ankle.set_voltage(0, force=True)
                time.sleep(0.1)

        if self.has_loadcell:
            self._loadcell.update()

    def run_tui(self):
        if self.has_knee:
            self.tui.add_knee(self.knee)

            if self.tui.joint is None:
                self.tui.set_active_joint(name="knee", parent="joint")

        if self.has_ankle:
            self.tui.add_ankle(self.ankle)

            if self.tui.joint is None:
                self.tui.set_active_joint(name="ankle", parent="joint")

        self.tui.add_update_callback(self.update)
        self.tui.run()

    def initialize_tui(self):

        from opensourceleg.tui import COLORS

        self.tui.add_group(
            name="left",
            parent="root",
            layout="vertical",
            border=True,
        )

        self.tui.add_group(
            name="control_panel",
            parent="root",
            layout="vertical",
            border=False,
            show_title=False,
        )

        self.tui.add_group(
            name="plot",
            parent="left",
            layout="vertical",
            border=True,
            show_title=True,
        )

        self.tui.add_group(
            name="plot_attributes",
            parent="left",
            layout="vertical",
            border=False,
            show_title=False,
        )

        self.tui.add_plot(
            parent="plot",
            color=COLORS.white,
        )

        self.tui.add_group(
            name="joint",
            parent="plot_attributes",
            layout="grid",
            border=True,
            show_title=True,
        )

        self.tui.add_group(
            name="attributes",
            parent="plot_attributes",
            layout="grid",
            border=True,
            show_title=True,
        )

        # ------------ RADIO BUTTONS ------------ #

        self.tui.add_radio_button(
            name="knee",
            category="joint",
            parent="joint",
            callback=self.tui.set_active_joint,
            color=COLORS.white,
            is_checked=True,
            row=0,
            col=0,
        )

        self.tui.add_radio_button(
            name="ankle",
            category="joint",
            parent="joint",
            callback=self.tui.set_active_joint,
            color=COLORS.white,
            is_checked=False,
            row=0,
            col=1,
        )

        self.tui.add_radio_button(
            name="motor_position",
            category="attributes",
            parent="attributes",
            callback=self.set_tui_attribute,
            color=COLORS.white,
            is_checked=True,
            row=0,
            col=0,
        )

        self.tui.add_radio_button(
            name="motor_velocity",
            category="attributes",
            parent="attributes",
            callback=self.set_tui_attribute,
            color=COLORS.white,
            is_checked=False,
            row=0,
            col=1,
        )

        self.tui.add_radio_button(
            name="motor_current",
            category="attributes",
            parent="attributes",
            callback=self.set_tui_attribute,
            color=COLORS.white,
            is_checked=False,
            row=0,
            col=2,
        )

        self.tui.add_radio_button(
            name="joint_position",
            category="attributes",
            parent="attributes",
            callback=self.set_tui_attribute,
            color=COLORS.white,
            is_checked=False,
            row=1,
            col=0,
        )

        self.tui.add_radio_button(
            name="joint_velocity",
            category="attributes",
            parent="attributes",
            callback=self.set_tui_attribute,
            color=COLORS.white,
            is_checked=False,
            row=1,
            col=1,
        )

        self.tui.add_radio_button(
            name="joint_torque",
            category="attributes",
            parent="attributes",
            callback=self.set_tui_attribute,
            color=COLORS.white,
            is_checked=False,
            row=1,
            col=2,
        )

        self.tui.add_radio_button(
            name="loadcell_Fx",
            category="attributes",
            parent="attributes",
            callback=self.set_tui_attribute,
            color=COLORS.white,
            is_checked=False,
            row=2,
            col=0,
        )

        self.tui.add_radio_button(
            name="loadcell_fy",
            category="attributes",
            parent="attributes",
            callback=self.set_tui_attribute,
            color=COLORS.white,
            is_checked=False,
            row=2,
            col=1,
        )

        self.tui.add_radio_button(
            name="loadcell_fz",
            category="attributes",
            parent="attributes",
            callback=self.set_tui_attribute,
            color=COLORS.white,
            is_checked=False,
            row=2,
            col=2,
        )

        # ------------ CONTROL PANEL ------------ #

        self.tui.add_group(
            name="joint_control",
            parent="control_panel",
            layout="horizontal",
            border=False,
            show_title=False,
        )

        self.tui.add_group(
            name="knee",
            parent="joint_control",
            layout="grid",
            border=True,
            show_title=True,
        )

        self.tui.add_group(
            name="ankle",
            parent="joint_control",
            layout="grid",
            border=True,
            show_title=True,
        )

        self.tui.add_group(
            name="utility",
            parent="control_panel",
            layout="grid",
            border=False,
            show_title=False,
        )

        self.tui.add_group(
            name="estop",
            parent="control_panel",
            layout="grid",
            border=False,
            show_title=False,
        )

        # ------------ UTILITY BUTTONS ------------ #

        self.tui.add_button(
            name="emergency_stop",
            parent="estop",
            callback=self.estop,
            color=COLORS.maize,
            border=True,
        )

        self.tui.add_button(
            name="safety_reset",
            parent="utility",
            callback=self.reset,
            color=COLORS.white,
            row=0,
            col=0,
        )

        self.tui.add_group(
            name="calibrate",
            parent="utility",
            layout="grid",
            border=True,
            show_title=True,
            row=0,
            col=1,
        )

        self.tui.add_button(
            name="encoders",
            parent="calibrate",
            # callback=self.calibrate_encoders,
            color=COLORS.white,
            row=0,
            col=0,
        )

        self.tui.add_button(
            name="loadcell",
            parent="calibrate",
            # callback=self.calibrate_loadcell,
            color=COLORS.white,
            row=1,
            col=0,
        )

        # ------------ KNEE BUTTONS ------------ #

        self.tui.add_button(
            name="home",
            parent="knee",
            callback=self.home,
            color=COLORS.white,
            row=0,
            col=0,
        )

        self.tui.add_group(
            name="knee_data",
            parent="knee",
            layout="grid",
            border=True,
            show_title=False,
            row=1,
            col=0,
        )

        # ------------ KNEE VALUES ------------ #

        self.tui.add_text(
            name="Voltage (mV): ",
            parent="knee_data",
            row=0,
            col=0,
        )

        self.tui.add_value(
            name="voltage",
            parent="knee_data",
            default=0,
            callback=self.set_motor_voltage_sp,
            row=0,
            col=1,
        )

        self.tui.add_text(
            name="Position (deg): ",
            parent="knee_data",
            row=1,
            col=0,
        )

        self.tui.add_value(
            name="position",
            parent="knee_data",
            default=0,
            callback=self.set_motor_position_sp,
            row=1,
            col=1,
        )

        self.tui.add_text(
            name="Current (mA): ",
            parent="knee_data",
            row=2,
            col=0,
        )

        self.tui.add_value(
            name="current",
            parent="knee_data",
            default=0,
            callback=self.set_motor_current_sp,
            row=2,
            col=1,
        )

        self.tui.add_text(
            name=" ",
            parent="knee_data",
            row=3,
            col=0,
        )

        self.tui.add_text(
            name="Stiffness: ",
            parent="knee_data",
            row=4,
            col=0,
        )

        self.tui.add_value(
            name="stiffness",
            parent="knee_data",
            default=200,
            callback=self.set_stiffness_sp,
            row=4,
            col=1,
        )

        self.tui.add_text(
            name="Damping: ",
            parent="knee_data",
            row=5,
            col=0,
        )

        self.tui.add_value(
            name="damping",
            parent="knee_data",
            default=400,
            callback=self.set_damping_sp,
            row=5,
            col=1,
        )

        self.tui.add_text(
            name="Equilibrium Position (deg): ",
            parent="knee_data",
            row=6,
            col=0,
        )

        self.tui.add_value(
            name="equilibrium_position",
            parent="knee_data",
            default=0,
            callback=self.set_equilibrium_position_sp,
            row=6,
            col=1,
        )

        self.tui.add_group(
            name="mode",
            parent="knee",
            layout="grid",
            border=True,
            show_title=True,
            row=2,
            col=0,
        )

        self.tui.add_text(
            name=" ",
            parent="mode",
            row=2,
            col=0,
        )

        self.tui.add_text(
            name=" Updates only the setpoints that are ",
            parent="mode",
            color=COLORS.turquoise,
            row=3,
            col=0,
        )

        self.tui.add_text(
            name=" relevant to the selected control mode.",
            parent="mode",
            color=COLORS.turquoise,
            row=4,
            col=0,
        )

        self.tui.add_group(
            name="kupdater",
            parent="knee",
            layout="grid",
            border=False,
            show_title=False,
            row=3,
            col=0,
        )

        self.tui.add_button(
            name="stop",
            parent="kupdater",
            callback=self.stop_joint,
            color=COLORS.white,
            row=0,
            col=0,
        )

        self.tui.add_button(
            name="update",
            parent="kupdater",
            callback=self.update_joint,
            color=COLORS.white,
            row=0,
            col=1,
        )

        # ------------ KNEE DROPDOWN ------------ #

        self.tui.add_dropdown(
            name="knee",
            parent="mode",
            options=["Voltage (V)", "Position (P)", "Current (I)", "Impedance (Z)"],
            callback=self.set_control_mode_sp,
            row=1,
            col=0,
        )

        # ------------ ANKLE BUTTONS ------------ #

        self.tui.add_button(
            name="home",
            parent="ankle",
            callback=self.home,
            color=COLORS.white,
            row=0,
            col=0,
        )

        self.tui.add_group(
            name="ankle_data",
            parent="ankle",
            layout="grid",
            border=True,
            show_title=False,
            row=1,
            col=0,
        )

        # ------------ ANKLE VALUES ------------ #

        self.tui.add_text(
            name="Voltage (mV): ",
            parent="ankle_data",
            row=0,
            col=0,
        )

        self.tui.add_value(
            name="voltage",
            parent="ankle_data",
            default=0,
            callback=self.set_motor_voltage_sp,
            row=0,
            col=1,
        )

        self.tui.add_text(
            name="Position (deg): ",
            parent="ankle_data",
            row=1,
            col=0,
        )

        self.tui.add_value(
            name="position",
            parent="ankle_data",
            default=0,
            callback=self.set_motor_position_sp,
            row=1,
            col=1,
        )

        self.tui.add_text(
            name="Current (mA): ",
            parent="ankle_data",
            row=2,
            col=0,
        )

        self.tui.add_value(
            name="current",
            parent="ankle_data",
            default=0,
            callback=self.set_motor_current_sp,
            row=2,
            col=1,
        )

        self.tui.add_text(
            name=" ",
            parent="ankle_data",
            row=3,
            col=0,
        )

        self.tui.add_text(
            name="Stiffness: ",
            parent="ankle_data",
            row=4,
            col=0,
        )

        self.tui.add_value(
            name="stiffness",
            parent="ankle_data",
            default=200,
            callback=self.set_stiffness_sp,
            row=4,
            col=1,
        )

        self.tui.add_text(
            name="Damping: ",
            parent="ankle_data",
            row=5,
            col=0,
        )

        self.tui.add_value(
            name="damping",
            parent="ankle_data",
            default=400,
            callback=self.set_damping_sp,
            row=5,
            col=1,
        )

        self.tui.add_text(
            name="Equilibrium Position (deg): ",
            parent="ankle_data",
            row=6,
            col=0,
        )

        self.tui.add_value(
            name="equilibrium_position",
            parent="ankle_data",
            default=0,
            callback=self.set_equilibrium_position_sp,
            row=6,
            col=1,
        )

        self.tui.add_group(
            name="mode",
            parent="ankle",
            layout="grid",
            border=True,
            show_title=True,
            row=2,
            col=0,
        )

        self.tui.add_text(
            name=" ",
            parent="mode",
            row=2,
            col=0,
        )

        self.tui.add_text(
            name=" Updates only the setpoints that are ",
            parent="mode",
            color=COLORS.turquoise,
            row=3,
            col=0,
        )

        self.tui.add_text(
            name=" relevant to the selected control mode.",
            parent="mode",
            color=COLORS.turquoise,
            row=4,
            col=0,
        )

        self.tui.add_group(
            name="aupdater",
            parent="ankle",
            layout="grid",
            border=False,
            show_title=False,
            row=3,
            col=0,
        )

        self.tui.add_button(
            name="stop",
            parent="aupdater",
            callback=self.stop_joint,
            color=COLORS.white,
            row=0,
            col=0,
        )

        self.tui.add_button(
            name="update",
            parent="aupdater",
            callback=self.update_joint,
            color=COLORS.white,
            row=0,
            col=1,
        )

        # ------------ ANKLE DROPDOWN ------------ #

        self.tui.add_dropdown(
            name="ankle",
            parent="mode",
            options=["Voltage (V)", "Position (P)", "Current (I)", "Impedance (Z)"],
            callback=self.set_control_mode_sp,
        )

    def set_motor_current_sp(self, **kwargs):
        self.log.debug("[OSL] Setting motor current setpoint.")

        name = kwargs["name"]
        parent = kwargs["parent"]
        value = int(self.tui.values[parent + "_" + name].text())

        _name = parent + name

        if "knee" in _name and self.has_knee:
            self.knee._motor_current_sp = value
            self.log.debug("[OSL] Setting knee motor current setpoint to %s." % value)

        elif "ankle" in name and self.has_ankle:
            self.ankle._motor_current_sp = value

    def set_motor_position_sp(self, **kwargs):
        self.log.debug("[OSL] Setting motor position setpoint.")

        name = kwargs["name"]
        parent = kwargs["parent"]
        _value = int(self.tui.values[parent + "_" + name].text())

        value = np.deg2rad(_value)
        _name = parent + name

        if "knee" in _name and self.has_knee:
            self.knee._motor_position_sp = value
            self.log.debug("[OSL] Setting knee motor position setpoint to %s." % value)

        elif "ankle" in name and self.has_ankle:
            self.ankle._motor_position_sp = value

    def set_motor_voltage_sp(self, **kwargs):
        self.log.debug("[OSL] Setting motor voltage setpoint.")

        name = kwargs["name"]
        parent = kwargs["parent"]
        value = int(self.tui.values[parent + "_" + name].text())

        _name = parent + name

        if "knee" in _name and self.has_knee:
            self.knee._motor_voltage_sp = value
            self.log.debug("[OSL] Setting knee motor voltage setpoint to %s." % value)

        elif "ankle" in name and self.has_ankle:
            self.ankle._motor_voltage_sp = value

    def set_stiffness_sp(self, **kwargs):
        self.log.debug("[OSL] Setting stiffness setpoint.")

        name = kwargs["name"]
        parent = kwargs["parent"]
        value = int(self.tui.values[parent + "_" + name].text())

        _name = parent + name

        if "knee" in _name and self.has_knee:
            self.knee._stiffness_sp = value
            self.log.debug("[OSL] Setting knee stiffness setpoint to %s." % value)

        elif "ankle" in name and self.has_ankle:
            self.ankle._stiffness_sp = value

    def set_damping_sp(self, **kwargs):
        self.log.debug("[OSL] Setting damping setpoint.")

        name = kwargs["name"]
        parent = kwargs["parent"]
        value = int(self.tui.values[parent + "_" + name].text())

        _name = parent + name

        if "knee" in _name and self.has_knee:
            self.knee._damping_sp = value
            self.log.debug("[OSL] Setting knee damping setpoint to %s." % value)

        elif "ankle" in name and self.has_ankle:
            self.ankle._damping_sp = value

    def set_equilibrium_position_sp(self, **kwargs):
        self.log.debug("[OSL] Setting equilibrium position setpoint.")

        name = kwargs["name"]
        parent = kwargs["parent"]
        _value = int(self.tui.values[parent + "_" + name].text())

        value = np.deg2rad(_value)

        _name = parent + name

        if "knee" in _name and self.has_knee:
            self.knee._equilibrium_position_sp = value
            self.log.debug(
                "[OSL] Setting knee equilibrium position setpoint to %s." % value
            )

        elif "ankle" in name and self.has_ankle:
            self.ankle._equilibrium_position_sp = value

    def set_control_mode_sp(self, **kwargs):
        self.log.debug("[OSL] Setting control mode setpoint.")

        name = kwargs["name"]
        parent = kwargs["parent"]
        _mode_id = self.tui.dropdowns[parent + "_" + name].currentIndex()
        _name = (parent + name).lower()

        if "knee" in _name and self.has_knee:
            self.knee._control_mode_sp = self.tui.control_modes[_mode_id]
            self.log.debug(
                "[OSL] Setting knee control mode setpoint to %s."
                % self.tui.control_modes[_mode_id]
            )

        elif "ankle" in _name and self.has_ankle:
            self.ankle._control_mode_sp = self.tui.control_modes[_mode_id]
            self.log.debug(
                "[OSL] Setting ankle control mode setpoint to %s."
                % self.tui.control_modes[_mode_id]
            )

    def estop(self, **kwargs):
        self.log.debug("[OSL] Emergency stop activated.")
        self.tui.quit()

    def set_tui_attribute(self, **kwargs):
        name = kwargs["name"]
        self.tui.set_active_attribute(name)

    def home(self, **kwargs):
        name = kwargs["name"]
        parent = kwargs["parent"]

        _name = (parent + name).lower()

        if "knee" in _name:
            self.log.debug("[OSL] Homing knee joint.")
            if self.has_knee:
                self.knee.home()

        elif "ankle" in _name:
            self.log.debug("[OSL] Homing ankle joint.")
            if self.has_ankle:
                self.ankle.home()

    def calibrate_loadcell(self, **kwargs):
        self.log.debug("[OSL] Calibrating loadcell.")
        if self.has_loadcell:
            self.loadcell.reset()

    def calibrate_encoders(self, **kwargs):
        self.log.debug("[OSL] Calibrating encoders.")

        if self.has_knee:
            self.knee.make_encoder_map()

        if self.has_ankle:
            self.ankle.make_encoder_map()

    def reset(self, **kwargs):
        self.log.debug("[OSL] Resetting OSL.")

        if self.has_knee:
            self.knee.set_mode("voltage")
            self.knee.set_voltage(0, force=True)

            time.sleep(0.1)

        if self.has_ankle:
            self.ankle.set_mode("voltage")
            self.ankle.set_voltage(0, force=True)

            time.sleep(0.1)

    def update_joint(self, **kwargs):
        name = kwargs["name"]
        parent = kwargs["parent"]

        _name = (parent + name).lower()

        if "kupdater" in _name:
            self.log.debug("[OSL] Updating knee joint.")

            if self.has_knee:

                if self.knee.control_mode_sp == "voltage":
                    self.knee.set_mode(self.knee.control_mode_sp)
                    self.knee.set_voltage(self.knee.motor_voltage_sp, force=True)

                elif self.knee.control_mode_sp == "current":
                    self.knee.set_mode(self.knee.control_mode_sp)
                    self.knee.set_current_gains()
                    self.knee.set_current(self.knee.motor_current_sp, force=True)

                elif self.knee.control_mode_sp == "position":
                    self.knee.set_mode(self.knee.control_mode_sp)
                    self.knee.set_position_gains()
                    self.knee.set_output_position(self.knee.motor_position_sp)

                else:
                    self.knee.set_mode(self.knee.control_mode_sp)

                    self.knee.set_impedance_gains(
                        K=self.knee.stiffness_sp,
                        B=self.knee.damping_sp,
                    )

                    self.knee.set_output_position(self.knee.equilibirum_position_sp)

        elif "aupdater" in _name:
            self.log.debug("[OSL] Updating ankle joint.")

            if self.has_ankle:

                if self.ankle.control_mode_sp == "voltage":
                    self.ankle.set_mode(self.ankle.control_mode_sp)
                    self.ankle.set_voltage(self.ankle.motor_voltage_sp, force=True)

                elif self.ankle.control_mode_sp == "current":
                    self.ankle.set_mode(self.ankle.control_mode_sp)
                    self.ankle.set_current_gains()
                    self.ankle.set_current(self.ankle.motor_current_sp, force=True)

                elif self.ankle.control_mode_sp == "position":
                    self.ankle.set_mode(self.ankle.control_mode_sp)
                    self.ankle.set_position_gains()
                    self.ankle.set_output_position(self.ankle.motor_position_sp)

                else:
                    self.ankle.set_mode(self.ankle.control_mode_sp)

                    self.ankle.set_impedance_gains(
                        K=self.ankle.stiffness_sp,
                        B=self.ankle.damping_sp,
                    )

                    self.ankle.set_output_position(self.ankle.equilibirum_position_sp)

    def stop_joint(self, **kwargs):
        name = kwargs["name"]
        parent = kwargs["parent"]

        _name = (parent + name).lower()

        if "kupdater" in _name:
            self.log.debug("[OSL] Stopping knee joint.")
            if self.has_knee:
                self.knee.set_mode("voltage")
                self.knee.set_voltage(0, force=True)

                time.sleep(0.1)

        elif "aupdater" in _name:
            self.log.debug("[OSL] Stopping ankle joint.")
            if self.has_ankle:
                self.ankle.set_mode("voltage")
                self.ankle.set_voltage(0, force=True)

                time.sleep(0.1)

    @property
    def knee(self):
        if self.has_knee:
            return self._knee
        else:
            self.log.warning("[OSL] Knee is not connected.")

    @property
    def ankle(self):
        if self.has_ankle:
            return self._ankle
        else:
            self.log.warning("[OSL] Ankle is not connected.")

    @property
    def loadcell(self):
        if self.has_loadcell:
            return self._loadcell
        else:
            self.log.warning("[OSL] Loadcell is not connected.")

    @property
    def units(self):
        return self._units

    @property
    def has_knee(self):
        return self._has_knee

    @property
    def has_ankle(self):
        return self._has_ankle

    @property
    def has_loadcell(self):
        return self._has_loadcell

    @property
    def loadcell(self):
        return self._loadcell

    @property
    def has_tui(self):
        return self._has_tui


if __name__ == "__main__":
    osl = OpenSourceLeg(frequency=100)

    osl.add_joint(
        name="knee",
        port="/dev/ttyACM6",
        baud_rate=230400,
        gear_ratio=41.4999,
    )

    osl.add_loadcell(
        dephy_mode=False,
    )

    osl.add_tui()

    with osl:
        # osl.loadcell.update()
        # osl.log.info("[OSL] Loadcell: {}".format(osl.loadcell.fz))

        osl.run_tui()

        # osl.knee.home(
        #     homing_voltage=2000,
        # )

        # osl.knee.set_mode("impedance")
        # osl.knee.set_impedance_gains()

        # osl.log.info("[OSL] Setting knee to pi rad.")
        # osl.knee.set_output_position(np.deg2rad(30))

        # time.sleep(3)
