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

import flexsea.enums as fxe
import numpy as np
from flexsea.device import Device

from opensourceleg.tui import COLORS, TUI
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


DEFAULT_POSITION_GAINS = Gains(kp=200, ki=50, kd=0, K=0, B=0, ff=0)

DEFAULT_CURRENT_GAINS = Gains(kp=40, ki=400, kd=0, K=0, B=0, ff=128)

DEFAULT_IMPEDANCE_GAINS = Gains(kp=40, ki=400, kd=0, K=300, B=1600, ff=128)


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
    def __init__(self, control_mode: str, device: "DephyActpack"):
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
    def mode(self) -> str:
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
        super().__init__("voltage", device)
        self._entry_callback = self._entry
        self._exit_callback = self._exit

    def _entry(self):
        print("Entering voltage mode")

    def _exit(self):
        self._set_voltage(0)
        print("Exiting voltage mode")

    def _set_voltage(self, voltage: int):
        self._device.command_motor_voltage(
            voltage,
        )


class CurrentMode(ActpackMode):
    def __init__(self, device: "DephyActpack"):
        super().__init__("current", device)
        self._entry_callback = self._entry
        self._exit_callback = self._exit

    def _entry(self):
        if not self.has_gains:
            self._set_gains()

        self._set_current(0)

    def _exit(self):
        self._device.command_motor_voltage(0)

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

    def _set_current(self, current: int):
        """Sets the Q-axis current of the motor

        Args:
            current (int): _description_
        """
        self._device.command_motor_current(
            current,
        )


class PositionMode(ActpackMode):
    def __init__(self, device: "DephyActpack"):
        super().__init__("position", device)
        self._entry_callback = self._entry
        self._exit_callback = self._exit

    def _entry(self):
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
        self._device.command_motor_voltage(0)
        print("Exiting POSITION mode")

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

    def _set_motor_position(self, counts: int):
        """Sets the motor position

        Args:
            counts (int): position in counts
        """
        self._device.command_motor_position(
            counts,
        )


class ImpedanceMode(ActpackMode):
    def __init__(self, device: "DephyActpack"):
        super().__init__("impedance", device)
        self._entry_callback = self._entry
        self._exit_callback = self._exit

    def _entry(self):
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
        self._device.command_motor_voltage(0)
        print("Exiting IMPEDANCE mode")

    def _set_motor_position(self, counts: int):
        """Sets the motor position

        Args:
            counts (int): position in counts
        """
        self._device.command_motor_position(
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
        super().__init__(os.path.realpath(port), baud_rate, debug_level)
        self._debug_level = debug_level
        self._dephy_log = dephy_log
        self._frequency = frequency
        self._data: dict = None
        self._log = logger
        self._state = None
        self._units = units if units else DEFAULT_UNITS

        self._modes: dict[str, ActpackMode] = {
            "voltage": VoltageMode(self),
            "position": PositionMode(self),
            "current": CurrentMode(self),
            "impedance": ImpedanceMode(self),
        }

        self._mode: ActpackMode = self._modes["voltage"]

    def start(self):
        self.open(self._debug_level, log_enabled=self._dephy_log)
        self.start_streaming(self._frequency)
        time.sleep(0.1)
        self._mode.enter()

    def stop(self):
        self.close()

    def update(self):
        self._data = self.read()

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

        Raises:
            ValueError: If the mode is not POSITION and force is False
        """
        if self._mode != self._modes["position"]:
            if force:
                self._mode.transition(self._modes["position"])
            else:
                raise ValueError(f"Cannot set position gains in mode {self._mode}")

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

        Raises:
            ValueError: If the mode is not CURRENT and force is False
        """
        if self._mode != self._modes["current"]:
            if force:
                self._mode.transition(self._modes["current"])
            else:
                raise ValueError(f"Cannot set current gains in mode {self._mode}")

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

        Raises:
            ValueError: If the mode is not IMPEDANCE and force is False
        """
        if self._mode != self._modes["impedance"]:
            if force:
                self._mode.transition(self._modes["impedance"])
            else:
                raise ValueError(f"Cannot set impedance gains in mode {self._mode}")

        self._mode._set_gains(kp=kp, ki=ki, K=K, B=B, ff=ff)

    def set_voltage(self, value: float, force: bool = False):
        """
        Sets the q axis voltage

        Args:
            value (float): The voltage to set
            force (bool): Force the mode transition. Defaults to False.

        Raises:
            ValueError: If the mode is not VOLTAGE and force is False

        """
        if self._mode != self._modes["voltage"]:
            if force:
                self._mode.transition(self._modes["voltage"])
            else:
                raise ValueError(f"Cannot set voltage in mode {self._mode}")

        self._mode._set_voltage(
            int(self._units.convert_to_default_units(value, "voltage")),
        )

    def set_current(self, value: float, force: bool = False):
        """
        Sets the q axis current

        Args:
            value (float): The current to set
            force (bool): Force the mode transition. Defaults to False.

        Raises:
            ValueError: If the mode is not CURRENT and force is False
        """
        if self._mode != self._modes["current"]:
            if force:
                self._mode.transition(self._modes["current"])
            else:
                raise ValueError(f"Cannot set current in mode {self._mode}")

        self._mode._set_current(
            int(self._units.convert_to_default_units(value, "current")),
        )

    def set_motor_torque(self, torque: float, force: bool = False):
        """
        Sets the motor torque

        Args:
            torque (float): The torque to set
            force (bool): Force the mode transition. Defaults to False.

        Raises:
            ValueError: If the mode is not CURRENT and force is False
        """
        if self._mode != self._modes["current"]:
            if force:
                self._mode.transition(self._modes["current"])
            else:
                raise ValueError(f"Cannot set motor_torque in mode {self._mode}")

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

        Raises:
            ValueError: If the mode is not POSITION or IMPEDANCE
        """
        if self._mode not in [self._modes["position"], self._modes["impedance"]]:
            raise ValueError(f"Cannot set motor position in mode {self._mode}")

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
        return self._mode()

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
    def voltage(self):
        return self._units.convert_from_default_units(
            self._data.mot_volt,
            "voltage",
        )

    @property
    def current(self):
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
            self.current * NM_PER_AMP,
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
        baud_rate: int = 115200,
        frequency: int = 1000,
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

        if "knee" in name.lower() or "ankle" in name.lower():
            self._name = name
        else:
            raise ValueError(f"Invalid joint name: {name}")

        if os.path.isfile(f"./{self._name}_encoder_map.npy"):
            coefficients = np.load(f"./{self._name}_encoder_map.npy")
            self._encoder_map = np.polynomial.polynomial.Polynomial(coefficients)
        else:
            self._log.info(
                f"[{self._name}] No encoder map found. Please run the calibration routine."
            )

    def home(
        self,
        homing_voltage: int = 2000,
        homing_frequency: int = 100,
    ):

        is_homing = True

        CURRENT_THRESHOLD = 6000
        VELOCITY_THRESHOLD = 0.01

        self.set_voltage(-1 * homing_voltage)  # mV, negative for counterclockwise

        _motor_encoder_array = []
        _joint_encoder_array = []

        time.sleep(0.1)

        while is_homing:
            time.sleep(1 / homing_frequency)

            _motor_encoder_array.append(self.motor_position)
            _joint_encoder_array.append(self.joint_position)

            if (
                abs(self.output_velocity) <= VELOCITY_THRESHOLD
                or abs(self.current) >= CURRENT_THRESHOLD
            ):
                self.set_voltage(0)
                is_homing = False

        time.sleep(0.1)

        if np.std(_motor_encoder_array) < 1e-6:
            self._log.warn(
                f"[{self._name}] Motor encoder not working. Please check the wiring."
            )
            return
        elif np.std(_joint_encoder_array) < 1e-6:
            self._log.warn(
                f"[{self._name}] Joint encoder not working. Please check the wiring."
            )
            return

        if "ankle" in self._name.lower():
            self._zero_pos = np.deg2rad(30)

        self._is_homed = True

        if self.encoder_map is None:
            if (
                input(f"[{self._name}] Would you like to make an encoder map? (y/n): ")
                == "y"
            ):
                self.make_encoder_map()

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
            self._log.warn(
                f"[{self._name}] Please home the joint before making the encoder map."
            )
            return

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

        input()
        _start_time = time.time()

        while time.time() - _start_time < 10:
            self.update()
            _joint_position_array.append(self.joint_position)
            _output_position_array.append(self.output_position)

            time.sleep(1 / self.frequency)

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


class Loadcell:
    def __init__(
        self,
        joint: Joint,
        amp_gain: float = 125.0,
        exc: float = 5.0,
        loadcell_matrix: np.array = None,
        logger: "Logger" = None,
    ) -> None:
        self._joint = joint
        self._amp_gain = amp_gain
        self._exc = exc

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

        loadcell_signed = (self._joint.genvars - 2048) / 4095 * self._exc
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
            if self._joint.isStreaming:
                self._joint.update()
                self.update()
                self._loadcell_zero = self._loadcell_data

                for _ in range(number_of_iterations):
                    self.update(ideal_loadcell_zero)
                    loadcell_offset = self._loadcell_data
                    self._loadcell_zero = (loadcell_offset + self._loadcell_zero) / 2.0

            else:
                self._log.warn(
                    "[Loadcell] {self._joint.name} joint isn't streaming data. Please start streaming data before initializing loadcell."
                )
                return

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
        return self._loadcell_data[3]

    @property
    def my(self):
        return self._loadcell_data[4]

    @property
    def mz(self):
        return self._loadcell_data[5]


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

        self.tui = TUI()
        self.initialize_tui()

        self._knee: Joint = None
        self._ankle: Joint = None
        self._loadcell: Loadcell = None

        self.log = Logger(
            file_path=file_name, log_format="[%(asctime)s] %(levelname)s: %(message)s"
        )

        self.loop = SoftRealtimeLoop(dt=1.0 / self._frequency, report=False, fade=0.1)

        self._units: UnitsDefinition = DEFAULT_UNITS

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

    def __repr__(self) -> str:
        return f"OSL object with 0 joints"

    def add_joint(
        self,
        name: str = "knee",
        port: str = "/dev/ttyACM0",
        baud_rate: int = 115200,
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
            self.log.warn("[OSL] Joint name is not recognized.")

    def add_loadcell(
        self,
        joint: Joint,
        amp_gain: float = 125.0,
        exc: float = 5.0,
        loadcell_matrix=None,
    ):
        self._has_loadcell = True
        self._loadcell = Loadcell(
            joint=joint,
            amp_gain=amp_gain,
            exc=exc,
            loadcell_matrix=loadcell_matrix,
            logger=self.log,
        )

    def update(self):
        if self.has_knee:
            self._knee.update()

        if self.has_ankle:
            self._ankle.update()

        if self.has_loadcell:
            self._loadcell.update()

    def initialize_tui(self):

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
            callback=self.tui.set_category,
            color=COLORS.white,
            is_checked=True,
            row=0,
            col=0,
        )

        self.tui.add_radio_button(
            name="ankle",
            category="joint",
            parent="joint",
            callback=self.tui.set_category,
            color=COLORS.white,
            is_checked=False,
            row=0,
            col=1,
        )

        self.tui.add_radio_button(
            name="motor_position",
            category="attributes",
            parent="attributes",
            callback=self.tui.set_category,
            color=COLORS.white,
            is_checked=True,
            row=0,
            col=0,
        )

        self.tui.add_radio_button(
            name="motor_velocity",
            category="attributes",
            parent="attributes",
            callback=self.tui.set_category,
            color=COLORS.white,
            is_checked=False,
            row=0,
            col=1,
        )

        self.tui.add_radio_button(
            name="motor_current",
            category="attributes",
            parent="attributes",
            callback=self.tui.set_category,
            color=COLORS.white,
            is_checked=False,
            row=0,
            col=2,
        )

        self.tui.add_radio_button(
            name="joint_position",
            category="attributes",
            parent="attributes",
            callback=self.tui.set_category,
            color=COLORS.white,
            is_checked=False,
            row=1,
            col=0,
        )

        self.tui.add_radio_button(
            name="joint_velocity",
            category="attributes",
            parent="attributes",
            callback=self.tui.set_category,
            color=COLORS.white,
            is_checked=False,
            row=1,
            col=1,
        )

        self.tui.add_radio_button(
            name="joint_torque",
            category="attributes",
            parent="attributes",
            callback=self.tui.set_category,
            color=COLORS.white,
            is_checked=False,
            row=1,
            col=2,
        )

        self.tui.add_radio_button(
            name="loadcell_Fx",
            category="attributes",
            parent="attributes",
            callback=self.tui.set_category,
            color=COLORS.white,
            is_checked=False,
            row=2,
            col=0,
        )

        self.tui.add_radio_button(
            name="loadcell_fy",
            category="attributes",
            parent="attributes",
            callback=self.tui.set_category,
            color=COLORS.white,
            is_checked=False,
            row=2,
            col=1,
        )

        self.tui.add_radio_button(
            name="loadcell_fz",
            category="attributes",
            parent="attributes",
            callback=self.tui.set_category,
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
            callback=self.tui.test_button,
            color=COLORS.white,
            row=0,
            col=0,
        )

        self.tui.add_button(
            name="loadcell",
            parent="calibrate",
            callback=self.tui.test_button,
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
            name="kdata",
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
            parent="kdata",
            row=0,
            col=0,
        )

        self.tui.add_value(
            name="voltage",
            parent="kdata",
            default=0,
            callback=self.tui.set_value,
            row=0,
            col=1,
        )

        self.tui.add_text(
            name="Position (deg): ",
            parent="kdata",
            row=1,
            col=0,
        )

        self.tui.add_value(
            name="position",
            parent="kdata",
            default=0,
            callback=self.tui.set_value,
            row=1,
            col=1,
        )

        self.tui.add_text(
            name="Current (mA): ",
            parent="kdata",
            row=2,
            col=0,
        )

        self.tui.add_value(
            name="current",
            parent="kdata",
            default=0,
            callback=self.tui.set_value,
            row=2,
            col=1,
        )

        self.tui.add_text(
            name=" ",
            parent="kdata",
            row=3,
            col=0,
        )

        self.tui.add_text(
            name="Stiffness (N/rad): ",
            parent="kdata",
            row=4,
            col=0,
        )

        self.tui.add_value(
            name="stiffness",
            parent="kdata",
            default=0,
            callback=self.tui.set_value,
            row=4,
            col=1,
        )

        self.tui.add_text(
            name="Damping (N/(rad/s)): ",
            parent="kdata",
            row=5,
            col=0,
        )

        self.tui.add_value(
            name="damping",
            parent="kdata",
            default=0,
            callback=self.tui.set_value,
            row=5,
            col=1,
        )

        self.tui.add_text(
            name="Equilibrium Position (deg): ",
            parent="kdata",
            row=6,
            col=0,
        )

        self.tui.add_value(
            name="equilibrium_position",
            parent="kdata",
            default=0,
            callback=self.tui.set_value,
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
            callback=self.tui.test_button,
            color=COLORS.white,
            row=0,
            col=1,
        )

        # ------------ KNEE DROPDOWN ------------ #

        self.tui.add_dropdown(
            name="knee",
            parent="mode",
            options=["Voltage (V)", "Position (P)", "Current (I)", "Impedance (Z)"],
            callback=self.tui.set_joint_mode,
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
            name="adata",
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
            parent="adata",
            row=0,
            col=0,
        )

        self.tui.add_value(
            name="voltage",
            parent="adata",
            default=0,
            callback=self.tui.set_value,
            row=0,
            col=1,
        )

        self.tui.add_text(
            name="Position (deg): ",
            parent="adata",
            row=1,
            col=0,
        )

        self.tui.add_value(
            name="position",
            parent="adata",
            default=0,
            callback=self.tui.set_value,
            row=1,
            col=1,
        )

        self.tui.add_text(
            name="Current (mA): ",
            parent="adata",
            row=2,
            col=0,
        )

        self.tui.add_value(
            name="current",
            parent="adata",
            default=0,
            callback=self.tui.set_value,
            row=2,
            col=1,
        )

        self.tui.add_text(
            name=" ",
            parent="adata",
            row=3,
            col=0,
        )

        self.tui.add_text(
            name="Stiffness (N/rad): ",
            parent="adata",
            row=4,
            col=0,
        )

        self.tui.add_value(
            name="stiffness",
            parent="adata",
            default=0,
            callback=self.tui.set_value,
            row=4,
            col=1,
        )

        self.tui.add_text(
            name="Damping (N/(rad/s)): ",
            parent="adata",
            row=5,
            col=0,
        )

        self.tui.add_value(
            name="damping",
            parent="adata",
            default=0,
            callback=self.tui.set_value,
            row=5,
            col=1,
        )

        self.tui.add_text(
            name="Equilibrium Position (deg): ",
            parent="adata",
            row=6,
            col=0,
        )

        self.tui.add_value(
            name="equilibrium_position",
            parent="adata",
            default=0,
            callback=self.tui.set_value,
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
            callback=self.tui.test_button,
            color=COLORS.white,
            row=0,
            col=1,
        )

        # ------------ ANKLE DROPDOWN ------------ #

        self.tui.add_dropdown(
            name="ankle",
            parent="mode",
            options=["Voltage (V)", "Position (P)", "Current (I)", "Impedance (Z)"],
            callback=self.tui.set_joint_mode,
        )

    def estop(self, **kwargs):
        self.log.info("[OSL] Emergency stop activated.")

        if self.has_knee:
            self.knee.stop()

        if self.has_ankle:
            self.ankle.stop()

    def home(self, **kwargs):
        name = kwargs["name"]
        parent = kwargs["parent"]

        _name = (parent + name).lower()

        if "knee" in _name:
            self.log.info("[OSL] Homing knee joint.")
            if self.has_knee:
                self.knee.home()

        elif "ankle" in _name:
            self.log.info("[OSL] Homing ankle joint.")
            if self.has_ankle:
                self.ankle.home()

    def reset(self, **kwargs):
        self.log.info("[OSL] Resetting OSL.")

        self.__exit__(None, None, None)

        time.sleep(1)

        self.__enter__()

    def stop_joint(self, **kwargs):
        name = kwargs["name"]
        parent = kwargs["parent"]

        _name = (parent + name).lower()

        if "kupdater" in _name:
            self.log.info("[OSL] Stopping knee joint.")
            if self.has_knee:
                self.knee.stop()

        elif "aupdater" in _name:
            self.log.info("[OSL] Stopping ankle joint.")
            if self.has_ankle:
                self.ankle.stop()

    @property
    def knee(self):
        if self.has_knee:
            return self._knee
        else:
            self.log.warn("[OSL] Knee is not connected.")

    @property
    def ankle(self):
        if self.has_ankle:
            return self._ankle
        else:
            self.log.warn("[OSL] Ankle is not connected.")

    @property
    def loadcell(self):
        if self.has_loadcell:
            return self._loadcell
        else:
            self.log.warn("[OSL] Loadcell is not connected.")

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


if __name__ == "__main__":
    osl = OpenSourceLeg()
    osl.tui.run()
