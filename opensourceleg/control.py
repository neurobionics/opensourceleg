import ctypes
import os
import sys
from dataclasses import dataclass

import flexsea.fx_enums as fxe
import numpy as np
import numpy.ctypeslib as ctl

from opensourceleg.utilities import get_ctype_args


@dataclass
class ControlModes:
    """
    Control modes for the Dephy Actpack.

    Available modes are Voltage, Current, Position, Impedance.
    """

    voltage: ctypes.c_int = fxe.FX_VOLTAGE
    current: ctypes.c_int = fxe.FX_CURRENT
    position: ctypes.c_int = fxe.FX_POSITION
    impedance: ctypes.c_int = fxe.FX_IMPEDANCE


CONTROL_MODE = ControlModes()


@dataclass
class Gains:
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


class Vector2D(ctypes.Structure):
    __slots__ = ["x", "y"]
    _fields_ = [
        ("x", ctypes.c_double),
        ("y", ctypes.c_double),
    ]


class Vector3D(ctypes.Structure):
    __slots__ = ["x", "y", "z"]
    _fields_ = [
        ("x", ctypes.c_double),
        ("y", ctypes.c_double),
        ("z", ctypes.c_double),
    ]


class IMU(ctypes.Structure):
    __slots__ = ["angle", "velocity", "acceleration"]
    _fields_ = [
        ("angle", Vector3D),
        ("velocity", Vector3D),
        ("acceleration", Vector3D),
    ]


class Loadcell(ctypes.Structure):
    __slots__ = ["fx", "fy", "fz", "mx", "my", "mz"]
    _fields_ = [
        ("fx", ctypes.c_double),
        ("fy", ctypes.c_double),
        ("fz", ctypes.c_double),
        ("mx", ctypes.c_double),
        ("my", ctypes.c_double),
        ("mz", ctypes.c_double),
    ]


class PIDGains(ctypes.Structure):
    __slots__ = ["kp", "ki", "kd"]
    _fields_ = [
        ("kp", ctypes.c_double),
        ("ki", ctypes.c_double),
        ("kd", ctypes.c_double),
    ]


class PIDTerms(ctypes.Structure):
    __slots__ = ["p", "i", "d", "total"]
    _fields_ = [
        ("p", ctypes.c_double),
        ("i", ctypes.c_double),
        ("d", ctypes.c_double),
        ("total", ctypes.c_double),
    ]


class Sensors(ctypes.Structure):
    __slots__ = ["imu", "loadcell"]
    _fields_ = [
        ("imu", IMU),
        ("loadcell", Loadcell),
    ]


class Controller:
    """
    Controller class to load compiled controllers.
    """

    def __init__(self, frequency: float) -> None:
        """
        Initialize the Controller class.

        Parameters
        ----------
        frequency : float
            Frequency of the controller
        """
        self._frequency = frequency
        self._lib: ctypes.CDLL = None

        self._sensors = Sensors()

    def load(self, controller_name, controller_path) -> ctypes.CDLL:
        self.lib = ctl.load_library(controller_name, controller_path)


if __name__ == "__main__":

    from opensourceleg.control import Controller

    hkic_controller = Controller(200)
    hkic_controller.load("hkic_controller.so", "../")
