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


class CompiledController:
    """
    Controller class to handle using compiled controllers.
    """

    def __init__(self, main_function_name, controller_path, initialization_function_name, cleanup_function_name) -> None:
        """
        Initialize the Controller class.

        Parameters
        ----------
        main_function_name : string
            Name of the main function to call within the library
        controller_path : string
            Full path to the .so object. 
        """
        # Load functions
        self.lib_cleanup = self._load_function(cleanup_function_name, controller_path)
        self.lib = self._load_function(main_function_name, controller_path)
        self.lib_init = self._load_function(initialization_function_name, controller_path)
        
        # Call init function if there is one
        if not self.lib_init == None:
            self.lib_init()

        # Alias the ctypes class as member of this class
        self.types = ctypes

        # Define default sensor list
        self.DEFAULT_SENSOR_LIST = [('knee_angle',self.types.c_double),
                                    ('ankle_angle',self.types.c_double)]
        
        # Initialize input and output lists
        self._input_type = None
        self.inputs = None
        self._output_type = None
        self.outputs = None

    def __del__(self):
        if not self.lib_cleanup == None:
            self.lib_cleanup()

    def _load_function(self, controller_name, controller_path) -> ctypes.CDLL:
        if controller_name == None:
            return None
        else:
            return ctl.load_library(controller_name, controller_path)

    def Define_Inputs(self, input_list):
        self._input_type = self.Define_Type('inputs', input_list)
        self.inputs = self._input_type()

    def Define_Outputs(self, output_list): 
        self._output_type = self.Define_Type('outputs',output_list)
        self.outputs = self._output_type()

    def Define_Type(self, type_name, parameter_list):
        slots = []
        for param in parameter_list:
                slots.append(param[0])
        class ctypes_structure(ctypes.Structure):
            _fields_ = parameter_list
            __slots__ = slots
        setattr(self.types, type_name, ctypes_structure)
        return getattr(self.types, type_name)

if __name__ == "__main__":

    pass
