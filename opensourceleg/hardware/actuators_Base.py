"""
New Actuator Module for Abstract Base Class
Updated 05/2024
"""

from typing import Any, Callable, Union, overload

import ctypes
import os
import time
from ctypes import c_int
from dataclasses import dataclass
import numpy as np
# Abstract Base Class Definition
from abc import ABC, abstractmethod

import flexsea.fx_enums as fxe

"""


from flexsea.device import Device

from ..tools.logger import Logger
from .thermal import ThermalModel

"""


"""
Module Overview:

This module defines classes related to controlling the Dephy Actpack, including control modes,
gains, and Actpack modes. It also provides a class for the Dephy Actpack itself.

Key Classes:

- `ControlModes`: Enumerates available control modes for the Dephy Actpack.
- `Gains`: Dataclass for controller gains.
- `ActpackMode`: Base class for Actpack modes, including `VoltageMode`, `CurrentMode`, `PositionMode`, and `ImpedanceMode`.
- `ActpackControlModes`: Enumerates available Actpack modes.
- `DephyActpack`: Class for interacting with the Dephy Actpack.

Usage Guide:

1. Create an instance of `DephyActpack` with appropriate parameters (e.g., port, baud_rate, frequency).
2. Start the actpack using the `start` method.
3. Set the desired control mode using the `set_mode` method.
4. Set gains for the selected control mode using methods like `set_position_gains`, `set_current_gains`, etc.
5. Optionally, update the actpack using the `update` method to query the latest values.
6. Stop the actpack using the `stop` method.

"""




@dataclass
class ControlModes:
    """
    Control modes for the Dephy Actpack.

    Available modes are Voltage, Current, Position, Impedance.
    
    ## Remain Unchanged
    """

    voltage: ctypes.c_int = fxe.FX_VOLTAGE
    current: ctypes.c_int = fxe.FX_CURRENT
    position: ctypes.c_int = fxe.FX_POSITION
    impedance: ctypes.c_int = fxe.FX_IMPEDANCE

    def __repr__(self) -> str:
        return f"ControlModes"


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
        
    ## Remain structure Unchanged, values change through actuator def class
    """

    kp: int = 0
    ki: int = 0
    kd: int = 0
    K: int = 0
    B: int = 0
    ff: int = 0

    def __repr__(self) -> str:
        return f"kp={self.kp}, ki={self.ki}, kd={self.kd}, K={self.K}, B={self.B}, ff={self.ff}"


"""
Section for control parameters definition, needs to be packed into base class
"""
class MotorParam(ABC):
    @abstractmethod
    def __init__(self) -> None:
        super().__init__()

CONTROL_MODE = ControlModes()

MOTOR_COUNT_PER_REV: float = 16384
NM_PER_AMP: float = 0.1133
NM_PER_MILLIAMP: float = NM_PER_AMP / 1000
RAD_PER_COUNT: float = 2 * np.pi / MOTOR_COUNT_PER_REV
RAD_PER_DEG: float = np.pi / 180

RAD_PER_SEC_GYROLSB: float = np.pi / 180 / 32.8
M_PER_SEC_SQUARED_ACCLSB: float = 9.80665 / 8192

IMPEDANCE_A: float = 0.00028444
IMPEDANCE_C: float = 0.0007812

NM_PER_RAD_TO_K: float = RAD_PER_COUNT / IMPEDANCE_C * 1e3 / NM_PER_AMP
NM_S_PER_RAD_TO_B: float = RAD_PER_DEG / IMPEDANCE_A * 1e3 / NM_PER_AMP

MAX_CASE_TEMPERATURE: float = 80

DEFAULT_POSITION_GAINS = Gains(kp=50, ki=0, kd=0, K=0, B=0, ff=0)

DEFAULT_CURRENT_GAINS = Gains(kp=40, ki=400, kd=0, K=0, B=0, ff=128)

DEFAULT_IMPEDANCE_GAINS = Gains(kp=40, ki=400, kd=0, K=200, B=400, ff=128)

class MotorObj(ABC):
    """
    Abstract Base Class for Motor Object
    """
    @abstractmethod
    def __init__(self) -> None:
        super().__init__()
    