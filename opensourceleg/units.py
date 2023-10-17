# Global Units Dictionary
import enum
from dataclasses import dataclass


@dataclass
class force:
    N = 1.0
    lbf = 4.4482216152605
    kgf = 9.80665


@dataclass
class torque:
    N_m = 1.0
    lbf_inch = 0.1129848290276167
    kgf_cm = 0.0980665


@dataclass
class stiffness:
    N_m_per_rad = 1.0
    N_m_per_deg = 0.017453292519943295


@dataclass
class damping:
    N_m_per_rad_per_s = 1.0
    N_m_per_deg_per_s = 0.017453292519943295


@dataclass
class length:
    m = 1.0
    cm = 0.01
    inch = 0.0254


@dataclass
class position:
    rad = 1.0
    deg = 0.017453292519943295


@dataclass
class mass:
    kg = 1.0
    g = 0.001
    lb = 0.45359237


@dataclass
class velocity:
    rad_per_s = 1.0
    deg_per_s = 0.017453292519943295
    rpm = 0.10471975511965977


@dataclass
class acceleration:
    rad_per_s2 = 1.0
    deg_per_s2 = 0.017453292519943295


@dataclass
class time:
    s = 1.0
    ms = 0.001


@dataclass
class current:
    mA = 1
    A = 1000


@dataclass
class voltage:
    mV = 1
    V = 1000


def convert_to_default(value: float, from_unit: float) -> float:
    """
    Convert a value from an user unit to the default unit.

    Args:
        value (float): Value to convert
        from_unit (float): Unit corresponding to the value that is being converted to the default unit.

    Returns:
        float: Converted value in default units.

    Example:
        >>> convert_to_default(2, units.current.A)
        2000 # returns value in mA, which is the default unit for current

        >>> convert_to_default(10, units.voltage.V)
        10000 # returns value in mV, which is the default unit for voltage

        >>> convert_to_default(45, units.position.deg)
        0.7853981633974483 # returns value in rad, which is the default unit for position
    """
    return value * from_unit


def convert_from_default(value: float, to_unit: float) -> float:
    """
    Convert a value from the default unit to an user unit.

    Args:
        value (float): Value to convert
        to_unit (float): Desired unit for the converted value.

    Returns:
        float: Converted value in desired unit.

    Example:

        >>> convert_from_default(2000, units.current.A)
        2.0 # A

        >>> convert_from_default(10000, units.voltage.V)
        10.0 # V

        >>> convert_from_default(0.7853981633974483, units.position.deg)
        45.0 # deg
    """

    return value / to_unit


if __name__ == "__main__":
    print(convert_to_default(2, current.A))
    print(convert_to_default(10, voltage.V))
    print(convert_to_default(45, position.deg))
    print(convert_from_default(2000, current.A))
    print(convert_from_default(10000, voltage.V))
    print(convert_from_default(0.7853981633974483, position.deg))
