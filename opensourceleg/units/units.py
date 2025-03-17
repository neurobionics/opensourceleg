from enum import Enum
from typing import Dict, Type, Union, Optional
import re

class UnitType(Enum):
    """Enumeration of physical quantity types supported by the units system.
    
    Each enum value represents a different physical quantity type and maps to a specific
    unit class that defines the available units for that quantity.
    
    Attributes:
        FORCE: Force quantities (N, lbf, kgf)
        TORQUE: Torque quantities (N_m, lbf_inch, kgf_cm)
        STIFFNESS: Stiffness quantities (N_m_per_rad, N_m_per_deg)
        DAMPING: Damping quantities (N_m_per_rad_per_s, N_m_per_deg_per_s)
        LENGTH: Length quantities (m, cm, inch)
        POSITION: Position quantities (rad, deg)
        MASS: Mass quantities (kg, g, lb)
        VELOCITY: Velocity quantities (rad_per_s, deg_per_s, rpm)
        ACCELERATION: Acceleration quantities (rad_per_s2, deg_per_s2)
        CURRENT: Current quantities (A, mA)
        VOLTAGE: Voltage quantities (V, mV)
    """
    FORCE = "force"
    TORQUE = "torque"
    STIFFNESS = "stiffness"
    DAMPING = "damping"
    LENGTH = "length"
    POSITION = "position"
    MASS = "mass"
    VELOCITY = "velocity"
    ACCELERATION = "acceleration"
    CURRENT = "current"
    VOLTAGE = "voltage"

class BaseUnit(float, Enum):
    """Base class for all unit enums.
    
    Inherits from both float and Enum to allow units to have conversion values
    while maintaining enum behavior. The conversion value represents the multiplier
    needed to convert from this unit to the default unit of the same type.
    
    Example:
        >>> class Length(BaseUnit):
        ...     m = 1.0  # Default unit
        ...     cm = 0.01  # 1 cm = 0.01 m
        ...     inch = 0.0254  # 1 inch = 0.0254 m
    """
    def __str__(self) -> str:
        """Returns the name of the unit as a string.
        
        Returns:
            str: The name of the unit (e.g., 'N', 'kg', 'rad')
        """
        return self.name

class Force(BaseUnit):
    N = 1.0  # Default
    lbf = 4.4482216152605
    kgf = 9.80665

class Torque(BaseUnit):
    N_m = 1.0  # Default
    lbf_inch = 0.1129848290276167
    kgf_cm = 0.0980665

class Stiffness(BaseUnit):
    N_m_per_rad = 1.0  # Default
    N_m_per_deg = 0.017453292519943295

class Damping(BaseUnit):
    N_m_per_rad_per_s = 1.0  # Default
    N_m_per_deg_per_s = 0.017453292519943295

class Length(BaseUnit):
    m = 1.0  # Default
    cm = 0.01
    inch = 0.0254

class Position(BaseUnit):
    rad = 1.0  # Default
    deg = 0.017453292519943295

class Mass(BaseUnit):
    kg = 1.0  # Default
    g = 0.001
    lb = 0.45359237

class Velocity(BaseUnit):
    rad_per_s = 1.0  # Default
    deg_per_s = 0.017453292519943295
    rpm = 0.10471975511965977

class Acceleration(BaseUnit):
    rad_per_s2 = 1.0  # Default
    deg_per_s2 = 0.017453292519943295

class Current(BaseUnit):
    A = 1.0  # Default
    mA = 0.001

class Voltage(BaseUnit):
    V = 1.0  # Default
    mV = 0.001

# Map unit types to their respective unit classes
UNIT_TYPE_MAP: Dict[UnitType, Type[BaseUnit]] = {
    UnitType.FORCE: Force,
    UnitType.TORQUE: Torque,
    UnitType.STIFFNESS: Stiffness,
    UnitType.DAMPING: Damping,
    UnitType.LENGTH: Length,
    UnitType.POSITION: Position,
    UnitType.MASS: Mass,
    UnitType.VELOCITY: Velocity,
    UnitType.ACCELERATION: Acceleration,
    UnitType.CURRENT: Current,
    UnitType.VOLTAGE: Voltage,
}

# Default units for each quantity type
_default_units: Dict[UnitType, BaseUnit] = {
    UnitType.FORCE: Force.N,
    UnitType.TORQUE: Torque.N_m,
    UnitType.STIFFNESS: Stiffness.N_m_per_rad,
    UnitType.DAMPING: Damping.N_m_per_rad_per_s,
    UnitType.LENGTH: Length.m,
    UnitType.POSITION: Position.rad,
    UnitType.MASS: Mass.kg,
    UnitType.VELOCITY: Velocity.rad_per_s,
    UnitType.ACCELERATION: Acceleration.rad_per_s2,
    UnitType.CURRENT: Current.A,
    UnitType.VOLTAGE: Voltage.V,
}

class Quantity:
    """A class representing a physical quantity with a value and unit.
    
    This class handles unit conversions and mathematical operations between quantities
    of the same type. It supports automatic conversion to default units and comparison
    operations.
    
    Attributes:
        _unit_type (UnitType): The type of physical quantity (e.g., FORCE, LENGTH)
        _unit (BaseUnit): The current unit of the quantity
        _value (float): The numeric value in the current unit
    
    Example:
        >>> force = Quantity(10, UnitType.FORCE, Force.N)
        >>> print(force)
        10 N
        >>> force_lbf = force.to(Force.lbf)
        >>> print(force_lbf)
        2.2481 lbf
        >>> force == force_lbf  # Comparison in default units
        True
    """
    
    def __init__(self, value: float, unit_type: UnitType, unit: Optional[BaseUnit] = None):
        """Initialize a new Quantity instance.
        
        Args:
            value (float): The numeric value of the quantity
            unit_type (UnitType): The type of physical quantity
            unit (Optional[BaseUnit], optional): The unit of the quantity. If None,
                uses the default unit for the unit_type. Defaults to None.
        
        Raises:
            ValueError: If the unit is not compatible with the unit_type
            TypeError: If value cannot be converted to float
        """
        self._unit_type = unit_type
        self._unit = unit or _default_units[unit_type]
        self._value = float(value)
        
    @property
    def value(self) -> float:
        """Get the value in the current unit.
        
        Returns:
            float: The numeric value in the current unit
        """
        return self._value
        
    @property
    def in_default_unit(self) -> float:
        """Get the value converted to the default unit.
        
        Returns:
            float: The numeric value converted to the default unit for this quantity type
        """
        return self._value * self._unit.value
        
    def to(self, unit: BaseUnit) -> 'Quantity':
        """Convert the quantity to a different unit.
        
        Args:
            unit (BaseUnit): The target unit to convert to
        
        Returns:
            Quantity: A new Quantity instance with the value converted to the target unit
        
        Raises:
            ValueError: If the target unit is not compatible with this quantity's type
        
        Example:
            >>> force = Quantity(10, UnitType.FORCE, Force.N)
            >>> force_lbf = force.to(Force.lbf)
            >>> print(force_lbf)
            2.2481 lbf
        """
        if not isinstance(unit, type(self._unit)):
            raise ValueError(f"Cannot convert to unit of different type: {unit}")
        new_value = self.in_default_unit / unit.value
        return Quantity(new_value, self._unit_type, unit)
        
    def __str__(self) -> str:
        """Returns a string representation of the quantity with its unit.
        
        Returns:
            str: String in the format "value unit" (e.g., "10 N", "45 deg")
        """
        return f"{self._value} {self._unit}"
        
    def __repr__(self) -> str:
        """Returns a detailed string representation of the quantity.
        
        Returns:
            str: String representation showing the constructor call
        """
        return f"Quantity({self._value}, {self._unit_type}, {self._unit})"
        
    def __float__(self) -> float:
        """Convert the quantity to a float in the default unit.
        
        Returns:
            float: The value converted to the default unit
        """
        return float(self.in_default_unit)
        
    def __eq__(self, other: Union['Quantity', float]) -> bool:
        """Compare this quantity with another quantity or float value.
        
        Args:
            other (Union[Quantity, float]): Another quantity or float value to compare with
        
        Returns:
            bool: True if the quantities are equal when converted to default units
        
        Raises:
            ValueError: If comparing quantities of different types
        
        Example:
            >>> force1 = Quantity(10, UnitType.FORCE, Force.N)
            >>> force2 = Quantity(2.2481, UnitType.FORCE, Force.lbf)
            >>> force1 == force2
            True
        """
        if isinstance(other, Quantity):
            if self._unit_type != other._unit_type:
                raise ValueError("Cannot compare quantities of different types")
            return self.in_default_unit == other.in_default_unit
        return self.in_default_unit == float(other)

def set_default_unit(unit_type: UnitType, unit: BaseUnit) -> None:
    """Set the default unit for a quantity type.
    
    Args:
        unit_type (UnitType): The type of physical quantity
        unit (BaseUnit): The unit to set as default
    
    Raises:
        ValueError: If the unit is not valid for the given unit_type
    
    Example:
        >>> set_default_unit(UnitType.POSITION, Position.deg)
        >>> angle = Quantity(90, UnitType.POSITION)  # Uses degrees by default
        >>> print(angle)
        90 deg
    """
    if not isinstance(unit, UNIT_TYPE_MAP[unit_type]):
        raise ValueError(f"Unit {unit} is not valid for type {unit_type}")
    _default_units[unit_type] = unit

def parse_quantity(value_str: str, unit_type: UnitType) -> Quantity:
    """Parse a string representation of a quantity.
    
    Args:
        value_str (str): String to parse in format "value unit" (e.g., "10 N", "45 deg")
        unit_type (UnitType): The expected type of physical quantity
    
    Returns:
        Quantity: A new Quantity instance parsed from the string
    
    Raises:
        ValueError: If the string format is invalid or the unit is unknown
        
    Example:
        >>> force = parse_quantity("10 N", UnitType.FORCE)
        >>> print(force)
        10 N
        >>> print(force.to(Force.lbf))
        2.2481 lbf
        
        >>> angle = parse_quantity("45 deg", UnitType.POSITION)
        >>> print(angle)
        45 deg
        >>> print(angle.to(Position.rad))
        0.7854 rad
    """
    match = re.match(r"^([-+]?\d*\.?\d*)\s*([a-zA-Z_]+)$", value_str.strip())
    if not match:
        raise ValueError(f"Invalid quantity format: {value_str}")
    
    value, unit_str = match.groups()
    unit_class = UNIT_TYPE_MAP[unit_type]
    
    try:
        unit = unit_class[unit_str]
    except KeyError:
        raise ValueError(f"Unknown unit '{unit_str}' for type {unit_type}")
    
    return Quantity(float(value), unit_type, unit)

# Example usage:
if __name__ == "__main__":
    # Create quantities with explicit units
    force = Quantity(10, UnitType.FORCE, Force.N)
    print(f"Force: {force}")  # 10 N
    
    # Convert to different unit
    force_lbf = force.to(Force.lbf)
    print(f"Force in lbf: {force_lbf}")  # ~2.25 lbf
    
    # Parse from string
    angle = parse_quantity("45 deg", UnitType.POSITION)
    print(f"Angle: {angle}")  # 45 deg
    print(f"Angle in radians: {angle.to(Position.rad)}")  # ~0.785 rad
    
    # Change default unit
    set_default_unit(UnitType.POSITION, Position.deg)
    new_angle = Quantity(90, UnitType.POSITION)  # Uses degrees by default now
    print(f"New angle: {new_angle}")  # 90 deg
