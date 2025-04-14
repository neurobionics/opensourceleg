from dataclasses import dataclass
from enum import Enum
from functools import wraps
from typing import Any, Callable, Optional, Union


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
UNIT_TYPE_MAP: dict[UnitType, type[BaseUnit]] = {
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
_default_units: dict[UnitType, BaseUnit] = {
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

    def to(self, unit: BaseUnit) -> "Quantity":
        """Convert the quantity to a different unit.

        Args:
            unit (BaseUnit): The target unit to convert to

        Returns:
            Quantity: A new Quantity instance with the value converted to the target unit

        Raises:
            TypeError: If the target unit is not compatible with this quantity's type

        Example:
            >>> force = Quantity(10, UnitType.FORCE, Force.N)
            >>> force_lbf = force.to(Force.lbf)
            >>> print(force_lbf)
            2.2481 lbf
        """
        if not isinstance(unit, type(self._unit)):
            raise TypeError(f"Cannot convert to unit of different type: {unit}")
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
        """Convert the quantity value to a float. No conversion is done.

        Returns:
            float: The quanitity's value converted to a float.
        """
        return float(self.value)

    def __eq__(self, other: Union["Quantity", float]) -> bool:
        """Compare this quantity with another quantity or float value.

        Args:
            other (Union[Quantity, float]): Another quantity or float value to compare with

        Returns:
            bool: True if the quantities are equal when converted to default units

        Raises:
            ValueError: If comparing quantities of different types
        """
        if isinstance(other, Quantity):
            if self._unit_type != other._unit_type:
                raise ValueError("Cannot compare quantities of different types")
            return self.in_default_unit == other.in_default_unit
        return self.in_default_unit == float(other)

    def __add__(self, other: Union["Quantity", float]) -> "Quantity":
        """Add a quantity with another quantity or float value. Assumes units are consistent if adding with a float.

        Args:
            other (Union[Quantity, float]): Another quantity or float value to add

        Returns:
            Quantity: A new quantity with the sum of the values

        Raises:
            ValueError: If the quantities are of different types
        """
        if isinstance(other, (int, float)):
            # Treat float as if its units are consistent with the quantity
            return Quantity(self.value + other, self._unit_type, self._unit)
        elif isinstance(other, Quantity):
            if self._unit_type != other._unit_type:
                raise ValueError("Cannot add quantities of different types")
            # Convert both to default units, add, then convert back to original unit
            sum_in_default = self.in_default_unit + other.in_default_unit
            return Quantity(sum_in_default / self._unit.value, self._unit_type, self._unit)
        else:
            raise TypeError("Can only add Quantity with Quantity or float")

    def __radd__(self, other: float) -> "Quantity":
        """Reverse addition for float values. Assumes units are consistent if adding with a float.

        Args:
            other (float): Float value to add

        Returns:
            Quantity: A new quantity with the sum
        """
        return self.__add__(other)

    def __sub__(self, other: Union["Quantity", float]) -> "Quantity":
        """Subtract a quantity from another quantity or float value.
        Assumes units are consistent if subtracting with a float.

        Args:
            other (Union[Quantity, float]): Another quantity or float value to subtract

        Returns:
            Quantity: A new quantity with the difference of the values

        Raises:
            ValueError: If the quantities are of different types
        """
        if isinstance(other, (int, float)):
            # Treat float as if its units are consistent with the quantity
            return Quantity(self.value - other, self._unit_type, self._unit)
        elif isinstance(other, Quantity):
            if self._unit_type != other._unit_type:
                raise ValueError("Cannot subtract quantities of different types")
            # Convert both to default units, subtract, then convert back to original unit
            diff_in_default = self.in_default_unit - other.in_default_unit
            return Quantity(diff_in_default / self._unit.value, self._unit_type, self._unit)
        else:
            raise TypeError("Can only subtract Quantity with Quantity or float")

    def __rsub__(self, other: float) -> "Quantity":
        """Reverse subtraction for Quantity and float values. Assumes units are consistent if subtracting with a float.

        Args:
            other (float): Float value to subtract from

        Returns:
            Quantity: A new quantity with the difference
        """
        if isinstance(other, (int, float)):
            # Treat float as if its units are consistent with the quantity
            return Quantity(other - self.value, self._unit_type, self._unit)
        else:  # Don't need to implement __rsub__ for Quantity as __sub__ handles this
            raise TypeError("Can only subtract Quantity from float")

    def __mul__(self, other: Union["Quantity", float]) -> "Quantity":
        """Multiply a quantity by a scalar or another quantity of the same type.
        Assumes units are consistent if multiplying with a float.

        Args:
            other (Union[Quantity, float]): Scalar value or another quantity

        Returns:
            Quantity: A new quantity with the product

        Raises:
            ValueError: If multiplying with incompatible quantity types
        """
        if isinstance(other, (int, float)):
            # Scalar multiplication
            return Quantity(self._value * other, self._unit_type, self._unit)
        elif isinstance(other, Quantity):
            # Special cases for physical quantity multiplication
            if self._unit_type == other._unit_type:
                result_in_default = self.in_default_unit * other.in_default_unit
                return Quantity(result_in_default / self._unit.value, self._unit_type, self._unit)
            else:
                raise ValueError(f"Cannot multiply {self._unit_type} with {other._unit_type}")
        else:
            raise TypeError("Can only multiply Quantity with scalar or Quantity")

    def __rmul__(self, other: float) -> "Quantity":
        """Reverse multiplication for Quantity and float values.
        Assumes units are consistent if multiplying with a float.

        Args:
            other (float): Scalar value to multiply with

        Returns:
            Quantity: A new quantity with the product
        """
        return self.__mul__(other)

    def __truediv__(self, other: Union["Quantity", float]) -> "Quantity":
        """Divide a quantity by a scalar or another quantity of the same type.
        Assumes units are consistent if dividing with a float.

        Args:
            other (Union[Quantity, float]): Scalar value or another quantity

        Returns:
            Quantity: A new quantity with the quotient

        Raises:
            ValueError: If dividing by incompatible quantity types
        """
        if isinstance(other, (int, float)):
            # Scalar division
            return Quantity(self._value / other, self._unit_type, self._unit)
        elif isinstance(other, Quantity):
            # Special cases for physical quantity division
            if self._unit_type == other._unit_type:
                # Torque / Position = Stiffness
                result_in_default = self.in_default_unit / other.in_default_unit
                return Quantity(result_in_default / self._unit.value, self._unit_type, self._unit)
            else:
                raise ValueError(f"Cannot divide {self._unit_type} by {other._unit_type}")
        else:
            raise TypeError("Can only divide Quantity by scalar or Quantity")

    def __rtruediv__(self, other: float) -> "Quantity":
        """Reverse division for Quantity and float values. Assumes units are consistent if dividing with a float.

        Args:
            other (float): Scalar value to divide by

        Returns:
            Quantity: A new quantity with the quotient
        """
        if isinstance(other, (int, float)):
            return Quantity(other / self._value, self._unit_type, self._unit)
        else:
            raise TypeError("Can only divide scalar by Quantity")


@dataclass
class UnitConfig:
    """Configuration for a unit type."""

    default_unit: BaseUnit
    allow_negative: bool = True


class UnitsManager:
    """Singleton class for managing units and conversions.

    This class provides a centralized way to handle unit conversions and default
    unit configurations. It uses a singleton pattern to ensure consistent unit
    handling throughout the application.

    The manager maintains two sets of defaults:
    1. SI defaults (immutable) - Used for internal calculations
    2. User defaults (mutable) - Used for input/output

    Example:
        >>> manager = UnitsManager()
        >>> manager.set_default_unit(UnitType.FORCE, Force.lbf)
        >>> force = manager.create_quantity(10, UnitType.FORCE)
        >>> print(force)
        10 lbf
    """

    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if not hasattr(self, "_initialized"):
            self._user_defaults: dict[UnitType, BaseUnit] = {}
            self._initialized = True

    def get_default_unit(self, unit_type: UnitType) -> BaseUnit:
        """Get the default unit for a given unit type.

        Returns the user-defined default if set, otherwise returns the SI default.

        Args:
            unit_type: The type of unit to get the default for

        Returns:
            The default unit for the given type (user default if set, otherwise SI default)
        """
        return self._user_defaults.get(unit_type, _default_units[unit_type])

    def set_default_unit(self, unit_type: UnitType, unit: BaseUnit) -> None:
        """Set the user default unit for a given unit type.

        Args:
            unit_type: The type of unit to set the default for
            unit: The unit to set as default

        Raises:
            TypeError: If the unit is not compatible with the unit type
        """
        if not isinstance(unit, UNIT_TYPE_MAP[unit_type]):
            raise TypeError(f"Unit {unit} is not compatible with type {unit_type}")
        self._user_defaults[unit_type] = unit

    def reset_to_si_defaults(self) -> None:
        """Reset all user defaults to SI defaults."""
        self._user_defaults.clear()

    def create_quantity(self, value: float, unit_type: UnitType, unit: Optional[BaseUnit] = None) -> Quantity:
        """Create a Quantity object with the given value and unit type.

        Args:
            value: The numeric value
            unit_type: The type of unit
            unit: Optional specific unit to use. If None, uses the user default (or SI default if no user default)

        Returns:
            A new Quantity object

        Raises:
            ValueError: If the unit is not compatible with the unit type
        """
        if unit is None:
            unit = self.get_default_unit(unit_type)
        return Quantity(value, unit_type, unit)


def with_units(*unit_types: UnitType, return_type: UnitType):
    """Decorator for handling unit conversions in function arguments and return values.

    This decorator handles unit conversions between user-defined defaults and SI units:
    1. Converts input values from user-defaults to SI units for calculations
    2. Performs calculations in SI units
    3. Converts results back to user-defaults for output

    Args:
        *unit_types: Variable number of UnitType arguments specifying the expected unit types
                    for the function arguments in order
        return_type: The UnitType that the function should return

    Returns:
        A decorator function that handles unit conversions

    Example:
        >>> @with_units(UnitType.FORCE, UnitType.LENGTH, return_type=UnitType.TORQUE)
        ... def calculate_work(force: float, distance: float) -> float:
        ...     return force * distance
        >>> work = calculate_work(10, 5)  # Returns 50.0 (in user-default units)
    """

    def decorator(func: Callable[..., Any]) -> Callable[..., Any]:
        @wraps(func)
        def wrapper(*args: Any, **kwargs: Any) -> Any:
            manager = UnitsManager()

            # Convert input values from user-defaults to SI units
            converted_args = []
            for arg, unit_type in zip(args, unit_types):
                # Create Quantity in user-default unit
                user_quantity = Quantity(float(arg), unit_type, manager.get_default_unit(unit_type))
                # Convert to SI unit for calculation
                si_unit = _default_units[unit_type]
                si_quantity = user_quantity.to(si_unit)
                converted_args.append(si_quantity)

            # Call the function with SI unit quantities
            result = func(*converted_args, **kwargs)

            # Convert result back to user-default unit if it's a Quantity
            if isinstance(result, Quantity):
                if result._unit_type != return_type:
                    raise ValueError(f"Function returned Quantity of type {result._unit_type}, expected {return_type}")
                # Convert from SI to user-default unit
                user_unit = manager.get_default_unit(return_type)
                user_quantity = result.to(user_unit)
                return user_quantity.value

            # If result is already the correct type, return it
            return result

        return wrapper

    return decorator


if __name__ == "__main__":
    # Create quantities with explicit units
    force = Quantity(10, UnitType.FORCE, Force.N)
    print(f"Force: {force}")  # 10 N

    # Convert to different unit
    force_lbf = force.to(Force.lbf)
    print(f"Force in lbf: {force_lbf}")  # ~2.25 lbf

    # units manager usage
    um = UnitsManager()
    um.set_default_unit(UnitType.POSITION, Position.deg)

    @with_units(UnitType.POSITION, UnitType.POSITION, return_type=UnitType.POSITION)
    def calc_angle(angle1: float, angle2: float) -> float:
        return angle1 + angle2 + 1.56

    angle = calc_angle(90, 45)
    print(angle)
