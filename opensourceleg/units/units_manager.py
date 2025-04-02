"""Units management system for OpenSourceLeg.

This module provides a simple units management system that handles unit conversions
and default unit configurations. It uses a singleton pattern to ensure consistent
unit handling throughout the application.
"""

from typing import Dict, Optional, Type, Callable, Any
from enum import Enum
from dataclasses import dataclass
from functools import wraps

from opensourceleg.units.units import (
    UnitType,
    BaseUnit,
    Quantity,
    UNIT_TYPE_MAP,
    _default_units,
)

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
        if not hasattr(self, '_initialized'):
            self._user_defaults: Dict[UnitType, BaseUnit] = {}
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
            ValueError: If the unit is not compatible with the unit type
        """
        if not isinstance(unit, UNIT_TYPE_MAP[unit_type]):
            raise ValueError(f"Unit {unit} is not compatible with type {unit_type}")
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
    def decorator(func: Callable[..., T]) -> Callable[..., T]:
        @wraps(func)
        def wrapper(*args: Any, **kwargs: Any) -> T:
            manager = UnitsManager()
            
            # Convert input values from user-defaults to SI units
            converted_args = []
            for arg, unit_type in zip(args, unit_types):
                # Create Quantity in user-default unit
                user_quantity = Quantity(float(arg), unit_type)
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
                return float(user_quantity)
            
            # If result is already the correct type, return it
            return result
        
        return wrapper
    return decorator 