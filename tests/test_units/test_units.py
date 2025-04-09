# Global Units Dictionary

import numpy as np
import pytest
from opensourceleg.units import (
    UnitType,
    Force,
    Torque,
    Length,
    Position,
    Mass,
    Acceleration,
    UnitsManager,
    with_units
)

# Example functions using the decorators
@with_units(UnitType.FORCE, UnitType.LENGTH, return_type=UnitType.TORQUE)
def calculate_work(force: float, distance: float) -> float:
    """Calculate work done by a force over a distance."""
    return force * distance

@with_units(UnitType.TORQUE, UnitType.POSITION, return_type=UnitType.TORQUE)
def calculate_rotational_work(torque: float, angle: float) -> float:
    """Calculate rotational work done by a torque over an angle."""
    return torque * angle

@with_units(UnitType.MASS, UnitType.ACCELERATION, return_type=UnitType.FORCE)
def calculate_force(mass: float, acceleration: float) -> float:
    """Calculate force from mass and acceleration."""
    return mass * acceleration

@with_units(UnitType.FORCE, UnitType.LENGTH, return_type=UnitType.TORQUE)
def calculate_torque(force: float, lever_arm: float) -> float:
    """Calculate torque from force and lever arm."""
    return force * lever_arm

def test_basic_unit_conversion():
    """Test basic unit conversion with float inputs and outputs."""
    work = calculate_work(10, 5)  # 10 N * 5 m
    assert isinstance(work, float)
    assert work == 50.0  # Result in N⋅m

def test_force_calculation():
    """Test force calculation with mass and acceleration."""
    force = calculate_force(5, 2)  # 5 kg * 2 m/s²
    assert isinstance(force, float)
    assert force == 10.0  # Result in N

def test_torque_calculation():
    """Test torque calculation with force and lever arm."""
    torque = calculate_torque(10, 0.5)  # 10 N * 0.5 m
    assert isinstance(torque, float)
    assert torque == 5.0  # Result in N⋅m

def test_rotational_work():
    """Test rotational work calculation with torque and angle."""
    work = calculate_rotational_work(10, np.pi/2)  # 10 N⋅m * π/2 rad
    assert isinstance(work, float)
    assert work == pytest.approx(15.707963267948966)  # Result in N⋅m

def test_negative_values():
    """Test handling of negative values."""
    # Test negative force
    work = calculate_work(-10, 5)
    assert work == -50.0
    
    # Test negative distance
    work = calculate_work(10, -5)
    assert work == -50.0
    
    # Test negative torque
    work = calculate_rotational_work(-10, np.pi/2)
    assert work == pytest.approx(-15.707963267948966)

def test_wrong_return_type():
    """Test error handling for wrong return type."""
    @with_units(UnitType.FORCE, return_type=UnitType.TORQUE)
    def wrong_return_type(force: float) -> float:
        return force  # Wrong return type (should be torque)
    
    with pytest.raises(ValueError, match="Function returned Quantity of type force, expected torque"):
        wrong_return_type(10)

def test_user_defined_default_units():
    """Test that decorator respects user-defined default units."""
    manager = UnitsManager()
    
    # Test with SI units (defaults)
    work = calculate_work(10, 5)  # 10 N * 5 m
    assert work == 50.0  # Result in N⋅m
    
    # Change defaults to imperial units
    manager.set_default_unit(UnitType.FORCE, Force.lbf)
    manager.set_default_unit(UnitType.LENGTH, Length.inch)
    
    # Test with imperial units
    work = calculate_work(10, 5)  # 10 lbf * 5 inches
    assert work == 50.0  # Result in lbf⋅inch
    
    # Test force calculation with imperial units
    manager.set_default_unit(UnitType.MASS, Mass.lb)
    manager.set_default_unit(UnitType.ACCELERATION, Acceleration.ft_per_s2)
    force = calculate_force(5, 2)  # 5 lb * 2 ft/s²
    assert force == 10.0  # Result in lbf
    
    # Test rotational work with imperial units
    manager.set_default_unit(UnitType.TORQUE, Torque.lbf_inch)
    manager.set_default_unit(UnitType.POSITION, Position.deg)
    work = calculate_rotational_work(10, 90)  # 10 lbf⋅inch * 90 deg
    assert work == pytest.approx(900.0)  # Result in lbf⋅inch
    
    # Reset defaults to SI units
    manager.set_default_unit(UnitType.FORCE, Force.N)
    manager.set_default_unit(UnitType.LENGTH, Length.m)
    manager.set_default_unit(UnitType.MASS, Mass.kg)
    manager.set_default_unit(UnitType.ACCELERATION, Acceleration.m_per_s2)
    manager.set_default_unit(UnitType.TORQUE, Torque.N_m)
    manager.set_default_unit(UnitType.POSITION, Position.rad)
    
    # Verify back to SI units
    work = calculate_work(10, 5)  # 10 N * 5 m
    assert work == 50.0  # Result in N⋅m

def test_unit_conversion_consistency():
    """Test that unit conversions are consistent across different default units."""
    manager = UnitsManager()
    
    # Test with SI units
    work_si = calculate_work(10, 5)  # 10 N * 5 m = 50 N⋅m
    
    # Change to imperial units
    manager.set_default_unit(UnitType.FORCE, Force.lbf)
    manager.set_default_unit(UnitType.LENGTH, Length.inch)
    
    # Convert SI values to imperial
    force_lbf = 10 * Force.lbf.value  # Convert 10 N to lbf
    distance_inch = 5 * Length.inch.value  # Convert 5 m to inches
    work_imperial = calculate_work(force_lbf, distance_inch)
    
    # Results should be equivalent
    assert work_imperial == pytest.approx(work_si * Torque.lbf_inch.value) 