# Global Units Dictionary

import numpy as np
import pytest

from opensourceleg.utilities.units import (
    Acceleration,
    Current,
    Damping,
    Force,
    Length,
    Mass,
    Position,
    Stiffness,
    Torque,
    Velocity,
    Voltage,
    convert_from_default,
    convert_to_default,
)

# Values to iterate over for testing
VALUES = np.append(np.array([0, 1, -1, 1000, -1000]), np.random.random(5))


def test_convert_to_from_default():
    # Testing behaviour when invalid input type is passed to convert_to_default & convert_from_default
    with pytest.raises(TypeError):
        convert_to_default("TEST", Force.kgf)
    with pytest.raises(TypeError):
        convert_to_default("T", Force.kgf)
    with pytest.raises(TypeError):
        convert_from_default("TEST", Force.N)
    with pytest.raises(TypeError):
        convert_from_default("T", Force.N)

    # Testing behaviour on all possible unit conversions to default & from default
    categories = [
        Force,
        Torque,
        Stiffness,
        Damping,
        Length,
        Position,
        Mass,
        Velocity,
        Acceleration,
        Current,
        Voltage,
    ]
    units = []
    # Add all units to array
    for c in categories:
        for unit in c:
            units.append(unit)

    # Iterate over all possible units and test converting to/from default
    for unit in units:
        for value in VALUES:
            assert convert_to_default(value, unit) == value * unit
            assert convert_from_default(value, unit) == value / unit


def test_force():
    assert Force.N == 1.0
    assert Force.lbf == 4.4482216152605
    assert Force.kgf == 9.80665


def test_torque():
    assert Torque.N_m == 1.0
    assert Torque.lbf_inch == 0.1129848290276167
    assert Torque.kgf_cm == 0.0980665


def test_stiffness():
    assert Stiffness.N_m_per_rad == 1.0
    assert Stiffness.N_m_per_deg == 0.017453292519943295


def test_damping():
    assert Damping.N_m_per_rad_per_s == 1.0
    assert Damping.N_m_per_deg_per_s == 0.017453292519943295


def test_length():
    assert Length.m == 1.0
    assert Length.cm == 0.01
    assert Length.inch == 0.0254


def test_position():
    assert Position.rad == 1.0
    assert Position.deg == 0.017453292519943295


def test_mass():
    assert Mass.kg == 1.0
    assert Mass.g == 0.001
    assert Mass.lb == 0.45359237


def test_velocity():
    assert Velocity.rad_per_s == 1.0
    assert Velocity.deg_per_s == 0.017453292519943295
    assert Velocity.rpm == 0.10471975511965977


def test_acceleration():
    assert Acceleration.rad_per_s2 == 1.0
    assert Acceleration.deg_per_s2 == 0.017453292519943295


def test_current():
    assert Current.mA == 1
    assert Current.A == 1000


def test_voltage():
    assert Voltage.mV == 1
    assert Voltage.V == 1000
