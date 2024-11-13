# Global Units Dictionary

import numpy as np
import pytest

from opensourceleg.units import *

# Values to iterate over for testing
VALUES = np.append(np.array([0, 1, -1, 1000, -1000]), np.random.random(5))


def test_convert_to_from_default():
    # Testing behaviour when invalid input type is passed to convert_to_default & convert_from_default
    with pytest.raises(TypeError):
        convert_to_default("TEST", force.kgf)
    with pytest.raises(TypeError):
        convert_to_default("T", force.kgf)
    with pytest.raises(TypeError):
        convert_from_default("TEST", force.N)
    with pytest.raises(TypeError):
        convert_from_default("T", force.N)

    # Testing behaviour on all possible unit conversions to default & from default
    categories = [
        force,
        torque,
        stiffness,
        damping,
        length,
        position,
        mass,
        velocity,
        acceleration,
        time,
        current,
        voltage,
    ]
    units = []
    # Add all units to array
    for c in categories:
        for unit in dir(c):
            if not unit.startswith("__"):
                value = getattr(c, unit)
                print(value)
                units.append(value)

    # Iterate over all possible units and test converting to/from default
    for unit in units:
        for value in VALUES:
            assert convert_to_default(value, unit) == value * unit
            assert convert_from_default(value, unit) == value / unit


def test_force():
    assert force.N == 1.0
    assert force.lbf == 4.4482216152605
    assert force.kgf == 9.80665
    # assert repr(force) == "force"


def test_torque():
    assert torque.N_m == 1.0
    assert torque.lbf_inch == 0.1129848290276167
    assert torque.kgf_cm == 0.0980665
    # assert repr(torque) == "torque"


def test_stiffness():
    assert stiffness.N_m_per_rad == 1.0
    assert stiffness.N_m_per_deg == 0.017453292519943295

    # assert repr(stiffness) == "stiffness"


def test_damping():
    assert damping.N_m_per_rad_per_s == 1.0
    assert damping.N_m_per_deg_per_s == 0.017453292519943295

    # assert repr(damping) == "damping"


def test_length():
    assert length.m == 1.0
    assert length.cm == 0.01
    assert length.inch == 0.0254

    # assert repr(length) == "length"


def test_position():
    assert position.rad == 1.0
    assert position.deg == 0.017453292519943295

    # assert repr(position) == "position"


def test_mass():
    assert mass.kg == 1.0
    assert mass.g == 0.001
    assert mass.lb == 0.45359237

    # assert repr(mass) == "mass"


def test_velocity():
    assert velocity.rad_per_s == 1.0
    assert velocity.deg_per_s == 0.017453292519943295
    assert velocity.rpm == 0.10471975511965977

    # assert repr(velocity) == "velocity"


def test_acceleration():
    assert acceleration.rad_per_s2 == 1.0
    assert acceleration.deg_per_s2 == 0.017453292519943295

    # assert repr(acceleration) == "acceleration"


def test_time():
    assert time.s == 1.0
    assert time.ms == 0.001

    # assert repr(time) == "time"


def test_current():
    assert current.mA == 1
    assert current.A == 1000

    # assert repr(current) == "current"


def test_voltage():
    assert voltage.mV == 1
    assert voltage.V == 1000

    # assert repr(voltage) == "voltage"
