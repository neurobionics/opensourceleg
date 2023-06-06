

import opensourceleg.units as units
import pytest


@pytest.mark.parametrize(
    ("attribute", "unit", "expected"),
    [
        ("force", "lbf", "lbf"),
        ("length", "in", "in")
    ]
)

def test_setter(attribute, unit, expected):
    default_units = units.DEFAULT_UNITS
    default_units[attribute] = unit
    assert default_units[attribute] == expected
    






@pytest.mark.parametrize(
    ("attribute", "unit", "expected_output", "expected_error"),
    [
        ("forc", "N", "'Invalid key: forc'", KeyError),
        ("force", "n", "Invalid unit: n", ValueError), 
        ("", "N", "'Invalid key: '", KeyError),
        ("force", "", "Invalid unit: ", ValueError),
    ]
)


def test_setter_error(attribute, unit, expected_output, expected_error):
    default_units = units.DEFAULT_UNITS
    
    with pytest.raises(Exception) as e:
        default_units[attribute] = unit
    assert str(e.value) == expected_output
    assert type(e.value) == expected_error


@pytest.mark.parametrize(
    ("attribute", "expected_output", "expected_error"),
    
    [
        ("tork", "'Invalid key: tork'", KeyError),
        ("", "'Invalid key: '", KeyError)
    ]
    
)

def test_getter_error(attribute, expected_output, expected_error):
    default_units = units.DEFAULT_UNITS
    with pytest.raises(Exception) as e:
        default_units[attribute]
    assert str(e.value) == expected_output
    assert type(e.value) == expected_error



@pytest.mark.parametrize(
    ("value", "attribute", "unit", "expected"),
    [
        (2, "force", "N", 2),
        (2, "torque", "N-m", 2),
        (2, "stiffness", "N/rad", 2),
        (2, "damping", "N/(rad/s)", 2),
        (2, "length", "m", 2),
        (2, "position", "rad", 2),
        (2, "mass", "kg", 2),
        (2, "velocity", "rad/s", 2),
        (2, "acceleration", "rad/s^2", 2),
        (2, "time", "s", 2),
        (2, "current", "mA", 2),
        (2, "voltage", "mV", 2),
        (2, "gravity", "m/s^2", 2),
        (2, "temperature", "C", 2),
        (20, "force", "lbf", 20 * 4.4482216152605),
        (-30, "force", "lbf", -30 * 4.4482216152605),
        (20, "force", "kgf", 20 * 9.80665),
        (-30, "force", "kgf", -30 * 9.80665),
        (20, "torque", "lbf-in", 20 * 0.1129848290276167),
        (-30, "torque", "lbf-in", -30 * 0.1129848290276167),
        (20, "torque", "lbf-ft", 20 * 1.3558179483314004),
        (-30, "torque", "lbf-ft", -30 * 1.3558179483314004),
        (20, "torque", "kgf-cm", 20 * 0.0980665),
        (-30, "torque", "kgf-cm", -30 * 0.0980665),
        (20, "torque", "kgf-m", 20 * 0.980665),
        (-30, "torque", "kgf-m", -30 * 0.980665),
        (20, "stiffness", "N/deg", 20 * 0.017453292519943295),
        (-30, "stiffness", "N/deg", -30 * 0.017453292519943295),
        (20, "stiffness", "lbf/rad", 20 * 0.224809),
        (-30, "stiffness", "lbf/rad", -30 * 0.224809),
        (20, "stiffness", "lbf/deg", 20 * 0.003490659),
        (-30, "stiffness", "lbf/deg", -30 * 0.003490659),
        (20, "stiffness", "kgf/rad", 20 * 1.8518518518518519),
        (-30, "stiffness", "kgf/rad", -30 * 1.8518518518518519),
        (20, "stiffness", "kgf/deg", 20 * 0.031746031746031744),
        (-30, "stiffness", "kgf/deg", -30 * 0.031746031746031744),
        (20, "damping", "N/(deg/s)", 20 * 0.017453292519943295),
        (-30, "damping", "N/(deg/s)", -30 * 0.017453292519943295),
        (20, "damping", "lbf/(deg/s)", 20 * 0.003490659),
        (-30, "damping", "lbf/(deg/s)", -30 * 0.003490659),
        (20, "damping", "lbf/(rad/s)", 20 * 0.224809),
        (-30, "damping", "lbf/(rad/s)", -30 * 0.224809),
        (20, "damping", "kgf/(rad/s)", 20 * 1.8518518518518519),
        (-30, "damping", "kgf/(rad/s)", -30 * 1.8518518518518519),
        (20, "damping", "kgf/(deg/s)", 20 * 0.031746031746031744),
        (-30, "damping", "kgf/(deg/s)", -30 * 0.031746031746031744),
        (20, "length", "cm", 20 * 0.01),
        (-30, "length", "cm", -30 * 0.01),
        (20, "length", "in", 20 * 0.0254),
        (-30, "length", "in", -30 * 0.0254),
        (20, "length", "ft", 20 * 0.3048),
        (-30, "length", "ft", -30 * 0.3048),
        (20, "position", "deg", 20 * 0.017453292519943295),
        (-30, "position", "deg", -30 * 0.017453292519943295),
        (20, "mass", "g", 20 * 0.001),
        (-30, "mass", "g", -30 * 0.001),
        (20, "mass", "lb", 20 * 0.45359237),
        (-30, "mass", "lb", -30 * 0.45359237),
        (20, "velocity", "deg/s", 20 * 0.017453292519943295),
        (-30, "velocity", "deg/s", -30 * 0.017453292519943295),
        (20, "velocity", "rpm", 20 * 0.10471975511965977),
        (-30, "velocity", "rpm", -30 * 0.10471975511965977),
        (20, "acceleration", "deg/s^2", 20 * 0.017453292519943295),
        (-30, "acceleration", "deg/s^2", -30 * 0.017453292519943295),
        (20, "time", "ms", 20 * 0.001),
        (-30, "time", "ms", -30 * 0.001),
        (20, "time", "us", 20 * 0.000001),
        (-30, "time", "us", -30 * 0.000001),
        (20, "current", "A", 20 * 1000),
        (-30, "current", "A", -30 * 1000),
        (20, "voltage", "V", 20 * 1000),
        (-30, "voltage", "V", -30 * 1000),
        (20, "gravity", "g", 20 * 9.80665),
        (-30, "gravity", "g", -30 * 9.80665),
        (20, "temperature", "F", (20 - 32) * 0.55556),
        (-30, "temperature", "F", (-30 - 32) * 0.55556),
        (20, "temperature", "K", 20 - 273.15),
        (-30, "temperature", "K", -30 - 273.15)   
    ]
)

def test_convert_to_default_units(value, attribute, unit, expected):
    default_units = units.DEFAULT_UNITS
    default_units[attribute] = unit
    assert default_units.convert_to_default_units(value, attribute) == expected

@pytest.mark.parametrize(
    ("value", "attribute", "unit", "expected"),
    [
        (2, "force", "N", 2),
        (2, "torque", "N-m", 2),
        (2, "stiffness", "N/rad", 2),
        (2, "damping", "N/(rad/s)", 2),
        (2, "length", "m", 2),
        (2, "position", "rad", 2),
        (2, "mass", "kg", 2),
        (2, "velocity", "rad/s", 2),
        (2, "acceleration", "rad/s^2", 2),
        (2, "time", "s", 2),
        (2, "current", "mA", 2),
        (2, "voltage", "mV", 2),
        (2, "gravity", "m/s^2", 2),
        (2, "temperature", "C", 2),
        (20, "force", "lbf", 20 / 4.4482216152605),
        (-30, "force", "lbf", -30 / 4.4482216152605),
        (20, "force", "kgf", 20 / 9.80665),
        (-30, "force", "kgf", -30 / 9.80665),
        (20, "torque", "lbf-in", 20 / 0.1129848290276167),
        (-30, "torque", "lbf-in", -30 / 0.1129848290276167),
        (20, "torque", "lbf-ft", 20 / 1.3558179483314004),
        (-30, "torque", "lbf-ft", -30 / 1.3558179483314004),
        (20, "torque", "kgf-cm", 20 / 0.0980665),
        (-30, "torque", "kgf-cm", -30 / 0.0980665),
        (20, "torque", "kgf-m", 20 / 0.980665),
        (-30, "torque", "kgf-m", -30 / 0.980665),
        (20, "stiffness", "N/deg", 20 / 0.017453292519943295),
        (-30, "stiffness", "N/deg", -30 / 0.017453292519943295),
        (20, "stiffness", "lbf/rad", 20 / 0.224809),
        (-30, "stiffness", "lbf/rad", -30 / 0.224809),
        (20, "stiffness", "lbf/deg", 20 / 0.003490659),
        (-30, "stiffness", "lbf/deg", -30 / 0.003490659),
        (20, "stiffness", "kgf/rad", 20 / 1.8518518518518519),
        (-30, "stiffness", "kgf/rad", -30 / 1.8518518518518519),
        (20, "stiffness", "kgf/deg", 20 / 0.031746031746031744),
        (-30, "stiffness", "kgf/deg", -30 / 0.031746031746031744),
        (20, "damping", "N/(deg/s)", 20 / 0.017453292519943295),
        (-30, "damping", "N/(deg/s)", -30 / 0.017453292519943295),
        (20, "damping", "lbf/(deg/s)", 20 / 0.003490659),
        (-30, "damping", "lbf/(deg/s)", -30 / 0.003490659),
        (20, "damping", "lbf/(rad/s)", 20 / 0.224809),
        (-30, "damping", "lbf/(rad/s)", -30 / 0.224809),
        (20, "damping", "kgf/(rad/s)", 20 / 1.8518518518518519),
        (-30, "damping", "kgf/(rad/s)", -30 / 1.8518518518518519),
        (20, "damping", "kgf/(deg/s)", 20 / 0.031746031746031744),
        (-30, "damping", "kgf/(deg/s)", -30 / 0.031746031746031744),
        (20, "length", "cm", 20 / 0.01),
        (-30, "length", "cm", -30 / 0.01),
        (20, "length", "in", 20 / 0.0254),
        (-30, "length", "in", -30 / 0.0254),
        (20, "length", "ft", 20 / 0.3048),
        (-30, "length", "ft", -30 / 0.3048),
        (20, "position", "deg", 20 / 0.017453292519943295),
        (-30, "position", "deg", -30 / 0.017453292519943295),
        (20, "mass", "g", 20 / 0.001),
        (-30, "mass", "g", -30 / 0.001),
        (20, "mass", "lb", 20 / 0.45359237),
        (-30, "mass", "lb", -30 / 0.45359237),
        (20, "velocity", "deg/s", 20 / 0.017453292519943295),
        (-30, "velocity", "deg/s", -30 / 0.017453292519943295),
        (20, "velocity", "rpm", 20 / 0.10471975511965977),
        (-30, "velocity", "rpm", -30 / 0.10471975511965977),
        (20, "acceleration", "deg/s^2", 20 / 0.017453292519943295),
        (-30, "acceleration", "deg/s^2", -30 /0.017453292519943295),
        (20, "time", "ms", 20 / 0.001),
        (-30, "time", "ms", -30 / 0.001),
        (20, "time", "us", 20 / 0.000001),
        (-30, "time", "us", -30 / 0.000001),
        (20, "current", "A", 20 / 1000),
        (-30, "current", "A", -30 / 1000),
        (20, "voltage", "V", 20 / 1000),
        (-30, "voltage", "V", -30 / 1000),
        (20, "gravity", "g", 20 / 9.80665),
        (-30, "gravity", "g", -30 / 9.80665),
        (20, "temperature", "F", 20 / 0.55556 + 32),
        (-30, "temperature", "F", -30 / 0.55556 + 32),
        (20, "temperature", "K", 20 + 273.15),
        (-30, "temperature", "K", -30 + 273.15)   
    ]
)

def test_convert_from_default_units(value, attribute, unit, expected):
    default_units = units.DEFAULT_UNITS
    default_units[attribute] = unit
    assert default_units.convert_from_default_units(value, attribute) == expected







