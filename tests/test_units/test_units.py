import pytest

import opensourceleg.units as units


# Testing the UnitsDefinition setter method
@pytest.mark.parametrize(
    ("attribute", "unit", "expected"), [("force", "lbf", "lbf"), ("length", "in", "in")]
)
def test_setter(attribute, unit, expected):

    """
    Tests the UnitsDefinition setter method\n
    Asserts the class variable of the UnitsDefinition object is equal to the expected value.
    """

    default_units = units.DEFAULT_UNITS
    default_units[attribute] = unit
    assert default_units[attribute] == expected


@pytest.mark.parametrize(
    ("attribute", "expected"),
    [("position", "rad"), ("acceleration", "rad/s^2"), ("temperature", "C")],
)
def test_getter(attribute, expected):

    """
    Tests the UnitsDefinition getter method\n
    Asserts the class variable of the UnitsDefinition object is equal to the expected value.
    """
    default_units = units.DEFAULT_UNITS
    assert default_units[attribute] == expected


# Testing the UnitsDefinition setter method for errors
@pytest.mark.parametrize(
    ("attribute", "unit", "expected_output", "expected_error"),
    [
        ("forc", "N", "'Invalid key: forc'", KeyError),
        ("force", "n", "Invalid unit: n", ValueError),
        ("", "N", "'Invalid key: '", KeyError),
        ("force", "", "Invalid unit: ", ValueError),
    ],
)
def test_setter_error(attribute, unit, expected_output, expected_error):

    """
    Tests the UnitsDefinition setter method for errors\n
    Asserts the error message and error type are equal to the expected values.
    """

    default_units = units.DEFAULT_UNITS

    with pytest.raises(Exception) as e:
        default_units[attribute] = unit
    assert str(e.value) == expected_output
    assert type(e.value) == expected_error


@pytest.mark.parametrize(
    ("attribute", "expected_output", "expected_error"),
    [("tork", "'Invalid key: tork'", KeyError), ("", "'Invalid key: '", KeyError)],
)
def test_getter_error(attribute, expected_output, expected_error):

    """
    Tests the UnitsDefinition getter method for errors\n
    Asserts the error message and error type are equal to the expected values.
    """

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
        (2, "stiffness", "N-m/rad", 2),
        (2, "damping", "N-m/(rad/s)", 2),
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
        (20, "torque", "kgf-cm", 20 * 0.0980665),
        (-30, "torque", "kgf-cm", -30 * 0.0980665),
        (20, "stiffness", "N-m/deg", 20 * 0.017453292519943295),
        (-30, "stiffness", "N-m/deg", -30 * 0.017453292519943295),
        (20, "damping", "N-m/(deg/s)", 20 * 0.017453292519943295),
        (-30, "damping", "N-m/(deg/s)", -30 * 0.017453292519943295),
        (20, "length", "cm", 20 * 0.01),
        (-30, "length", "cm", -30 * 0.01),
        (20, "length", "in", 20 * 0.0254),
        (-30, "length", "in", -30 * 0.0254),
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
        (20, "current", "A", 20 * 1000),
        (-30, "current", "A", -30 * 1000),
        (20, "voltage", "V", 20 * 1000),
        (-30, "voltage", "V", -30 * 1000),
        (20, "gravity", "g", 20 * 9.80665),
        (-30, "gravity", "g", -30 * 9.80665),
        (20, "temperature", "F", (20 - 32) * 0.55556),
        (-30, "temperature", "F", (-30 - 32) * 0.55556),
        (20, "temperature", "K", 20 - 273.15),
        (-30, "temperature", "K", -30 - 273.15),
    ],
)
def test_convert_to_default_units(value, attribute, unit, expected):

    """
    Tests the UnitsDefinition convert_to_default_units method\n
    Asserts the returned value is equal to the expected value.
    """
    default_units = units.DEFAULT_UNITS
    default_units[attribute] = unit
    assert default_units.convert_to_default_units(value, attribute) == expected


@pytest.mark.parametrize(
    ("value", "attribute", "unit", "expected"),
    [
        (2, "force", "N", 2),
        (2, "torque", "N-m", 2),
        (2, "stiffness", "N-m/rad", 2),
        (2, "damping", "N-m/(rad/s)", 2),
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
        (20, "torque", "kgf-cm", 20 / 0.0980665),
        (-30, "torque", "kgf-cm", -30 / 0.0980665),
        (20, "stiffness", "N-m/deg", 20 / 0.017453292519943295),
        (-30, "stiffness", "N-m/deg", -30 / 0.017453292519943295),
        (20, "damping", "N-m/(deg/s)", 20 / 0.017453292519943295),
        (-30, "damping", "N-m/(deg/s)", -30 / 0.017453292519943295),
        (20, "length", "cm", 20 / 0.01),
        (-30, "length", "cm", -30 / 0.01),
        (20, "length", "in", 20 / 0.0254),
        (-30, "length", "in", -30 / 0.0254),
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
        (-30, "acceleration", "deg/s^2", -30 / 0.017453292519943295),
        (20, "time", "ms", 20 / 0.001),
        (-30, "time", "ms", -30 / 0.001),
        (20, "current", "A", 20 / 1000),
        (-30, "current", "A", -30 / 1000),
        (20, "voltage", "V", 20 / 1000),
        (-30, "voltage", "V", -30 / 1000),
        (20, "gravity", "g", 20 / 9.80665),
        (-30, "gravity", "g", -30 / 9.80665),
        (20, "temperature", "F", 20 / 0.55556 + 32),
        (-30, "temperature", "F", -30 / 0.55556 + 32),
        (20, "temperature", "K", 20 + 273.15),
        (-30, "temperature", "K", -30 + 273.15),
    ],
)
def test_convert_from_default_units(value, attribute, unit, expected):

    """
    Tests the UnitsDefinition convert_from_default_units method\n
    Asserts the returned value is equal to the expected value.
    """

    default_units = units.DEFAULT_UNITS
    default_units[attribute] = unit
    assert default_units.convert_from_default_units(value, attribute) == expected
