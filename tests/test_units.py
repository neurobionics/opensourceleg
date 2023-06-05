

import opensourceleg.units as units
import pytest


@pytest.mark.parametrize(
    ("value", "attribute", "expected"),
    [
        (2, "force", 2),
        (2, "torque", 2),
        (2, "stiffness", 2),
        (2, "damping", 2),
        (2, "length", 2),
        (2, "position", 2),
        (2, "mass", 2),
        (2, "velocity", 2),
        (2, "acceleration", 2),
        (2, "time", 2),
        (2, "current", 2),
        (2, "voltage", 2),
        (2, "gravity", 2),
        (2, "temperature", 2)
    ]
)

def test_convert_to_default_units_default(value, attribute, expected):
    default_units = units.DEFAULT_UNITS
    assert default_units.convert_to_default_units(value, attribute) == expected


    
@pytest.mark.parametrize(
    ("value", "attribute", "expected"),
    [
        (1, "force", 4.4482216152605),
        (20, "force", 20 * 4.4482216152605),
        (-30, "force", -30 * 4.4482216152605)
    ]
)
    
def test_convert_to_default_units_force_lbf(value, attribute, expected):
    default_units = units.DEFAULT_UNITS
    default_units["force"] = "lbf"
    
    assert default_units.convert_to_default_units(value, attribute) == expected
    
    
@pytest.mark.parametrize(
    ("value", "attribute", "expected"),
    [
        (1, "force", 9.80665),
        (20, "force", 20 * 9.80665),
        (-30, "force", -30 * 9.80665)
    ]
) 
    
def test_convert_to_default_units_force_kgf(value, attribute, expected):
    default_units = units.DEFAULT_UNITS
    default_units["force"] = "kgf"
    
    assert default_units.convert_to_default_units(value, attribute) == expected
    

@pytest.mark.parametrize(
    ("value", "attribute", "expected"),
    [
        (1, "torque", 0.1129848290276167),
        (20, "torque", 20 * 0.1129848290276167),
        (-30, "torque", -30 * 0.1129848290276167)
    ]
)

def test_convert_to_default_units_torque_lbfin(value, attribute, expected):
    default_units = units.DEFAULT_UNITS
    default_units["torque"] = "lbf-in"
    
    assert default_units.convert_to_default_units(value, attribute) == expected
    

@pytest.mark.parametrize(
    ("value", "attribute", "expected"),
    [
        (1, "torque", 1.3558179483314004),
        (20, "torque", 20 * 1.3558179483314004),
        (-30, "torque", -30 * 1.3558179483314004)
    ]
)

def test_convert_to_default_units_torque_lbfft(value, attribute, expected):
    default_units = units.DEFAULT_UNITS
    default_units["torque"] = "lbf-ft"
    
    assert default_units.convert_to_default_units(value, attribute) == expected
    

@pytest.mark.parametrize(
    ("value", "attribute", "expected"),
    [
        (1, "torque", 0.0980665),
        (20, "torque", 20 * 0.0980665),
        (-30, "torque", -30 * 0.0980665)
    ]
)

def test_convert_to_default_units_torque_kgfcm(value, attribute, expected):
    default_units = units.DEFAULT_UNITS
    default_units["torque"] = "kgf-cm"
    
    assert default_units.convert_to_default_units(value, attribute) == expected

@pytest.mark.parametrize(
    ("value", "attribute", "expected"),
    [
        (1, "torque", 0.980665),
        (20, "torque", 20 * 0.980665),
        (-30, "torque", -30 * 0.980665)
    ]
)

def test_convert_to_default_units_torque_kgfm(value, attribute, expected):
    default_units = units.DEFAULT_UNITS
    default_units["torque"] = "kgf-m"
    
    assert default_units.convert_to_default_units(value, attribute) == expected
    

@pytest.mark.parametrize(
    ("value", "attribute", "expected"),
    [
        (1, "stiffness", 0.017453292519943295),
        (20, "stiffness", 20 * 0.017453292519943295),
        (-30, "stiffness", -30 * 0.017453292519943295)
    ]
)

def test_convert_to_default_units_stiffness_Ndeg(value, attribute, expected):
    default_units = units.DEFAULT_UNITS
    default_units["stiffness"] = "N/deg"
    
    assert default_units.convert_to_default_units(value, attribute) == expected
    

@pytest.mark.parametrize(
    ("value", "attribute", "expected"),
    [
        (1, "stiffness", 0.224809),
        (20, "stiffness", 20 * 0.224809),
        (-30, "stiffness", -30 * 0.224809)
    ]
)

def test_convert_to_default_units_stiffness_lbfrad(value, attribute, expected):
    default_units = units.DEFAULT_UNITS
    default_units["stiffness"] = "lbf/rad"
    
    assert default_units.convert_to_default_units(value, attribute) == expected
    


@pytest.mark.parametrize(
    ("value", "attribute", "expected"),
    [
        (1, "stiffness", 0.003490659),
        (20, "stiffness", 20 * 0.003490659),
        (-30, "stiffness", -30 * 0.003490659)
    ]
)

def test_convert_to_default_units_stiffness_lbfdeg(value, attribute, expected):
    default_units = units.DEFAULT_UNITS
    default_units["stiffness"] = "lbf/deg"
    
    assert default_units.convert_to_default_units(value, attribute) == expected
    

@pytest.mark.parametrize(
    ("value", "attribute", "expected"),
    [
        (1, "stiffness", 1.8518518518518519),
        (20, "stiffness", 20 * 1.8518518518518519),
        (-30, "stiffness", -30 * 1.8518518518518519)
    ]
)

def test_convert_to_default_units_stiffness_kgfrad(value, attribute, expected):
    default_units = units.DEFAULT_UNITS
    default_units["stiffness"] = "kgf/rad"
    
    assert default_units.convert_to_default_units(value, attribute) == expected
    


@pytest.mark.parametrize(
    ("value", "attribute", "expected"),
    [
        (1, "stiffness", 0.031746031746031744),
        (20, "stiffness", 20 * 0.031746031746031744),
        (-30, "stiffness", -30 * 0.031746031746031744)
    ]
)

def test_convert_to_default_units_stiffness_kgfdeg(value, attribute, expected):
    default_units = units.DEFAULT_UNITS
    default_units["stiffness"] = "kgf/deg"
    
    assert default_units.convert_to_default_units(value, attribute) == expected



@pytest.mark.parametrize(
    ("value", "attribute", "expected"),
    [
        (1, "damping", 0.017453292519943295),
        (20, "damping", 20 * 0.017453292519943295),
        (-30, "damping", -30 * 0.017453292519943295)
    ]
)

def test_convert_to_default_units_stiffness_Ndegs(value, attribute, expected):
    default_units = units.DEFAULT_UNITS
    default_units["damping"] = "N/(deg/s)"
    
    assert default_units.convert_to_default_units(value, attribute) == expected


@pytest.mark.parametrize(
    ("value", "attribute", "expected"),
    [
        (1, "damping", 0.224809),
        (20, "damping", 20 * 0.224809),
        (-30, "damping", -30 * 0.224809)
    ]
)

def test_convert_to_default_units_stiffness_lbfrads(value, attribute, expected):
    default_units = units.DEFAULT_UNITS
    default_units["damping"] = "lbf/(rad/s)"
    
    assert default_units.convert_to_default_units(value, attribute) == expected


@pytest.mark.parametrize(
    ("value", "attribute", "expected"),
    [
        (1, "damping", 1.8518518518518519),
        (20, "damping", 20 * 1.8518518518518519),
        (-30, "damping", -30 * 1.8518518518518519)
    ]
)

def test_convert_to_default_units_stiffness_kgfrads(value, attribute, expected):
    default_units = units.DEFAULT_UNITS
    default_units["damping"] = "kgf/(rad/s)"
    
    assert default_units.convert_to_default_units(value, attribute) == expected


@pytest.mark.parametrize(
    ("value", "attribute", "expected"),
    [
        (1, "damping", 0.031746031746031744),
        (20, "damping", 20 * 0.031746031746031744),
        (-30, "damping", -30 * 0.031746031746031744)
    ]
)

def test_convert_to_default_units_stiffness_kgfdegs(value, attribute, expected):
    default_units = units.DEFAULT_UNITS
    default_units["damping"] = "kgf/(deg/s)"
    
    assert default_units.convert_to_default_units(value, attribute) == expected
    
@pytest.mark.parametrize(
    ("value", "attribute", "expected"),
    [
        (1, "length", 0.01),
        (20, "length", 20 * 0.01),
        (-30, "length", -30 * 0.01)
    ]
)

def test_convert_to_default_units_length_cm(value, attribute, expected):
    default_units = units.DEFAULT_UNITS
    default_units["length"] = "cm"
    
    assert default_units.convert_to_default_units(value, attribute) == expected
    

@pytest.mark.parametrize(
    ("value", "attribute", "expected"),
    [
        (1, "length", 0.0254),
        (20, "length", 20 * 0.0254),
        (-30, "length", -30 * 0.0254)
    ]
)

def test_convert_to_default_units_length_in(value, attribute, expected):
    default_units = units.DEFAULT_UNITS
    default_units["length"] = "in"
    
    assert default_units.convert_to_default_units(value, attribute) == expected
    


@pytest.mark.parametrize(
    ("value", "attribute", "expected"),
    [
        (1, "length", 0.3048),
        (20, "length", 20 * 0.3048),
        (-30, "length", -30 * 0.3048)
    ]
)

def test_convert_to_default_units_length_ft(value, attribute, expected):
    default_units = units.DEFAULT_UNITS
    default_units["length"] = "ft"
    
    assert default_units.convert_to_default_units(value, attribute) == expected
    


@pytest.mark.parametrize(
    ("value", "attribute", "expected"),
    [
        (1, "position", 0.017453292519943295),
        (20, "position", 20 * 0.017453292519943295),
        (-30, "position", -30 * 0.017453292519943295)
    ]
)

def test_convert_to_default_units_position_deg(value, attribute, expected):
    default_units = units.DEFAULT_UNITS
    default_units["position"] = "deg"
    
    assert default_units.convert_to_default_units(value, attribute) == expected
    




