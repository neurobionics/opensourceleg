import pytest

from opensourceleg.actuators.base import MotorConstants
from opensourceleg.actuators.dephy import DephyActuator


class MockDephyActuator(DephyActuator):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, offline=True, **kwargs)


def test_motor_constants_setter_type_check():
    actuator = MockDephyActuator()
    # Should work with correct type
    actuator.MOTOR_CONSTANTS = actuator.MOTOR_CONSTANTS
    # Should fail with incorrect type
    with pytest.raises(TypeError):
        actuator.MOTOR_CONSTANTS = 123  # Not a MOTOR_CONSTANTS instance
    with pytest.raises(TypeError):
        actuator.MOTOR_CONSTANTS = None  # Not a MOTOR_CONSTANTS instance
    with pytest.raises(TypeError):
        actuator.MOTOR_CONSTANTS = {"foo": "bar"}  # Not a MOTOR_CONSTANTS instance

    class FakeMOTOR_CONSTANTS:
        def __init__(self):
            self.MOTOR_COUNT_PER_REV = 2048
            self.NM_PER_AMP = 0.02
            self.MAX_CASE_TEMPERATURE = 80.0
            self.MAX_WINDING_TEMPERATURE = 120.0

    fake_constants = FakeMOTOR_CONSTANTS()
    with pytest.raises(TypeError):
        actuator.MOTOR_CONSTANTS = fake_constants  # Looks like MOTOR_CONSTANTS, but isn't


def test_derived_constants_update_on_motor_constants_change():
    actuator = MockDephyActuator()
    original_nm_per_rad = actuator.NM_PER_RAD_TO_MOTOR_UNITS
    original_nm_s_per_rad = actuator.NM_S_PER_RAD_TO_MOTOR_UNITS
    # Create a new MotorConstants with a different NM_PER_AMP
    new_constants = MotorConstants(
        MOTOR_COUNT_PER_REV=actuator.MOTOR_CONSTANTS.MOTOR_COUNT_PER_REV,
        NM_PER_AMP=actuator.MOTOR_CONSTANTS.NM_PER_AMP * 2,  # Change value
        MAX_CASE_TEMPERATURE=actuator.MOTOR_CONSTANTS.MAX_CASE_TEMPERATURE,
        MAX_WINDING_TEMPERATURE=actuator.MOTOR_CONSTANTS.MAX_WINDING_TEMPERATURE,
    )
    actuator.MOTOR_CONSTANTS = new_constants
    # The derived constants should update accordingly
    assert original_nm_per_rad != actuator.NM_PER_RAD_TO_MOTOR_UNITS
    assert original_nm_s_per_rad != actuator.NM_S_PER_RAD_TO_MOTOR_UNITS
    # They should update in the expected direction (since NM_PER_AMP is in denominator)
    assert original_nm_per_rad / 2.0 == actuator.NM_PER_RAD_TO_MOTOR_UNITS
    assert original_nm_s_per_rad / 2.0 == actuator.NM_S_PER_RAD_TO_MOTOR_UNITS


def test_derived_constants_update_on_motor_constants_attribute_change():
    actuator = MockDephyActuator()
    original_nm_s_per_rad = actuator.NM_S_PER_RAD_TO_MOTOR_UNITS
    # Modify individual attribute instead of replacing the whole object
    actuator.MOTOR_CONSTANTS.NM_PER_AMP = actuator.MOTOR_CONSTANTS.NM_PER_AMP * 2
    # The derived constants should update accordingly
    assert original_nm_s_per_rad != actuator.NM_S_PER_RAD_TO_MOTOR_UNITS
    # They should update in the expected direction (since NM_PER_AMP is in denominator)
    assert original_nm_s_per_rad / 2.0 == actuator.NM_S_PER_RAD_TO_MOTOR_UNITS
    # Do it again to ensure multiple changes work
    original_nm_s_per_rad = actuator.NM_S_PER_RAD_TO_MOTOR_UNITS
    actuator.MOTOR_CONSTANTS.NM_PER_AMP = actuator.MOTOR_CONSTANTS.NM_PER_AMP * 2
    assert original_nm_s_per_rad != actuator.NM_S_PER_RAD_TO_MOTOR_UNITS
    assert original_nm_s_per_rad / 2.0 == actuator.NM_S_PER_RAD_TO_MOTOR_UNITS
