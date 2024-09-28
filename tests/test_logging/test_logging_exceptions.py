import pytest

from opensourceleg.logging.exceptions import *

#Test ActuatorStreamException
def test_actuator_stream_exception():
    test_str = "test"
    with pytest.raises(ActuatorStreamException) as e:
        raise ActuatorStreamException(test_str)
    assert str(e.value) == f"{test_str} is not streaming, please call start() method before sending commands"


#Test ActuatorConnectionException
def test_actuator_connection_exception():
    test_str = "test"
    with pytest.raises(ActuatorConnectionException) as e:
        raise ActuatorConnectionException("test")
    assert str(e.value) == f"{test_str} is not connected"


#Test ActuatorIsNoneException
def test_actuator_is_none_exception():
    test_str = "test"
    with pytest.raises(ActuatorIsNoneException) as e:
        raise ActuatorIsNoneException(test_str)
    assert str(e.value) == f"Actuator is None in {test_str} mode, please pass the actuator instance to the mode during initialization or set the actuator instance using set_actuator method."


#Test ControlModeException
def test_control_mode_exception():
    test_str = "test"
    test_att = "att"
    test_new = "new"
    with pytest.raises(ControlModeException) as e:
        raise ControlModeException(test_str, test_att, test_new)
    assert str(e.value) == f"[{test_str}] Cannot set {test_att} in {test_new} mode. Please set the actuator to {test_att} mode first."

#Test VoltageModeMissingException
def test_voltage_mode_missing_exception():
    test_str = "test"
    with pytest.raises(VoltageModeMissingException) as e:
        raise VoltageModeMissingException(test_str)
    assert str(e.value) == f"{test_str} must have a voltage mode"


#Test ActuatorKeyException
def test_actuator_key_exception():
    test_str = "test"
    test_key = "key"
    with pytest.raises(ActuatorKeyException) as e:
        raise ActuatorKeyException(test_str, test_key)
    assert str(e.value) == f"{test_str} does not have {test_key} key in the actuators dictionary. Please check the actuators dictionary for the `{test_key}` key."