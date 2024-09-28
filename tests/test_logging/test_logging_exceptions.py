import pytest

from opensourceleg.logging.exceptions import *

#Test ActuatorStreamException
def test_actuator_stream_exception():
    with pytest.raises(ActuatorStreamException) as e:
        raise ActuatorStreamException("test")
    assert str(e.value) == "test is not streaming, please call start() method before sending commands"


#Test ActuatorConnectionException
def test_actuator_connection_exception():
    with pytest.raises(ActuatorConnectionException) as e:
        raise ActuatorConnectionException("test")
    assert str(e.value) == "test is not connected"


#Test ActuatorIsNoneException
def test_actuator_is_none_exception():
    with pytest.raises(ActuatorIsNoneException) as e:
        raise ActuatorIsNoneException("test")
    assert str(e.value) == "Actuator is None in test mode, please pass the actuator instance to the mode during initialization or set the actuator instance using set_actuator method."


#Test ControlModeException
def test_control_mode_exception():
    with pytest.raises(ControlModeException) as e:
        raise ControlModeException("test", "att", "new")
    assert str(e.value) == "[test] Cannot set att in new mode. Please set the actuator to att mode first."

#Test VoltageModeMissingException
def test_voltage_mode_missing_exception():
    with pytest.raises(VoltageModeMissingException) as e:
        raise VoltageModeMissingException("test")
    assert str(e.value) == "test must have a voltage mode"


#Test ActuatorKeyException
def test_actuator_key_exception():
    with pytest.raises(ActuatorKeyException) as e:
        raise ActuatorKeyException("test", "test2")
    assert str(e.value) == "test does not have test2 key in the actuators dictionary. Please check the actuators dictionary for the `test2` key."