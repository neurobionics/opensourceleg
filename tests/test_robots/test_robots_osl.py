import pytest

from unittest.mock import Mock
from tests.test_actuators.test_actuators_base import MockActuator, MOTOR_CONSTANTS
from tests.test_sensors.test_sensors_base import MockSensor, MockLoadcell, MockEncoder
from opensourceleg.robots.osl import OpenSourceLeg
from opensourceleg.logging import LOGGER


@pytest.fixture
def sample_osl():
    tag_name = "test"
    test_tactuator = MockActuator("act_tag", 10, MOTOR_CONSTANTS(
            MOTOR_COUNT_PER_REV=1000,
            NM_PER_AMP=0.1,
            NM_PER_RAD_TO_K=1.0,
            NM_S_PER_RAD_TO_B=0.1,
            MAX_CASE_TEMPERATURE=100.0,
            MAX_WINDING_TEMPERATURE=150.0,
        ))
    test_actuators = {"actuator1": test_tactuator}
    test_tsensor = MockSensor()
    test_sensors = {"sensor1": test_tsensor}

    return OpenSourceLeg(tag=tag_name,actuators=test_actuators,sensors=test_sensors)


# Test home
def test_osl_home(sample_osl: OpenSourceLeg):
    # Only one actuator in mock_robot.actuators
    MockActuator.home = Mock()
    MockActuator.home.assert_not_called()
    sample_osl.home()
    MockActuator.home.assert_called_once()


# Test knee
def test_knee(sample_osl: OpenSourceLeg):
    # Test error case
    expected_error = "ERROR: Knee actuator not found. Please check for `knee` key in the actuators dictionary.\n" 
    
    with open(LOGGER.file_path) as f:
       output = f.read()
    assert expected_error not in output
    
    with pytest.raises(SystemExit) as e:  
        sample_osl.knee()
    assert e.type == SystemExit

    with open(LOGGER.file_path) as f:
       output = f.read()
    assert expected_error in output

    # Test non-error case
    test_knee_actuator = MockActuator("knee", 10, MOTOR_CONSTANTS(
            MOTOR_COUNT_PER_REV=1000,
            NM_PER_AMP=0.1,
            NM_PER_RAD_TO_K=1.0,
            NM_S_PER_RAD_TO_B=0.1,
            MAX_CASE_TEMPERATURE=100.0,
            MAX_WINDING_TEMPERATURE=150.0,
        ))
    test_actuators = {"knee": test_knee_actuator}
    sample_osl.actuators = test_actuators
    
    assert sample_osl.knee == test_knee_actuator
    

# Test ankle
def test_ankle(sample_osl: OpenSourceLeg): 
    # Test error case
    expected_error = "ERROR: Ankle actuator not found. Please check for `ankle` key in the actuators dictionary.\n" 

    with open(LOGGER.file_path) as f:
       output = f.read()
    assert expected_error not in output
    
    with pytest.raises(SystemExit) as e:  
        sample_osl.ankle()
    assert e.type == SystemExit

    with open(LOGGER.file_path) as f:
       output = f.read()
    assert expected_error in output

    # Test non-error case
    test_ankle_actuator = MockActuator("ankle", 10, MOTOR_CONSTANTS(
            MOTOR_COUNT_PER_REV=1000,
            NM_PER_AMP=0.1,
            NM_PER_RAD_TO_K=1.0,
            NM_S_PER_RAD_TO_B=0.1,
            MAX_CASE_TEMPERATURE=100.0,
            MAX_WINDING_TEMPERATURE=150.0,
        ))
    test_actuators = {"ankle": test_ankle_actuator}
    sample_osl.actuators = test_actuators
    
    assert sample_osl.ankle == test_ankle_actuator


# Test loadcell
def test_loadcell(sample_osl: OpenSourceLeg):
    # Test error case
    expected_error = "ERROR: Loadcell sensor not found. Please check for `loadcell` key in the sensors dictionary.\n" 

    with open(LOGGER.file_path) as f:
       output = f.read()
    assert expected_error not in output
    
    with pytest.raises(SystemExit) as e:  
        sample_osl.loadcell()
    assert e.type == SystemExit

    with open(LOGGER.file_path) as f:
       output = f.read()
    assert expected_error in output

    # Test non-error case
    test_loadcell_sensor = MockLoadcell()
    test_sensors = {"loadcell": test_loadcell_sensor}
    sample_osl.sensors = test_sensors

    assert sample_osl.loadcell == test_loadcell_sensor


# Test joint encoder knee
def test_joint_encoder_knee(sample_osl: OpenSourceLeg):
    # Test error case
    expected_error = "ERROR: Knee joint encoder sensor not found. Please check for `joint_encoder_knee` key in the sensors dictionary.\n" 

    with open(LOGGER.file_path) as f:
       output = f.read()
    assert expected_error not in output
    
    with pytest.raises(SystemExit) as e:  
        sample_osl.joint_encoder_knee()
    assert e.type == SystemExit

    with open(LOGGER.file_path) as f:
       output = f.read()
    assert expected_error in output

    # Test non-error case
    test_joint_knee = MockEncoder()
    test_sensors = {"joint_encoder_knee": test_joint_knee}
    sample_osl.sensors = test_sensors

    assert sample_osl.joint_encoder_knee == test_joint_knee


# Test joint encoder ankle
def test_joint_encoder_ankle(sample_osl: OpenSourceLeg):
    # Test error case
    expected_error = "ERROR: Ankle joint encoder sensor not found. Please check for `joint_encoder_ankle` key in the sensors dictionary.\n" 

    with open(LOGGER.file_path) as f:
       output = f.read()
    assert expected_error not in output
    
    with pytest.raises(SystemExit) as e:  
        sample_osl.joint_encoder_ankle()
    assert e.type == SystemExit

    with open(LOGGER.file_path) as f:
       output = f.read()
    assert expected_error in output

    # Test non-error case
    test_joint_ankle = MockEncoder()
    test_sensors = {"joint_encoder_ankle": test_joint_ankle}
    sample_osl.sensors = test_sensors

    assert sample_osl.joint_encoder_ankle == test_joint_ankle