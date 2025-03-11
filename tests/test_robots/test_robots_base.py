import os
from unittest.mock import Mock

import pytest

from opensourceleg.logging import Logger
from opensourceleg.robots.base import RobotBase
from tests.test_actuators.test_actuators_base import MOTOR_CONSTANTS, MockActuator
from tests.test_sensors.test_sensors_base import MockSensor

CURR_DIR = os.path.dirname(os.path.realpath(__file__))


# Creating a Mock Robot Base Class
class MockRobot(RobotBase):
    def start(self):
        super().start()

    def stop(self):
        super().stop()

    def update(self):
        super().update()


@pytest.fixture
def mock_robot():
    tag_name = "test"
    test_tactuator = MockActuator(
        "MockActuator",
        10,
        MOTOR_CONSTANTS(
            MOTOR_COUNT_PER_REV=1000,
            NM_PER_AMP=0.1,
            NM_PER_RAD_TO_K=1.0,
            NM_S_PER_RAD_TO_B=0.1,
            MAX_CASE_TEMPERATURE=100.0,
            MAX_WINDING_TEMPERATURE=150.0,
        ),
    )
    test_actuators = {"actuator1": test_tactuator}
    test_tsensor = MockSensor()
    test_sensors = {"sensor1": test_tsensor}
    return MockRobot(tag=tag_name, actuators=test_actuators, sensors=test_sensors)


@pytest.fixture(scope="function")
def test_logger():
    log = Logger(
        log_path=CURR_DIR,
        file_name="test_robots_base",
    )
    log.reset()
    yield log


# Test init
def test_robot_base_init():
    tag_name = "test"
    test_tactuator = MockActuator(
        "test",
        10,
        MOTOR_CONSTANTS(
            MOTOR_COUNT_PER_REV=1000,
            NM_PER_AMP=0.1,
            NM_PER_RAD_TO_K=1.0,
            NM_S_PER_RAD_TO_B=0.1,
            MAX_CASE_TEMPERATURE=100.0,
            MAX_WINDING_TEMPERATURE=150.0,
        ),
    )
    test_actuators = {"actuator": test_tactuator}
    test_tsensor = MockSensor()
    test_sensors = {"sensor": test_tsensor}
    sample_robot = MockRobot(tag=tag_name, actuators=test_actuators, sensors=test_sensors)
    assert all([
        sample_robot.tag == tag_name,
        sample_robot.actuators == test_actuators,
        sample_robot.sensors == test_sensors,
        type(sample_robot.actuators) is dict,
        type(sample_robot.sensors) is dict,
    ])


# Test enter
def test_robot_base_enter(mock_robot: MockRobot):
    mock_robot.start = Mock()
    mock_robot.start.assert_not_called()
    assert mock_robot.__enter__() == mock_robot
    mock_robot.start.assert_called_once()


# Test exit
def test_robot_base_exit(mock_robot: MockRobot):
    mock_robot.stop = Mock()
    mock_robot.stop.assert_not_called()
    filler = "filler"
    mock_robot.__exit__(filler, filler, filler)
    mock_robot.stop.assert_called_once()


# Test start
def test_robot_base_start(mock_robot: MockRobot, test_logger: Logger):
    # "pass" is implementation for start function in both Actuator and Sensor base classes
    # so just testing if called for that part of function
    MockActuator.start = Mock()
    MockSensor.start = Mock()

    MockActuator.start.assert_not_called()
    MockSensor.start.assert_not_called()

    mock_robot.start()

    file = open(test_logger.file_path)
    contents = file.read()
    assert ("DEBUG: Calling start method of MockActuator") in contents
    assert "DEBUG: Calling start method of SensorBase" in contents
    file.close()

    MockActuator.start.assert_called_once()
    MockSensor.start.assert_called_once()


# Test stop
def test_robot_base_stop(mock_robot: MockRobot, test_logger: Logger):
    # "pass" is implementation for stop function in both Actuator and Sensor base classes
    # so just testing if called for that part of function
    MockActuator.stop = Mock()
    MockSensor.stop = Mock()

    MockActuator.stop.assert_not_called()
    MockSensor.stop.assert_not_called()

    mock_robot.stop()

    file = open(test_logger.file_path)
    contents = file.read()
    assert "DEBUG: Calling stop method of MockActuator" in contents
    assert "DEBUG: Calling stop method of SensorBase" in contents
    file.close()

    MockActuator.stop.assert_called_once()
    MockSensor.stop.assert_called_once()


# Test update
def test_robot_base_update(mock_robot: MockRobot):
    # "pass" is implementation for update function in both Actuator and Sensor base classes so just testing if called
    MockActuator.update = Mock()
    MockSensor.update = Mock()

    MockActuator.update.assert_not_called()
    MockSensor.update.assert_not_called()

    mock_robot.update()

    MockActuator.update.assert_called_once()
    MockSensor.update.assert_called_once()


# Test tag
def test_robot_tag(mock_robot: MockRobot):
    mock_robot._tag = "tag_change"
    assert mock_robot.tag == "tag_change"
