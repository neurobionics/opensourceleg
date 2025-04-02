from unittest.mock import Mock

import pytest

from opensourceleg.sensors.base import (
    EncoderBase,
    IMUBase,
    LoadcellBase,
    SensorBase,
    SensorNotStreamingException,
    check_sensor_stream,
)


# Test SensorNotStreamingException init
def test_actuator_stream_exception():
    test_str = "test"
    with pytest.raises(SensorNotStreamingException) as e:
        raise SensorNotStreamingException(test_str)
    assert str(e.value) == (
        f"{test_str} is not streaming, please ensure that the connections are "
        "intact, power is on, and the start method is called."
    )

    with pytest.raises(SensorNotStreamingException) as e:
        raise SensorNotStreamingException()
    assert str(e.value) == (
        "Sensor is not streaming, please ensure that the connections are intact, "
        "power is on, and the start method is called."
    )


# Creating a Mock Sensor Base Class
class MockSensor(SensorBase):
    def __init__(self, tag: str = "MockSensor", **kwargs):
        self._is_streaming = False
        super().__init__(tag=tag, **kwargs)

    def data(self):
        pass

    def start(self):
        pass

    def stop(self):
        pass

    def update(self):
        pass

    @property
    def is_streaming(self):
        return self._is_streaming

    @check_sensor_stream
    def test_sensor_stream(self):
        return "test passed"

    def set_is_streaming(self):
        self._is_streaming = True


@pytest.fixture
def mock_sensor():
    return MockSensor()


# Test check_sensor_stream
def test_check_sensor_stream(mock_sensor: MockSensor):
    with pytest.raises(SensorNotStreamingException) as e:
        mock_sensor.test_sensor_stream()
    assert str(e.value) == (
        f"{mock_sensor.__repr__()} is not streaming, please ensure that the "
        "connections are intact, power is on, and the start method is called."
    )

    mock_sensor.set_is_streaming()
    assert mock_sensor.test_sensor_stream() == "test passed"


# Test SensorBase repr
def test_sensor_base_repr(mock_sensor: MockSensor):
    assert mock_sensor.__repr__() == "MockSensor[MockSensor]"


# Test SensorBase enter
def test_sensor_base_enter(mock_sensor: MockSensor):
    mock_sensor.start = Mock()
    mock_sensor.start.assert_not_called()
    assert mock_sensor.__enter__() == mock_sensor
    mock_sensor.start.assert_called_once()


# Test SensorBase exit
def test_sensor_base_exit(mock_sensor: MockSensor):
    mock_sensor.stop = Mock()
    mock_sensor.stop.assert_not_called()
    filler = "filler"
    mock_sensor.__exit__(filler, filler, filler)
    mock_sensor.stop.assert_called_once()


# Creating a Mock Encoder Base Class
class MockEncoder(EncoderBase, MockSensor):
    def __init__(self, tag: str):
        super().__init__(tag=tag)

    @property
    def position(self):
        pass

    @property
    def velocity(self):
        pass


@pytest.fixture
def mock_encoder():
    return MockEncoder(tag="MockEncoder")


# Test EncoderBase repr
def test_encoder_base_repr(mock_encoder: MockEncoder):
    assert mock_encoder.__repr__() == "EncoderBase"


# Creating a Mock Loadcell Class
class MockLoadcell(LoadcellBase, MockSensor):
    def __init__(self, tag: str):
        super().__init__(tag=tag)

    def calibrate(self):
        pass

    def reset(self):
        pass

    @property
    def fx(self):
        pass

    @property
    def fy(self):
        pass

    @property
    def fz(self):
        pass

    @property
    def mx(self):
        pass

    @property
    def my(self):
        pass

    @property
    def mz(self):
        pass

    @property
    def is_calibrated(self):
        pass


@pytest.fixture
def mock_loadcell():
    return MockLoadcell(tag="MockLoadcell")


# Test LoadcellBase repr
def test_loadcell_base_repr(mock_loadcell: MockLoadcell):
    assert mock_loadcell.__repr__() == "LoadcellBase"


# Creating a Mock IMUBase Class
class MockIMU(IMUBase, MockSensor):
    @property
    def acc_x(self):
        pass

    @property
    def acc_y(self):
        pass

    @property
    def acc_z(self):
        pass

    @property
    def gyro_x(self):
        pass

    @property
    def gyro_y(self):
        pass

    @property
    def gyro_z(self):
        pass


@pytest.fixture
def mock_imu():
    return MockIMU(tag="MockIMU")


# Test IMUBase repr
def test_imu_base_repr(mock_imu: MockIMU):
    assert mock_imu.__repr__() == "MockIMU[MockIMU]"
