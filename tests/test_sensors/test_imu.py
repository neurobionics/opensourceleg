from unittest.mock import Mock

import pytest

from opensourceleg.sensors.base import *
from opensourceleg.logging import LOGGER


class MockTypes:
    CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_EULER = "Estimated Orientation Euler"
    CH_FIELD_ESTFILTER_ESTIMATED_ANGULAR_RATE = "Estimated Angular Rate"
    CH_FIELD_ESTFILTER_ESTIMATED_LINEAR_ACCEL = "Estimated Linear Acceleration"
    CH_FIELD_ESTFILTER_GPS_TIMESTAMP = "GPS Timestamp"


@pytest.fixture
def patch_mscl(monkeypatch):
    mock_rate = "Sample Rate"
    mock_channels = Mock()
    mock_channel = Mock()

    monkeypatch.setattr('mscl.MipChannels', lambda: mock_channels)
    monkeypatch.setattr('mscl.MipChannel', lambda *args: mock_channel)
    monkeypatch.setattr('mscl.SampleRate', mock_rate)
    monkeypatch.setattr('mscl.MipTypes', MockTypes)

    return mock_channel


from opensourceleg.sensors.imu import *

# Creating a Mock LordMicrostrainIMU Class
class MockLordMicrostrainIMU(LordMicrostrainIMU): 
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
def sample_imu():
    return MockLordMicrostrainIMU()

# Test configure mip channels
def test_configure_mip_channels(sample_imu: MockLordMicrostrainIMU, patch_mscl):
    channels = sample_imu._configure_mip_channels()
    
    assert len(channels) == 4
    
    mock_channel = patch_mscl

    mock_channel.assert_any_call(MockTypes.CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_EULER, "Sample Rate")
    mock_channel.assert_any_call(MockTypes.CH_FIELD_ESTFILTER_ESTIMATED_ANGULAR_RATE, "Sample Rate")
    mock_channel.assert_any_call(MockTypes.CH_FIELD_ESTFILTER_ESTIMATED_LINEAR_ACCEL, "Sample Rate")
    mock_channel.assert_any_call(MockTypes.CH_FIELD_ESTFILTER_GPS_TIMESTAMP, "Sample Rate")


#PLACE BACK BEFORE OTHER TESTS
@pytest.fixture
def sample_imu_init():
    sample_imu = MockLordMicrostrainIMU()

    sample_imu._data = {"estRoll": [0], "estPitch": [0], 
                        "estYaw": [0], "estAngularRateX": [0], 
                        "estAngularRateY": [0], "estAngularRateZ": [0], 
                        "estLinearAccelX": [0], "estLinearAccelY": [0], 
                        "estLinearAccelZ": [0], "estFilterGpsTimeTow": [0]}
    return sample_imu


# Test init
def test_init_default(sample_imu: MockLordMicrostrainIMU):
    assert all([sample_imu.port == "/dev/ttyUSB0", 
               sample_imu.baud_rate == 921600,
               sample_imu.frequency == 200,
               sample_imu._port == "/dev/ttyUSB0",
               sample_imu._baud_rate == 921600,
               sample_imu._frequency == 200, 
               sample_imu._is_streaming == False, 
               sample_imu._connection == None,
               sample_imu._data == {},
               type(sample_imu._data) == dict])


def test_init_set():
    port_name = "test"
    baud = 10
    freq = 20
    sample_imu = MockLordMicrostrainIMU(port=port_name,baud_rate=baud,frequency=freq)
    assert all([sample_imu.port == port_name,
                sample_imu.baud_rate == baud,
                sample_imu.frequency == freq,
                sample_imu._port == port_name,
                sample_imu._baud_rate == baud,
                sample_imu._frequency == freq])

# Test start
#def test_start(sample_imu: MockLordMicrostrainIMU):
    #fix


# Test stop
#def test_stop(sample_imu: MockLordMicrostrainIMU):
    #sample_imu.start()
    #assert sample_imu.is_streaming == True
    #sample_imu.stop()
    #assert sample_imu.is_streaming == False
    #fix


# Test ping
def test_ping(sample_imu: MockLordMicrostrainIMU):
    with pytest.raises(SensorNotStreamingException) as e:
        sample_imu.ping()
    assert (
        str(e.value)
        == f"{sample_imu.__repr__()} is not streaming, please ensure that the connections are intact, power is on, and the start method is called."
    )
    #fix


# Test update
#def test_update(sample_imu: MockLordMicrostrainIMU):
    #fix


# Test LordMicrostrainIMU repr
def test_lord_microstrain_imu_repr(sample_imu: MockLordMicrostrainIMU):
    assert sample_imu.__repr__() == "IMULordMicrostrain"


# Test port
def test_port(sample_imu: MockLordMicrostrainIMU):
    test = "test"
    sample_imu._port = test
    assert sample_imu.port == test


# Test baud rate
def test_baud_rate(sample_imu: MockLordMicrostrainIMU):
    test = 10
    sample_imu._baud_rate = test
    assert sample_imu.baud_rate == test


# Test frequency
def test_frequency(sample_imu: MockLordMicrostrainIMU):
    test = 10
    sample_imu._frequency = test
    assert sample_imu.frequency == test


# Test is_streaming
def test_is_streaming(sample_imu: MockLordMicrostrainIMU):
    sample_imu._is_streaming = True
    assert sample_imu.is_streaming == True


# Test roll
def test_roll(sample_imu_init: MockLordMicrostrainIMU):
    estRoll = 5
    sample_imu_init._data["estRoll"] = estRoll
    assert sample_imu_init.roll == estRoll


# Test pitch
def test_pitch(sample_imu_init: MockLordMicrostrainIMU):
    estPitch = 10
    sample_imu_init._data["estPitch"] = estPitch
    assert sample_imu_init.pitch == estPitch


# Test yaw
def test_yaw(sample_imu_init: MockLordMicrostrainIMU):
    estYaw = 15
    sample_imu_init._data["estYaw"] = estYaw
    assert sample_imu_init.yaw == estYaw


# Test vel_x
def test_vel_x(sample_imu_init: MockLordMicrostrainIMU):
    estVelx = 20
    sample_imu_init._data["estAngularRateX"] = estVelx
    assert sample_imu_init.vel_x == estVelx


# Test vel_y
def test_vel_y(sample_imu_init: MockLordMicrostrainIMU):
    estVely = 25
    sample_imu_init._data["estAngularRateY"] = estVely
    assert sample_imu_init.vel_y == estVely


# Test vel_z
def test_vel_z(sample_imu_init: MockLordMicrostrainIMU):
    estVelz = 30
    sample_imu_init._data["estAngularRateZ"] = estVelz
    assert sample_imu_init.vel_z == estVelz


# Test acc_x
def test_acc_x(sample_imu_init: MockLordMicrostrainIMU):
    estAccx = 35
    sample_imu_init._data["estLinearAccelX"] = estAccx
    assert sample_imu_init.acc_x == estAccx


# Test acc_y
def test_acc_y(sample_imu_init: MockLordMicrostrainIMU):
    estAccy = 40
    sample_imu_init._data["estLinearAccelY"] = estAccy
    assert sample_imu_init.acc_y == estAccy


# Test acc_z
def test_acc_z(sample_imu_init: MockLordMicrostrainIMU):
    estAccz = 45
    sample_imu_init._data["estLinearAccelZ"] = estAccz
    assert sample_imu_init.acc_z == estAccz


# Test timestamp
def test_timestamp(sample_imu_init: MockLordMicrostrainIMU):
    estTime = 50
    sample_imu_init._data["estFilterGpsTimeTow"] = estTime
    assert sample_imu_init.timestamp == estTime