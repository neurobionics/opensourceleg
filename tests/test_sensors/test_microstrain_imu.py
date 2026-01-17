import pytest

from opensourceleg.logging import LOGGER
from opensourceleg.sensors.base import SensorNotStreamingException
from opensourceleg.sensors.imu import LordMicrostrainIMU


# Patched classes from mscl module
class MockMipChannel:
    def __init__(self, miptype: str, frequency: float):
        self.miptype = miptype
        self.frequency = frequency


MockTypes = [
    "CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_EULER",
    "CH_FIELD_ESTFILTER_ESTIMATED_ANGULAR_RATE",
    "CH_FIELD_ESTFILTER_ESTIMATED_LINEAR_ACCEL",
    "CH_FIELD_ESTFILTER_GPS_TIMESTAMP",
]


class MockConnection:
    def __init__(self, port: str, baud_rate: float):
        self.port = port
        self.baud_rate = baud_rate


class MockResponse:
    def __init__(self, ping_var):
        self.ping_var = ping_var

    def success(self):
        return self.ping_var


class MockData:
    def __init__(self, val: int):
        self.channel = "mockdata"
        self.val = val

    def channelName(self):
        return self.channel

    def as_float(self):
        return self.val


class MockDataPockets:
    def __init__(self, packetnum: int):
        self.packetnum = packetnum

    def data(self):
        return [MockData(10), MockData(20)]


class MockNode:
    def __init__(self, connection: MockConnection):
        self.connection = connection
        self.idle = False
        self.type = None
        self.configure = None
        self.datastream = None
        self.ping_var = False

    def setActiveChannelFields(self, mocktype: str, configure):
        self.type = mocktype
        self.configure = configure

    def enableDataStream(self, mocktype: str):
        self.datastream = mocktype

    def setToIdle(self):
        self.idle = True

    def ping(self):
        return MockResponse(self.ping_var)

    def getDataPackets(self, timeout: int, maxPackets: int):
        self.timeout = timeout
        return [MockDataPockets(f"{i}") for i in range(maxPackets)]


# Creating a Mock LordMicrostrainIMU Class with patched functions for mscl
class MockLordMicrostrainIMU(LordMicrostrainIMU):
    def __init__(
        self,
        tag: str = "MockIMULordMicrostrain",
        port: str = r"/dev/ttyUSB0",
        baud_rate: int = 921600,
        frequency: int = 200,
        update_timeout: int = 500,
        max_packets: int = 1,
        return_packets: bool = False,
        offline: bool = False,
    ):
        LOGGER.info("Initializing MockLordMicrostrainIMU")
        self._init_variables(tag, port, baud_rate, frequency, update_timeout, max_packets, return_packets, offline)

    def _configure_mip_channels(self):
        return [MockMipChannel(f"{i}", 200) for i in MockTypes]

    def start(self):
        self._connection = MockConnection("port", 200)
        self._node = MockNode(self._connection)
        self._node.setActiveChannelFields("mocktype", self._configure_mip_channels)
        self._node.enableDataStream("mocktype")
        self._is_streaming = True

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


@pytest.fixture
def sample_imu_init():
    sample_imu = MockLordMicrostrainIMU()

    sample_imu._data = {
        "estRoll": [0],
        "estPitch": [0],
        "estYaw": [0],
        "estAngularRateX": [0],
        "estAngularRateY": [0],
        "estAngularRateZ": [0],
        "estLinearAccelX": [0],
        "estLinearAccelY": [0],
        "estLinearAccelZ": [0],
        "estFilterGpsTimeTow": [0],
    }
    return sample_imu


# Test init
def test_init_default(sample_imu: MockLordMicrostrainIMU):
    assert all([
        sample_imu._port == "/dev/ttyUSB0",
        sample_imu._baud_rate == 921600,
        sample_imu._frequency == 200,
        sample_imu._is_streaming is False,
        sample_imu._connection is None,
        sample_imu._data == {},
        isinstance(sample_imu._data, dict),
    ])


def test_init_set():
    port_name = "test"
    baud = 10
    freq = 20
    sample_imu = MockLordMicrostrainIMU(port=port_name, baud_rate=baud, frequency=freq)
    assert all([
        sample_imu._port == port_name,
        sample_imu._baud_rate == baud,
        sample_imu._frequency == freq,
    ])


# Test configure mip channels
def test_configure_mip_channels(sample_imu: MockLordMicrostrainIMU):
    channels = sample_imu._configure_mip_channels()
    assert len(channels) == 4
    for i in range(0, 4):
        assert all([channels[i].miptype in MockTypes, isinstance(channels[i], MockMipChannel)])


# Test start
def test_start(sample_imu: MockLordMicrostrainIMU):
    assert all([
        sample_imu._connection is None,
        not hasattr(sample_imu, "_node"),
        sample_imu._is_streaming is False,
    ])

    sample_imu.start()

    assert all([
        isinstance(sample_imu._connection, MockConnection),
        isinstance(sample_imu._node, MockNode),
        sample_imu._is_streaming is True,
        sample_imu._node.type == "mocktype",
        sample_imu._node.datastream == "mocktype",
    ])


# Test stop
def test_stop_not_started(sample_imu: MockLordMicrostrainIMU):
    with pytest.raises(SensorNotStreamingException) as e:
        sample_imu.stop()
    assert str(e.value) == (
        f"{sample_imu.__repr__()} is not streaming, please ensure that the connections "
        "are intact, power is on, and the start method is called."
    )


def test_stop(sample_imu: MockLordMicrostrainIMU):
    sample_imu.start()
    assert sample_imu._is_streaming is True

    sample_imu.stop()

    assert all([sample_imu._is_streaming is False, sample_imu._node.idle is True])


# Test ping
def test_ping_not_started(sample_imu: MockLordMicrostrainIMU):
    with pytest.raises(SensorNotStreamingException) as e:
        sample_imu.ping()
    assert str(e.value) == (
        f"{sample_imu.__repr__()} is not streaming, please ensure that the connections "
        "are intact, power is on, and the start method is called."
    )


def test_ping_not_success(sample_imu: MockLordMicrostrainIMU):
    sample_imu.start()

    file = open(LOGGER._file_path)
    contents = file.read()
    assert (f"ERROR: Failed to ping the IMU at {sample_imu.port}") not in contents
    file.close()

    sample_imu._node.ping_var = False
    sample_imu.ping()
    file = open(LOGGER._file_path)
    contents = file.read()
    assert (f"ERROR: Failed to ping the IMU at {sample_imu.port}") in contents
    file.close()


def test_ping_success(sample_imu: MockLordMicrostrainIMU):
    sample_imu.start()

    file = open(LOGGER._file_path)
    contents = file.read()
    assert (f"INFO: Successfully pinged the IMU at {sample_imu.port}") not in contents
    file.close()

    sample_imu._node.ping_var = True
    sample_imu.ping()
    file = open(LOGGER._file_path)
    contents = file.read()
    assert (f"INFO: Successfully pinged the IMU at {sample_imu.port}") in contents
    file.close()


# Test update
def test_update(sample_imu: MockLordMicrostrainIMU):
    sample_imu.set_update_timeout(400)
    sample_imu.set_max_packets(3)
    sample_imu.set_return_packets(True)

    sample_imu.start()
    assert sample_imu._data == {}
    sample_imu.update()
    assert sample_imu._data == {"mockdata": 20}

    return_val = sample_imu.update()
    assert len(return_val) == 3
    for i in range(0, 3):
        assert isinstance(return_val[i], MockDataPockets)


# Test LordMicrostrainIMU repr
def test_lord_microstrain_imu_repr(sample_imu: MockLordMicrostrainIMU):
    assert sample_imu.__repr__() == "MockIMULordMicrostrain[MockLordMicrostrainIMU]"


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
    assert sample_imu.is_streaming is True


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
