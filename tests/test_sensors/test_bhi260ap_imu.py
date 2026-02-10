import numpy as np
import pytest

from opensourceleg.logging import LOGGER
from opensourceleg.sensors.imu import BHI260AP


# Patched classes from spidev module
class MockSPI:
    def __init__(self):
        self.bus = None
        self.cs = None
        self.max_speed_hz = None
        self.mode = None
        self.bits_per_word = None
        self._is_open = False
        self._data = {}

    def open(self, bus: int, cs: int):
        self.bus = bus
        self.cs = cs
        self._is_open = True

    def close(self):
        self._is_open = False

    def xfer2(self, tx_data: list) -> list:
        """Simulate SPI transfer"""
        if not self._is_open:
            raise RuntimeError("SPI device not open")

        # Return dummy data for register reads
        # For write operations (MSB=0), return zeros
        # For read operations (MSB=1), return mock register values
        if tx_data[0] & 0x80:  # Read operation
            reg_addr = tx_data[0] & 0x7F
            length = len(tx_data) - 1

            # Mock register values
            mock_registers = {
                BHI260AP.REG_CHIP_ID: [0x70],  # Valid chip ID
                BHI260AP.REG_BOOT_STATUS: [0x10],  # Host interface ready
                BHI260AP.REG_HOST_STATUS: [0x00],  # Active (not sleeping)
                BHI260AP.REG_INT_STATUS: [0x00],
                BHI260AP.REG_KERNEL_VERSION: [0x01, 0x00],  # Non-zero kernel version
                BHI260AP.REG_ROM_VERSION: [0x2E, 0x14],
                BHI260AP.REG_FUSER2_PRODUCT_ID: [0x89],
            }

            if reg_addr in mock_registers:
                return [0x00] + mock_registers[reg_addr][:length]
            return [0x00] * (length + 1)
        else:  # Write operation
            return [0x00] * len(tx_data)


# Creating a Mock BHI260AP Class with patched functions for spidev
class MockBHI260AP(BHI260AP):
    def __init__(
        self,
        tag: str = "MockBHI260AP",
        spi_bus: int = 0,
        spi_cs: int = 2,
        clock_freq: int = 2000000,
        data_rate: int = 200,
        firmware_path: str = "./BHI260AP.fw",
        offline: bool = False,
    ):
        LOGGER.info("Initializing MockBHI260AP")
        self._tag = tag
        self._spi_bus = spi_bus
        self._spi_cs = spi_cs
        self._clock_freq = clock_freq
        self._data_rate = data_rate
        self._firmware_path = firmware_path
        self._is_streaming = False
        self._enabled_sensors = {}
        self._sensor_data = {}
        self._stale_data_tracker = {}

        # Use mock SPI instead of real spidev
        self._spi = MockSPI()

    def _upload_firmware(self) -> None:
        """Mock firmware upload"""
        pass

    def start(self):
        """Start the mock IMU"""
        self._spi.open(self._spi_bus, self._spi_cs)
        self._spi.max_speed_hz = self._clock_freq
        self._spi.mode = 0b00
        self._spi.bits_per_word = 8

        if not self.verify_connection():
            raise RuntimeError("Error connecting to IMU.")

        self.flush_buffer()
        self._is_streaming = True
        LOGGER.info("IMU started successfully.")

    def stop(self):
        """Stop the mock IMU"""
        LOGGER.info("Stopping IMU...")
        for sensor_id in list(self._enabled_sensors):
            try:
                self._disable_sensor(sensor_id)
            except Exception as e:
                print(f"Warning: Could not disable sensor {sensor_id}: {e}")
        self._spi.close()
        self._is_streaming = False
        LOGGER.info("IMU stopped successfully.")


@pytest.fixture
def sample_imu():
    return MockBHI260AP()


@pytest.fixture
def sample_imu_with_gyro():
    imu = MockBHI260AP()
    imu.start()
    imu.enable_gyroscope(rate_hz=200, dynamic_range=2000)
    return imu


@pytest.fixture
def sample_imu_with_accel():
    imu = MockBHI260AP()
    imu.start()
    imu.enable_accelerometer(rate_hz=200, dynamic_range=4096)
    return imu


@pytest.fixture
def sample_imu_with_data():
    imu = MockBHI260AP()
    imu.start()
    imu.enable_gyroscope(rate_hz=200, dynamic_range=2000)
    imu.enable_accelerometer(rate_hz=200, dynamic_range=4096)

    # Simulate sensor data
    imu._sensor_data = {
        BHI260AP.SENSOR_ID_GYR: [{"timestamp": 0.0, "x": 0.1, "y": 0.2, "z": 0.3}],
        BHI260AP.SENSOR_ID_ACC: [{"timestamp": 0.0, "x": 1.0, "y": 2.0, "z": 3.0}],
    }

    return imu


# Test initialization
def test_init_default(sample_imu: MockBHI260AP):
    assert all([
        sample_imu._spi_bus == 0,
        sample_imu._spi_cs == 2,
        sample_imu._clock_freq == 2000000,
        sample_imu._data_rate == 200,
        sample_imu._is_streaming is False,
        isinstance(sample_imu._enabled_sensors, dict),
        isinstance(sample_imu._sensor_data, dict),
    ])


def test_init_custom():
    imu = MockBHI260AP(tag="CustomBHI", spi_bus=1, spi_cs=3, clock_freq=1000000, data_rate=100)
    assert all([
        imu._tag == "CustomBHI",
        imu._spi_bus == 1,
        imu._spi_cs == 3,
        imu._clock_freq == 1000000,
        imu._data_rate == 100,
    ])


# Test connection verification
def test_verify_connection(sample_imu: MockBHI260AP):
    sample_imu.start()
    result = sample_imu.verify_connection()
    assert result is True


def test_read_chip_id(sample_imu: MockBHI260AP):
    sample_imu.start()
    chip_id = sample_imu.read_chip_id()
    assert chip_id in sample_imu.CHIP_ID


# Test start/stop
def test_start(sample_imu: MockBHI260AP):
    assert sample_imu._is_streaming is False

    sample_imu.start()

    assert all([
        sample_imu._is_streaming is True,
        sample_imu._spi.bus == 0,
        sample_imu._spi.cs == 2,
    ])


def test_stop_not_started(sample_imu: MockBHI260AP):
    assert sample_imu._is_streaming is False
    sample_imu.stop()
    assert sample_imu._is_streaming is False


def test_stop(sample_imu: MockBHI260AP):
    sample_imu.start()
    assert sample_imu._is_streaming is True

    sample_imu.stop()

    assert sample_imu._is_streaming is False


# Test sensor enabling
def test_enable_gyroscope(sample_imu: MockBHI260AP):
    sample_imu.start()

    assert len(sample_imu._enabled_sensors) == 0

    sample_imu.enable_gyroscope(rate_hz=200, dynamic_range=2000)

    assert BHI260AP.SENSOR_ID_GYR in sample_imu._enabled_sensors


def test_enable_accelerometer(sample_imu: MockBHI260AP):
    sample_imu.start()

    assert len(sample_imu._enabled_sensors) == 0

    sample_imu.enable_accelerometer(rate_hz=200, dynamic_range=4096)

    assert BHI260AP.SENSOR_ID_ACC in sample_imu._enabled_sensors


def test_enable_linear_acceleration(sample_imu: MockBHI260AP):
    sample_imu.start()

    assert len(sample_imu._enabled_sensors) == 0

    sample_imu.enable_linear_acceleration(rate_hz=200, dynamic_range=4096)

    assert BHI260AP.SENSOR_ID_LIN_ACC in sample_imu._enabled_sensors


def test_enable_gravity(sample_imu: MockBHI260AP):
    sample_imu.start()

    assert len(sample_imu._enabled_sensors) == 0

    sample_imu.enable_gravity(rate_hz=200, dynamic_range=4096)

    assert BHI260AP.SENSOR_ID_GRAVITY in sample_imu._enabled_sensors


# Test invalid dynamic range
def test_enable_gyroscope_invalid_range(sample_imu: MockBHI260AP):
    sample_imu.start()

    with pytest.raises(RuntimeError):
        sample_imu.enable_gyroscope(rate_hz=200, dynamic_range=999)


def test_enable_accelerometer_invalid_range(sample_imu: MockBHI260AP):
    sample_imu.start()

    with pytest.raises(RuntimeError):
        sample_imu.enable_accelerometer(rate_hz=200, dynamic_range=999)


# Test properties
def test_is_streaming(sample_imu: MockBHI260AP):
    assert sample_imu.is_streaming is False

    sample_imu.start()
    assert sample_imu.is_streaming is True


def test_data_property(sample_imu_with_data: MockBHI260AP):
    data = sample_imu_with_data.data
    assert isinstance(data, dict)
    assert len(data) == 2


def test_gyro_x(sample_imu_with_data: MockBHI260AP):
    gyro_x = sample_imu_with_data.gyro_x
    assert isinstance(gyro_x, float)


def test_gyro_y(sample_imu_with_data: MockBHI260AP):
    gyro_y = sample_imu_with_data.gyro_y
    assert isinstance(gyro_y, float)


def test_gyro_z(sample_imu_with_data: MockBHI260AP):
    gyro_z = sample_imu_with_data.gyro_z
    assert isinstance(gyro_z, float)


def test_acc_x(sample_imu_with_data: MockBHI260AP):
    acc_x = sample_imu_with_data.acc_x
    assert isinstance(acc_x, float)


def test_acc_y(sample_imu_with_data: MockBHI260AP):
    acc_y = sample_imu_with_data.acc_y
    assert isinstance(acc_y, float)


def test_acc_z(sample_imu_with_data: MockBHI260AP):
    acc_z = sample_imu_with_data.acc_z
    assert isinstance(acc_z, float)


def test_gyro_property(sample_imu_with_data: MockBHI260AP):
    gyro = sample_imu_with_data.gyro
    assert isinstance(gyro, np.ndarray)


def test_accel_property(sample_imu_with_data: MockBHI260AP):
    accel = sample_imu_with_data.accel
    assert isinstance(accel, np.ndarray)


# Test FIFO parsing
def test_parse_fifo_empty(sample_imu: MockBHI260AP):
    parsed_data = sample_imu._parse_fifo(b"")
    assert parsed_data == []


def test_parse_fifo_with_padding(sample_imu: MockBHI260AP):
    # FIFO data with only padding bytes
    fifo_data = bytes([sample_imu.FIFO_EVENT_PADDING, sample_imu.FIFO_EVENT_FILLER])
    parsed_data = sample_imu._parse_fifo(fifo_data)
    assert parsed_data == []


# Test sensor data retrieval
def test_get_sensor_data_empty(sample_imu: MockBHI260AP):
    data = sample_imu._get_sensor_data(BHI260AP.SENSOR_ID_GYR, most_recent=True)
    assert len(data) == 0


def test_get_sensor_data_with_data(sample_imu_with_data: MockBHI260AP):
    data = sample_imu_with_data._get_sensor_data(BHI260AP.SENSOR_ID_GYR, most_recent=True)
    assert isinstance(data, np.ndarray)
    assert len(data) == 3


# Test boot status properties
def test_host_interface_ready(sample_imu: MockBHI260AP):
    # The mock returns 0x10 for boot status, which has bit 4 set
    sample_imu.start()
    is_ready = sample_imu.host_interface_ready
    assert isinstance(is_ready, (bool, np.bool_))


def test_firmware_verify_done(sample_imu: MockBHI260AP):
    sample_imu.start()
    is_done = sample_imu.firmware_verify_done
    assert isinstance(is_done, (bool, np.bool_))


def test_firmware_idle(sample_imu: MockBHI260AP):
    sample_imu.start()
    is_idle = sample_imu.firmware_idle
    assert isinstance(is_idle, (bool, np.bool_))


def test_power_state(sample_imu: MockBHI260AP):
    sample_imu.start()
    power = sample_imu.power_state
    assert isinstance(power, (bool, np.bool_))


# Test read methods
def test_read_rom_version(sample_imu: MockBHI260AP):
    sample_imu.start()
    rom_version = sample_imu.read_rom_version()
    assert isinstance(rom_version, int)


def test_read_kernel_version(sample_imu: MockBHI260AP):
    sample_imu.start()
    kernel_version = sample_imu.read_kernel_version()
    assert isinstance(kernel_version, int)


def test_read_product_id(sample_imu: MockBHI260AP):
    sample_imu.start()
    product_id = sample_imu.read_product_id()
    assert product_id == sample_imu.FUSER2_PRODUCT_ID
