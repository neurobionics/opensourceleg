"""
Module for interfacing with IMU sensors using the MSCL, Adafruit, and SPI libraries.

This module provides three IMU sensor implementations:
  - LordMicrostrainIMU: Uses the MSCL library to interface with a Lord Microstrain IMU.
  - BNO055: Uses the Adafruit BNO055 library to interface with a Bosch BNO055 IMU.
  - BHI260AP: Uses the SPI library to interface with a Bosch BHI260AP IMU.

Dependencies:
  - MSCL (for LordMicrostrainIMU): https://github.com/LORD-MicroStrain/MSCL/tree/master
  - adafruit_bno055, board, busio (for BNO055)
  - spidev, firmware (for BHI260AP): https://github.com/boschsensortec/BHI2xy_SensorAPI/tree/master/firmware/bhi260ap

Ensure that the required libraries are installed and that the library paths are added
to PYTHONPATH or sys.path if necessary.
"""

import os
import struct
import time
from typing import Any, Callable, ClassVar, Union, cast

import numpy as np

from opensourceleg.logging import LOGGER
from opensourceleg.sensors.base import IMUBase, check_sensor_stream


class LordMicrostrainIMU(IMUBase):
    # LordMicrostrain-specific offline configuration
    _OFFLINE_PROPERTIES: ClassVar[list[str]] = [
        *IMUBase._OFFLINE_PROPERTIES,
        "roll",
        "pitch",
        "yaw",
        "vel_x",
        "vel_y",
        "vel_z",
        "timestamp",
    ]
    _OFFLINE_PROPERTY_DEFAULTS: ClassVar[dict[str, Any]] = {
        **IMUBase._OFFLINE_PROPERTY_DEFAULTS,
        "data": {
            "estRoll": 0.0,
            "estPitch": 0.0,
            "estYaw": 0.0,
            "estAngularRateX": 0.0,
            "estAngularRateY": 0.0,
            "estAngularRateZ": 0.0,
            "estLinearAccelX": 0.0,
            "estLinearAccelY": 0.0,
            "estLinearAccelZ": 0.0,
            "estFilterGpsTimeTow": 0.0,
        },
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "vel_x": 0.0,
        "vel_y": 0.0,
        "vel_z": 0.0,
        "timestamp": 0.0,
    }
    """
    Sensor class for the Lord Microstrain IMU.

    This class interfaces with a Lord Microstrain Inertial Measurement Unit (IMU)
    via the MSCL library. It returns Euler angles (in radians), angular rates (in rad/s),
    and linear accelerations (in m/s^2).

    Resources:
        - Download v0.65 MSCL pre-built package for Raspbian:
          https://github.com/LORD-MicroStrain/MSCL/releases/download/v65.0.0/python3-mscl_65.0.0_arm64.deb
        - Read the MSCL installation instructions:
          https://github.com/LORD-MicroStrain/MSCL/blob/master/HowToUseMSCL.md
        - We assume that the MSCL library is installed in /usr/share/python3-mscl

    Note: MSCL v65 may encounter page alignment errors on some Raspberry Pi 5 kernels.
        Updating to a newer Raspberry Pi OS image with the rpi-v8 kernel
        or upgrading to MSCL v67.1.0 resolves the issue:
        https://github.com/LORD-MicroStrain/MSCL/releases/download/v67.1.0/MSCL_arm64_Python3.11_v67.1.0.deb
    """

    def __init__(
        self,
        tag: str = "LordMicrostrainIMU",
        port: str = r"/dev/ttyUSB0",
        baud_rate: int = 921600,
        frequency: int = 200,
        update_timeout: int = 500,
        max_packets: int = 1,
        return_packets: bool = False,
        offline: bool = False,
    ) -> None:
        """
        Initialize the LordMicrostrainIMU sensor.

        Args:
            port (str, optional): Serial port for the IMU. Defaults to "/dev/ttyUSB0".
            baud_rate (int, optional): Baud rate for the serial connection. Defaults to 921600.
            frequency (int, optional): Data streaming frequency in Hz. Defaults to 200.
            update_timeout (int, optional): Timeout for data packet retrieval in milliseconds. Defaults to 500.
            max_packets (int, optional): Maximum number of data packets to retrieve. Defaults to 1.
            return_packets (bool, optional): If True, returns the raw data packets. Defaults to False.
        """
        # Attempt to import the MSCL library and add its path.
        try:
            try:
                import mscl
            except (ImportError, ModuleNotFoundError):
                # Falling back to old method of appending to sys.path for importing older versions of mscl
                import sys

                legacy_path = "/usr/share/python3-mscl"
                if legacy_path not in sys.path:
                    sys.path.append(legacy_path)
                import mscl

            self.mscl = mscl
        except ImportError:
            LOGGER.warning(
                "Failed to import mscl. Please install the MSCL library from Lord Microstrain and append the path "
                "to the PYTHONPATH or sys.path. Checkout https://github.com/LORD-MicroStrain/MSCL/tree/master "
                "and https://lord-microstrain.github.io/MSCL/Documentation/MSCL%20API%20Documentation/index.html"
                "If you are using a newer version of MSCL, you may need to add /usr/lib/python3.version/dist-packages \
                to PYTHONPATH"
            )

            if not offline:
                exit(1)

        self._init_variables(
            tag=tag,
            port=port,
            baud_rate=baud_rate,
            frequency=frequency,
            update_timeout=update_timeout,
            max_packets=max_packets,
            return_packets=return_packets,
            offline=offline,
        )

    def _init_variables(
        self,
        tag: str,
        port: str,
        baud_rate: int,
        frequency: int,
        update_timeout: int,
        max_packets: int,
        return_packets: bool,
        offline: bool,
    ) -> None:
        super().__init__(tag=tag, offline=offline)
        self._port = port
        self._baud_rate = baud_rate
        self._frequency = frequency
        self._update_timeout = update_timeout
        self._max_packets = max_packets
        self._return_packets = return_packets
        self._is_streaming = False
        self._connection = None
        self._data: dict[str, float] = {}

    def _configure_mip_channels(self) -> Any:
        """
        Configure and return the MIP channels for data streaming.

        Sets up the desired channels for:
          - Estimated orientation (Euler angles)
          - Estimated angular rate
          - Estimated linear acceleration
          - GPS timestamp

        Returns:
            Any: A configured MipChannels object for the MSCL InertialNode.
        """
        channels = self.mscl.MipChannels()
        channels.append(
            self.mscl.MipChannel(
                self.mscl.MipTypes.CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_EULER,
                self.mscl.SampleRate.Hertz(self.frequency),
            )
        )
        channels.append(
            self.mscl.MipChannel(
                self.mscl.MipTypes.CH_FIELD_ESTFILTER_ESTIMATED_ANGULAR_RATE,
                self.mscl.SampleRate.Hertz(self.frequency),
            )
        )
        channels.append(
            self.mscl.MipChannel(
                self.mscl.MipTypes.CH_FIELD_ESTFILTER_ESTIMATED_LINEAR_ACCEL,
                self.mscl.SampleRate.Hertz(self.frequency),
            )
        )
        channels.append(
            self.mscl.MipChannel(
                self.mscl.MipTypes.CH_FIELD_ESTFILTER_GPS_TIMESTAMP,
                self.mscl.SampleRate.Hertz(self.frequency),
            )
        )

        return channels

    def set_update_timeout(self, timeout: int) -> None:
        """
        Set the update timeout for the sensor.
        """
        self._update_timeout = timeout

    def set_max_packets(self, max_packets: int) -> None:
        """
        Set the maximum number of packets to retrieve.
        """
        self._max_packets = max_packets

    def set_return_packets(self, return_packets: bool) -> None:
        """
        Set the return packets flag.
        """
        self._return_packets = return_packets

    def start(self) -> None:
        """
        Start the Lord Microstrain IMU sensor.

        Establishes a serial connection, configures the MIP channels, enables data streaming,
        and sets the streaming flag to True.
        """
        try:
            self._connection = self.mscl.Connection.Serial(os.path.realpath(self.port), self.baud_rate)
        except RuntimeError as e:
            LOGGER.error(f"Failed to connect to the IMU at {self.port}: {e}")
            exit(1)

        self._node = self.mscl.InertialNode(self._connection)
        self._validate_frequency()
        self._node.setActiveChannelFields(self.mscl.MipTypes.CLASS_ESTFILTER, self._configure_mip_channels())
        self._node.enableDataStream(self.mscl.MipTypes.CLASS_ESTFILTER)
        self._is_streaming = True

    @check_sensor_stream
    def stop(self) -> None:
        """
        Stop the Lord Microstrain IMU sensor.

        Sets the node to idle mode and updates the streaming flag to False.

        Raises:
            SensorNotStreamingException: If the sensor is not currently streaming.
        """
        self._node.setToIdle()
        self._is_streaming = False

    @check_sensor_stream
    def ping(self) -> None:
        """
        Ping the Lord Microstrain IMU sensor to verify connectivity.

        Logs an info message if the ping is successful, otherwise logs an error.

        Raises:
            SensorNotStreamingException: If the sensor is not currently streaming.
        """
        response = self._node.ping()

        if response.success():
            LOGGER.info(f"Successfully pinged the IMU at {self.port}")
        else:
            LOGGER.error(f"Failed to ping the IMU at {self.port}")

    def update(self) -> Union[None, Any]:
        """
        Retrieve and update IMU data from the sensor. To modify update parameters, use the set_update_timeout,
        set_max_packets, and set_return_packets methods.

        Returns:
            Union[None, Any]: Returns the data packets if `return_packets` is True; otherwise, None.
        """
        data_packets = self._node.getDataPackets(timeout=self.update_timeout, maxPackets=self.max_packets)
        data_points = data_packets[-1].data()
        self._data = {data.channelName(): data.as_float() for data in data_points}

        if self.return_packets:
            return data_packets
        else:
            return None

    @property
    def port(self) -> str:
        """
        Get the serial port used by the sensor.

        Returns:
            str: The serial port.
        """
        return self._port

    @property
    def baud_rate(self) -> int:
        """
        Get the baud rate used for the sensor connection.

        Returns:
            int: The baud rate.
        """
        return self._baud_rate

    @property
    def frequency(self) -> int:
        """
        Get the data streaming frequency of the sensor.

        Returns:
            int: The streaming frequency in Hz.
        """
        return self._frequency

    @property
    def is_streaming(self) -> bool:
        """
        Check if the sensor is currently streaming data.

        Returns:
            bool: True if streaming; otherwise, False.
        """
        return self._is_streaming

    @property
    def update_timeout(self) -> int:
        """
        Get the update timeout for the sensor.
        """
        return self._update_timeout

    @property
    def max_packets(self) -> int:
        """
        Get the maximum number of packets to retrieve.
        """
        return self._max_packets

    @property
    def return_packets(self) -> bool:
        """
        Get the return packets flag.
        """
        return self._return_packets

    @property
    def data(self) -> dict[str, float]:
        """
        Get the latest sensor data.

        Returns:
            dict[str, float]: A dictionary mapping channel names to their float values.
        """
        return self._data

    @property
    def roll(self) -> float:
        """
        Get the estimated roll angle in radians.

        Returns:
            float: Roll angle (rad).
        """
        return self._data["estRoll"]

    @property
    def pitch(self) -> float:
        """
        Get the estimated pitch angle in radians.

        Returns:
            float: Pitch angle (rad).
        """
        return self._data["estPitch"]

    @property
    def yaw(self) -> float:
        """
        Get the estimated yaw angle in radians.

        Returns:
            float: Yaw angle (rad).
        """
        return self._data["estYaw"]

    @property
    def vel_x(self) -> float:
        """
        Get the estimated angular velocity about the x-axis in rad/s.

        Returns:
            float: Angular velocity (rad/s) about the x-axis.
        """
        return self._data["estAngularRateX"]

    @property
    def vel_y(self) -> float:
        """
        Get the estimated angular velocity about the y-axis in rad/s.

        Returns:
            float: Angular velocity (rad/s) about the y-axis.
        """
        return self._data["estAngularRateY"]

    @property
    def vel_z(self) -> float:
        """
        Get the estimated angular velocity about the z-axis in rad/s.

        Returns:
            float: Angular velocity (rad/s) about the z-axis.
        """
        return self._data["estAngularRateZ"]

    @property
    def acc_x(self) -> float:
        """
        Get the estimated linear acceleration along the x-axis in m/s².

        Returns:
            float: Linear acceleration (m/s²) along the x-axis.
        """
        return self._data["estLinearAccelX"]

    @property
    def acc_y(self) -> float:
        """
        Get the estimated linear acceleration along the y-axis in m/s².

        Returns:
            float: Linear acceleration (m/s²) along the y-axis.
        """
        return self._data["estLinearAccelY"]

    @property
    def acc_z(self) -> float:
        """
        Get the estimated linear acceleration along the z-axis in m/s².

        Returns:
            float: Linear acceleration (m/s²) along the z-axis.
        """
        return self._data["estLinearAccelZ"]

    @property
    def gyro_x(self) -> float:
        """
        Get the measured gyroscopic value for the x-axis.

        Note:
            Gyro data is not available for the Lord Microstrain IMU, so this returns 0.0
            and logs a warning.

        Returns:
            float: 0.0
        """
        LOGGER.warning("Gyro data not available for Lord Microstrain IMU")
        return 0.0

    @property
    def gyro_y(self) -> float:
        """
        Get the measured gyroscopic value for the y-axis.

        Note:
            Gyro data is not available for the Lord Microstrain IMU, so this returns 0.0
            and logs a warning.

        Returns:
            float: 0.0
        """
        LOGGER.warning("Gyro data not available for Lord Microstrain IMU")
        return 0.0

    @property
    def gyro_z(self) -> float:
        """
        Get the measured gyroscopic value for the z-axis.

        Note:
            Gyro data is not available for the Lord Microstrain IMU, so this returns 0.0
            and logs a warning.

        Returns:
            float: 0.0
        """
        LOGGER.warning("Gyro data not available for Lord Microstrain IMU")
        return 0.0

    @property
    def timestamp(self) -> float:
        """
        Get the timestamp of the latest data packet in seconds.

        Returns:
            float: Timestamp (s) from the sensor data.
        """
        return self._data["estFilterGpsTimeTow"]

    def _validate_frequency(self) -> None:
        """
        Check and adjust frequency for compatibility with IMU refresh rate.
        """

        imu_sample_rate = self._node.getDataRateBase(self.mscl.MipTypes.CLASS_ESTFILTER)

        if self._frequency > imu_sample_rate or imu_sample_rate % self._frequency != 0:
            min_dist = float("inf")
            best_frequency = 200

            for divisor in range(1, int(np.sqrt(imu_sample_rate)) + 1):
                if imu_sample_rate % divisor == 0:
                    for candidate in [divisor, imu_sample_rate // divisor]:
                        dist = abs(self._frequency - candidate)
                        if dist < min_dist:
                            min_dist = dist
                            best_frequency = candidate

            LOGGER.info(
                f"""{self._frequency} is not a valid decimation of {imu_sample_rate},
                choosing closest decimation: {best_frequency}"""
            )

            self._frequency = best_frequency


class BNO055(IMUBase):
    """
    Sensor class for the Bosch BNO055 IMU.

    This class wraps the Adafruit BNO055 library to provide an interface
    consistent with the OSL sensor framework.

    Connections:
        - The sensor should be connected to the main I2C bus.
        - UART connectivity is not implemented.

    Requirements:
        - adafruit_bno055
        - board
        - busio

    Author:
        Kevin Best

    Date:
        8/22/2024
    """

    def __init__(
        self,
        tag: str = "BNO055",
        addr: int = 40,
        offline: bool = False,
    ) -> None:
        """
        Initialize the BNO055 sensor.

        Args:
            addr (int, optional): I2C address of the BNO055 sensor. Defaults to 40.
        """
        # Attempt to import the required libraries.
        try:
            import adafruit_bno055
            import board
            import busio

            self.adafruit_bno055 = adafruit_bno055
            self.board = board
            self.busio = busio
        except ImportError as e:
            LOGGER.error(f"BNO055IMU requires adafruit_bno055, board, and busio packages. Error: {e}")
            exit(1)

        super().__init__(tag=tag, offline=offline)
        self._address: int = addr
        self._gyro_data: list[float] = [0.0, 0.0, 0.0]
        self._acc_data: list[float] = [0.0, 0.0, 0.0]
        self._is_streaming = False

    def start(self) -> None:
        """
        Start the BNO055 sensor.

        Initializes the I2C bus, creates an instance of the Adafruit BNO055 sensor,
        configures the sensor settings, and sets the streaming flag to True.
        """
        i2c = self.busio.I2C(self.board.SCL, self.board.SDA)
        try:
            self._adafruit_imu = self.adafruit_bno055.BNO055_I2C(i2c, address=self._address)
        except ValueError:
            LOGGER.error("BNO055 IMU Not Found on i2c bus! Check wiring!")
        self.configure_IMU_settings()
        self._is_streaming = True

    def stop(self) -> None:
        """
        Stop the BNO055 sensor.

        Sets the streaming flag to False.
        """
        self._is_streaming = False

    def update(self) -> None:
        """
        Update the sensor data from the BNO055.

        Reads the latest acceleration and gyroscopic data from the sensor.
        """
        self._acc_data = self._adafruit_imu.acceleration
        self._gyro_data = self._adafruit_imu.gyro

    def configure_IMU_settings(self) -> None:
        """
        Configure the BNO055 sensor settings.

        Hard-coded configuration:
            - Enables external crystal.
            - Sets mode to ACCGYRO_MODE.
            - Configures accelerometer range to ACCEL_2G and bandwidth to ACCEL_15_63HZ.
            - Configures gyroscope range to GYRO_1000_DPS and bandwidth to GYRO_23HZ.
        """
        self._adafruit_imu.use_external_crystal = True
        self._adafruit_imu.mode = self.adafruit_bno055.ACCGYRO_MODE
        self._adafruit_imu.accel_range = self.adafruit_bno055.ACCEL_2G
        self._adafruit_imu.accel_bandwidth = self.adafruit_bno055.ACCEL_15_63HZ
        self._adafruit_imu.gyro_range = self.adafruit_bno055.GYRO_1000_DPS
        self._adafruit_imu.gyro_bandwidth = self.adafruit_bno055.GYRO_23HZ

    @property
    def acc_x(self) -> float:
        """
        Get the measured acceleration along the x-axis in m/s².

        Returns:
            float: Acceleration (m/s²) along the x-axis.
        """
        return self._acc_data[0]

    @property
    def acc_y(self) -> float:
        """
        Get the measured acceleration along the y-axis in m/s².

        Returns:
            float: Acceleration (m/s²) along the y-axis.
        """
        return self._acc_data[1]

    @property
    def acc_z(self) -> float:
        """
        Get the measured acceleration along the z-axis in m/s².

        Returns:
            float: Acceleration (m/s²) along the z-axis.
        """
        return self._acc_data[2]

    @property
    def gyro_x(self) -> float:
        """
        Get the measured rotational velocity about the x-axis in rad/s.

        Returns:
            float: Angular velocity (rad/s) about the x-axis.
        """
        return self._gyro_data[0]

    @property
    def gyro_y(self) -> float:
        """
        Get the measured rotational velocity about the y-axis in rad/s.

        Returns:
            float: Angular velocity (rad/s) about the y-axis.
        """
        return self._gyro_data[1]

    @property
    def gyro_z(self) -> float:
        """
        Get the measured rotational velocity about the z-axis in rad/s.

        Returns:
            float: Angular velocity (rad/s) about the z-axis.
        """
        return self._gyro_data[2]

    @property
    def is_streaming(self) -> bool:
        """
        Check if the BNO055 sensor is streaming data.

        Returns:
            bool: True if streaming; otherwise, False.
        """
        return self._is_streaming


class BHI260AP(IMUBase):
    """
    Sensor class for BHI250AP IMU.

    This class interfaces with a Bosch BHI260AP Inertial Measurement Unit (IMU). It
    can return gyroscope (rad/s), raw accelerometer (m/s^2), gravity (m/s^2),
    and linear acceleration (m/s^2).

    Requirements:
        - spidev

    Resources:
        - Download "BHI260AP.fw" firmware:
        https://github.com/boschsensortec/BHI2xy_SensorAPI/tree/master/firmware/bhi260ap

    Author:
        Katharine Walters

    Date:
        01/13/2026
    """

    # Register addresses - DMA Channels
    REG_CHAN0_CMD = 0x00  # Host Channel 0 - Command Input (write-only)
    REG_CHAN1_WAKEUP_FIFO = 0x01  # Host Channel 1 - Wake-up FIFO (read-only)
    REG_CHAN2_NONWAKEUP_FIFO = 0x02  # Host Channel 2 - Non-Wake-up FIFO (read-only)
    REG_CHAN3_STATUS = 0x03  # Host Channel 3 - Status/Debug FIFO (read-only)

    # Control registers
    REG_CHIP_CTRL = 0x05  # Chip Control
    REG_HOST_INTERFACE_CTRL = 0x06  # Host Interface Control
    REG_HOST_INT_CTRL = 0x07  # Host Interrupt Control
    REG_RESET_REQ = 0x14  # Reset Request
    REG_HOST_CTRL = 0x16  # Host Control (SPI mode, I2C watchdog)
    REG_HOST_STATUS = 0x17  # Host Status

    # Identification registers
    REG_FUSER2_PRODUCT_ID = 0x1C  # Fuser2 Product ID (0x89)
    REG_FUSER2_REV_ID = 0x1D  # Fuser2 Revision ID (0x02 or 0x03)
    REG_ROM_VERSION = 0x1E  # ROM Version (2 bytes)
    REG_KERNEL_VERSION = 0x20  # Kernel Version (2 bytes)
    REG_USER_VERSION = 0x22  # User Version (2 bytes)
    REG_CHIP_ID = 0x2B  # Chip ID (0x70 or 0xF0)

    # Status and error registers
    REG_FEATURE_STATUS = 0x24  # Feature Status
    REG_BOOT_STATUS = 0x25  # Boot Status
    REG_INT_STATUS = 0x2D  # Interrupt Status
    REG_ERROR_VALUE = 0x2E  # Error Value
    REG_ERROR_AUX = 0x2F  # Error Auxiliary
    REG_DEBUG_VALUE = 0x30  # Debug Value
    REG_DEBUG_STATE = 0x31  # Debug State
    REG_PHYS_SENS_INFO = 0x0121  # Physical sensor information

    # Expected chip ID values
    FUSER2_PRODUCT_ID = 0x89  # Fuser2 core identifier
    CHIP_ID: ClassVar[list[int]] = [0x70, 0xF0]  # Valid BHI260AP chip IDs

    # Commands
    CMD_SOFT_RESET = 0x01
    CMD_FIFO_FLUSH = 0x0009
    CMD_CONFIG_SENSOR = 0x000D
    CMD_CHNG_SENSOR_DYN_RNG = 0x000E

    # Parameters
    MAX_MESSAGE_LEN = 256  # Max number of message bytes
    DATA_RATES = (1.5625, 3.125, 6.25, 12.5, 25, 50, 100, 200, 400, 800)
    SENSOR_RANGES: ClassVar[dict[int, tuple]] = {
        1: (2048, 4096, 8192, 16384),  # (LSB)
        4: (2048, 4096, 8192, 16384),  # (LSB)
        10: (250, 500, 1000, 2000),  # (dps)
        13: (250, 500, 1000, 2000),  # (dps)
        28: (2048, 4096, 8192, 16384),  # (LSB)
        31: (2048, 4096, 8192, 16384),  # (LSB)
    }
    GRAVITY = 9.81  # (m/s^2)

    # Sensor IDs
    SENSOR_ID_ACC = 4  # Corrected accelerometer (Non-wakeup)
    SENSOR_ID_GYR = 13  # Corrected gyroscope (Non-wakeup)
    SENSOR_ID_GRAVITY = 28  # (Non-wakeup)
    SENSOR_ID_LIN_ACC = 31  # Linear acceleration (defined as accelerometer - gravity)

    # FIFO Event IDs for timestamps and meta events
    FIFO_EVENT_SMALL_DELTA_TS = 0xFB  # Small Delta Timestamp (251)
    FIFO_EVENT_LARGE_DELTA_TS = 0xFC  # Large Delta Timestamp (252)
    FIFO_EVENT_FULL_TS = 0xFD  # Full Timestamp (253)
    FIFO_EVENT_META = 0xFE  # Meta Event (254)
    FIFO_EVENT_FILLER = 0xFF  # Filler byte (255)
    FIFO_EVENT_PADDING = 0x00  # Padding byte (0)

    # Sensor Dict
    SENSOR_DICT: ClassVar[dict[str, int]] = {
        "Gyro": SENSOR_ID_GYR,
        "Accel": SENSOR_ID_ACC,
        "Gravity": SENSOR_ID_GRAVITY,
        "LinAccel": SENSOR_ID_LIN_ACC,
    }
    _SENSOR_ID_TO_NAME: ClassVar[dict[int, str]] = {v: k for k, v in SENSOR_DICT.items()}

    def __init__(
        self,
        tag: str = "BHI260AP",
        spi_bus: int = 0,
        spi_cs: int = 2,
        clock_freq: int = 2000000,
        data_rate: int = 200,
        firmware_path: str = "./BHI260AP.fw",
        offline: bool = False,
    ) -> None:
        """
        Initialize BHI260AP IMU

        Args:
            tag (str): Identifier for the IMU instance
            spi_bus (int): SPI bus number
            spi_cs (int): SPI chip select
            clock_freq (int): SPI clock speed in Hz
            data_rate (int): sensor sample rate in Hz
            firmware_path (str): path to firmware file
        """
        try:
            import spidev

            self._spi = spidev.SpiDev()
        except ImportError as e:
            LOGGER.error(f"Failed to import required libraries. BHI260AP requires spidev. Error: {e}")

            if not offline:
                exit(1)

        # Check parameters passed to initializer
        if data_rate not in self.DATA_RATES:
            LOGGER.warning(
                f"Requested data rate will be modified to match the nearest value equal to or greater "
                f"than the requested rate in {self.DATA_RATES}."
            )

        self._tag = tag
        self._spi_bus = spi_bus
        self._spi_cs = spi_cs
        self._clock_freq = clock_freq
        self._data_rate = data_rate
        self._firmware_path = firmware_path
        self._is_streaming = False

        self._enabled_sensors: dict[int, float] = {}
        self._sensor_data: dict[int, list[dict[str, Union[int, float]]]] = {}

        # Tracker for stale data
        self._stale_data_tracker: dict[int, int] = {}
        self._stale_threshold = 2

    def start(self) -> None:
        """
        Start the IMU by opening the SPI port
        """
        LOGGER.info("Starting BHI260AP IMU...")
        self._spi.open(self._spi_bus, self._spi_cs)
        self._spi.max_speed_hz = self._clock_freq
        self._spi.mode = 0b00  # CPOL=0, CPHA=0
        self._spi.bits_per_word = 8

        # Verify SPI connection
        if not self.verify_connection():
            raise RuntimeError("Error connecting to IMU.")

        # If kernel version = 0x00, auto-load firmware
        if self.read_kernel_version() == 0:
            self._upload_firmware()

        # Clear the first interrupt by flushing the content of the FIFO
        self.flush_buffer()

        self._is_streaming = True
        LOGGER.info("IMU started successfully.")

    def stop(self) -> None:
        """
        Stop the IMU by closing the SPI port.
        """
        LOGGER.info("Stopping IMU...")
        for sensor_id in list(self._enabled_sensors):
            try:
                self._disable_sensor(sensor_id)
            except Exception as e:
                LOGGER.error(f"Warning: Could not disable sensor {sensor_id}: {e}")
        self._spi.close()
        self._is_streaming = False
        LOGGER.info("IMU stopped successfully.")

    def _read_register(
        self, reg_addr: int, length: int = 1, return_bit_string: bool = False
    ) -> Union[list[int], np.ndarray]:
        """Read from register(s)"""
        # SPI read: set MSB to 1 for read operation
        tx_data = [reg_addr | 0x80] + [0x00] * length
        rx_data = self._spi.xfer2(tx_data)[1 : length + 1]
        if return_bit_string:
            bit_string = np.unpackbits(np.frombuffer(bytes(rx_data), dtype=np.uint8))[::-1]
            return bit_string  # LSB first in each byte
        else:
            return cast(list[int], rx_data)

    def _write_register(self, reg_addr: int, data: Union[int, list[int]]) -> None:
        """Write to register"""
        # SPI write: MSB is 0 for write operation
        tx_data = [reg_addr & 127, *data] if isinstance(data, list) else [reg_addr & 127, data]

        self._spi.xfer2(tx_data)
        time.sleep(0.001)  # Small delay after write

    def _read_parameter(self, param_id: int) -> None:
        """
        Initiate a read access. This command needs to be run before
        reading status from output channel 3.
        """
        cmd = list(struct.pack("<HH", param_id | 0x1000, 0))
        self._write_register(self.REG_CHAN0_CMD, cmd)

    def _poll_register_until(
        self,
        condition_property: Callable[[], Any],
        expected_value: bool = True,
        max_attempts: int = 250,
        delay: float = 0.0001,
        error_msg: str = "Error, expected register value not read",
    ) -> bool:
        """
        Polls a condition function until it returns the expected value or times out
        Args:
            condition_property (property): function to call repeatedly
            expected_value (any): value that indicates success
            max_attempts (int): maximum number of polling attempts
            delay (float): delay between polls in seconds
            error_msg (str): message printed if expected value not read

        Returns:
            bool: True if condition met, False if timeout
        """
        for _attempt in range(max_attempts):
            if condition_property() == expected_value:
                return True
            time.sleep(delay)
        LOGGER.info(error_msg)
        return False

    def _soft_reset(self) -> None:
        """
        Perform soft reset
        """
        # Write 0x01 to Reset Request register (bit 0 = 1)
        # This MUST be a single register write (no burst)
        self._write_register(self.REG_RESET_REQ, 0x01)

        # Wait T_wait = 4 μs minimum (device may wake from sleep)
        time.sleep(0.0001)

    def flush_buffer(self) -> None:
        """
        Flushes (discards) all FIFO buffers
        """
        cmd = struct.pack("<HHB3x", self.CMD_FIFO_FLUSH, 4, 0xFE)
        self._write_register(self.REG_CHAN0_CMD, list(cmd))

        time.sleep(0.05)

    def _upload_firmware(self) -> None:
        """
        Boots firmware to RAM
        """
        LOGGER.info("Starting firmware upload...")

        try:
            # Load firmware
            firmware = open(self._firmware_path, "rb").read()
            firmware_len_words = len(firmware) // 4  # Load firmware in multiple of 4 bytes
        except ImportError:
            LOGGER.error(
                "Failed to open BHI260AP firmware. Download 'BHI260AP.fw' from"
                "https://github.com/boschsensortec/BHI2xy_SensorAPI/tree/master/firmware/bhi260ap and"
                "provide filepath in constructor. Error: {e}"
            )

        # Perform soft reset to make host interface ready
        self._soft_reset()

        # Poll Boot Status register until Host Interface Ready bit is set
        self._poll_register_until(
            lambda: self.host_interface_ready, error_msg="Error, host interface ready bit not set after resetting"
        )

        # Send 'Upload to Program RAM' command
        cmd = struct.pack("<HH", 0x0002, firmware_len_words)
        self._write_register(self.REG_CHAN0_CMD, list(cmd))

        # Send firmware data in chunks
        offset = 0
        while offset < len(firmware):
            chunk = firmware[offset : offset + self.MAX_MESSAGE_LEN]
            self._write_register(self.REG_CHAN0_CMD, list(chunk))
            offset += self.MAX_MESSAGE_LEN

        # Poll Boot Status register until Firmware Verify Done bit is set
        self._poll_register_until(lambda: self.firmware_verify_done, error_msg="Error, firmware not verified")

        # Send 'Boot Program RAM' command to start execution of firmware
        boot_cmd = struct.pack("<HH", 0x0003, 0)
        self._write_register(self.REG_CHAN0_CMD, list(boot_cmd))

        # Poll Boot Status register until Host Interface Ready bit is set again
        self._poll_register_until(
            lambda: self.host_interface_ready,
            delay=0.001,
            error_msg="Error, host interface ready bit not set after firmware boot",
        )

        # Check kernel version is updated after uploading and booting firmware
        if self.read_kernel_version() != 0:
            LOGGER.info("Firmware booted and verified successfully.")
        else:
            raise RuntimeError("Error, BHI260AP firmware boot not successful. ")

    def _enable_sensor(
        self, sensor_id: int, scale: float, dynamic_range: int = -1, rate_hz: float = -1, latency: int = 0
    ) -> None:
        """
        Enable a virtual sensor

        Args:
            sensor_id (int): ID for virtual sensor to enable
            scale (float): convert raw value to correct units based on dynamic range
            dynamic_range (int): dynamic measurement range
            rate_hz (float): sample rate in hz
            latency (int): sensor latency in milliseconds
        """
        if rate_hz < 0:
            rate_hz = self._data_rate

        # Pack sample rate as 32-bit float
        rate_bytes = struct.pack("<f", rate_hz)

        # Pack latency as 24-bit value
        latency_bytes = struct.pack("<I", latency)[:3]

        # Build command packet
        cmd = struct.pack("<HH", self.CMD_CONFIG_SENSOR, 8)  # Command ID and length (bytes)
        cmd += struct.pack("<B", sensor_id)
        cmd += rate_bytes
        cmd += latency_bytes

        # Send command
        self._write_register(self.REG_CHAN0_CMD, list(cmd))

        # Wait for sensor to be configured
        time.sleep(0.05)

        # Change sensor dynamic range
        if dynamic_range > 0:
            self._change_sensor_dynamic_range(sensor_id, dynamic_range)

        # Modify list of enabled sensors
        self._enabled_sensors[sensor_id] = scale
        self._stale_data_tracker[sensor_id] = 0  # Initialize stale data tracker

        # Add 0 data to sensor data list
        sample = [{"timestamp": 0.0, "x": 0.0, "y": 0.0, "z": 0.0}]
        self._sensor_data[sensor_id] = sample

    # TODO: Implement this functionality
    def _change_sensor_dynamic_range(self, sensor_id: int, dynamic_range: int) -> None:
        """
        Selects a different dynamic range for a virtual sensor
        Args:
            sensor_id (int): Sensor ID
            dynamic_range (int): dynamic measurement range for sensor
        """
        LOGGER.warning("Changing sensor dynamic range not implemented. Setting IMU to default dynamic range.")
        dynamic_range = 0  # Patch fix: resets to default
        range_bytes = struct.pack("<H1x", dynamic_range)  # Unsigned short (2 bytes), 1 null byte

        # Build command packet
        cmd = struct.pack("<HH", self.CMD_CHNG_SENSOR_DYN_RNG, 4)  # Command ID and length (bytes)
        cmd += struct.pack("<B", sensor_id)
        cmd += range_bytes

        # Write commnd
        self._write_register(self.REG_CHAN0_CMD, list(cmd))

        time.sleep(0.01)

    def _disable_sensor(self, sensor_id: int) -> None:
        """
        Disables virtual sensor by setting sample rate to 0

        Args:
            sensor_id (int): Sensor ID to disable
        """
        scale = self._enabled_sensors[sensor_id]
        self._enable_sensor(sensor_id, scale=scale, rate_hz=0, latency=0)
        del self._enabled_sensors[sensor_id]

    def enable_gyroscope(self, rate_hz: int = -1, dynamic_range: int = 2000) -> None:
        """
        Enables gyroscope
        Args:
            rate_hz (int): Sample frequency (Hz)
            range (int): Dynamic measurement range (dps)
        """
        sensor_id = self.SENSOR_DICT["Gyro"]
        allowable_ranges = self.SENSOR_RANGES[sensor_id]
        if dynamic_range not in allowable_ranges:
            raise RuntimeError(f"Error: Gyroscope measurement range not in {allowable_ranges}.")
        scale = (2 * np.pi / 360) / (32768.0 / dynamic_range)
        self._enable_sensor(sensor_id, scale=scale, dynamic_range=dynamic_range, rate_hz=rate_hz)

    def enable_accelerometer(self, rate_hz: int = -1, dynamic_range: int = 4096) -> None:
        """
        Enables accelerometer
        Args:
            rate_hz (int): Sample frequency (Hz)
            range (int): Dynamic measurement range (LSB)
        """
        sensor_id = self.SENSOR_DICT["Accel"]
        allowable_ranges = self.SENSOR_RANGES[sensor_id]
        if dynamic_range not in allowable_ranges:
            raise RuntimeError(f"Error: Accelerometer measurement range not in {allowable_ranges}.")
        scale = self.GRAVITY / dynamic_range
        self._enable_sensor(sensor_id, scale=scale, dynamic_range=dynamic_range, rate_hz=rate_hz)

        LOGGER.warning(
            "Accelerometer signal susceptible to noise due to short-term linear noise."
            "Recommended to use 'gravity' signal when trying to estimate global angles."
        )

    def enable_linear_acceleration(self, rate_hz: int = -1, dynamic_range: int = 4096) -> None:
        """
        Enables linear acceleration sensor
        Args:
            rate_hz (int): Sample frequency (Hz)
            range (int): Dynamic measurement range (LSB)
        """
        sensor_id = self.SENSOR_DICT["LinAccel"]
        allowable_ranges = self.SENSOR_RANGES[sensor_id]
        if dynamic_range not in allowable_ranges:
            raise RuntimeError(f"Error: Linear acceleration measurement range not in {allowable_ranges}.")
        scale = self.GRAVITY / dynamic_range
        self._enable_sensor(sensor_id, scale=scale, dynamic_range=dynamic_range, rate_hz=rate_hz)

    def enable_gravity(self, rate_hz: int = -1, dynamic_range: int = 4096) -> None:
        """
        Enables gravity sensor
        Args:
            rate_hz (int): Sample frequency (Hz)
            range (int): Dynamic measurement range (LSB)
        """
        sensor_id = self.SENSOR_DICT["Gravity"]
        allowable_ranges = self.SENSOR_RANGES[sensor_id]
        if dynamic_range not in allowable_ranges:
            raise RuntimeError(f"Error: Gravity measurement range not in {allowable_ranges}.")
        scale = self.GRAVITY / dynamic_range
        self._enable_sensor(sensor_id, scale=scale, dynamic_range=dynamic_range, rate_hz=rate_hz)

    def update(self) -> None:
        """
        Read data in buffer and save to class
        """
        # Read and parse FIFO
        raw_data = self._read_fifo()
        parsed_data = self._parse_fifo(raw_data)

        if not parsed_data and not self._enabled_sensors:
            LOGGER.warning("BHI260AP: No sensors enabled.")

        for sensor_id in self._enabled_sensors:
            data_samples = [
                {"x": s["x"], "y": s["y"], "z": s["z"]} for s in parsed_data if s.get("sensor_id") == sensor_id
            ]
            if not data_samples:
                self._stale_data_tracker[sensor_id] = self._stale_data_tracker[sensor_id] + 1
            else:
                self._sensor_data[sensor_id] = data_samples
                self._stale_data_tracker[sensor_id] = 0

        # Check for stale data
        for sensor_id, stale_count in self._stale_data_tracker.items():
            if stale_count > self._stale_threshold:
                sensor_name = self._SENSOR_ID_TO_NAME.get(sensor_id)
                LOGGER.warning(f"{sensor_name} data is stale! No new data for {stale_count} updates.")

    def _read_fifo(self, address: int = REG_CHAN2_NONWAKEUP_FIFO) -> bytes:
        """
        Read bytes from FIFO channel
        Args:
            address: FIFO channel
        Returns:
            (bytes): Raw FIFO data
        """
        length_bytes = self._read_register(address, 2)
        transfer_len = struct.unpack("<H", bytes(length_bytes))[0]
        if transfer_len > 0:
            fifo_data = self._read_register(address, transfer_len)
            return bytes(fifo_data)
        return b""

    def _parse_fifo(self, fifo_data: bytes) -> list[dict]:  # noqa: C901
        """
        Parse raw data from a FIFO channel
        Args:
            fifo_data (bytes): data bytes to parse
        Returns:
            (list): Parsed sensor data
        """
        if fifo_data is None:
            return []

        samples = []
        offset = 0
        current_timestamp = 0
        while offset < len(fifo_data):
            # Read sensor_id
            sensor_id = fifo_data[offset]

            # Handle padding/filler bytes
            if sensor_id == self.FIFO_EVENT_PADDING or sensor_id == self.FIFO_EVENT_FILLER:
                offset += 1
                continue

            # Handle timestamp events
            if sensor_id == self.FIFO_EVENT_SMALL_DELTA_TS:  # Small Delta Timestamp
                if offset + 1 < len(fifo_data):
                    delta = struct.unpack("B", fifo_data[offset + 1 : offset + 2])[0]
                    current_timestamp += delta
                    offset += 2
                continue

            elif sensor_id == self.FIFO_EVENT_LARGE_DELTA_TS:  # Large Delta Timestamp
                if offset + 2 < len(fifo_data):
                    delta = struct.unpack("<H", fifo_data[offset + 1 : offset + 3])[0]
                    current_timestamp += delta
                    offset += 3
                continue

            elif sensor_id == self.FIFO_EVENT_FULL_TS:  # Full Timestamp
                if offset + 5 < len(fifo_data):
                    # 40-bit timestamp (5 bytes)
                    ts_bytes = fifo_data[offset + 1 : offset + 6]
                    current_timestamp = int.from_bytes(ts_bytes, byteorder="little")
                    offset += 6
                continue

            # Handle Meta Events (non-wakeup)
            elif sensor_id == self.FIFO_EVENT_META:
                if offset + 3 < len(fifo_data):
                    fifo_data[offset + 1]
                    # (Meta events ignored)
                    offset += 4
                else:
                    break
                continue

            # Handle 3D Vector sensors (accelerometer, gyroscope) - Section 15.1.3
            # Format: 1 byte ID + 6 bytes data (3x 16-bit signed integers)
            if sensor_id in self._enabled_sensors:  # Various acc/gyro sensor IDs
                if offset + 6 < len(fifo_data):
                    sample = self._parse_sensor_data(sensor_id, fifo_data[offset : offset + 7], current_timestamp)
                    samples.append(sample)
                    offset += 7
                else:
                    break
            else:
                # Unknown sensor type, skip
                offset += 1

        return samples

    def _parse_sensor_data(self, sensor_id: int, data: bytes, current_timestamp: int) -> dict:
        """
        Parse gyro or accel data
        """
        # Unpack 3D vector
        x = struct.unpack("<h", data[1:3])[0]
        y = struct.unpack("<h", data[3:5])[0]
        z = struct.unpack("<h", data[5:7])[0]

        # Get scale factor
        scale = self._enabled_sensors[sensor_id]

        sample = {
            "sensor_id": sensor_id,
            "timestamp": current_timestamp / 64000.0,  # Convert to seconds
            "x": x * scale,
            "y": y * scale,
            "z": z * scale,
        }

        return sample

    def _get_sensor_data(self, sensor_id: int, most_recent: bool) -> np.ndarray:
        """
        Gets data for arg sensor_id

        Args:
            most_recent (bool): If True, return only the most recent (x,y,z).
                                If False, return all gyro samples as an (N,3) array.

        Returns:
            np.ndarray: Most recent (x,y,z) array (shape (3,)) or
                         all samples (shape (N,3)), or [] if no gyro data.
        """
        if not self._sensor_data:
            return np.array([])

        # collect (x,y,z) tuples for samples matching sensor_id
        data_samples = [(s["x"], s["y"], s["z"]) for s in self._sensor_data[sensor_id]]

        if not data_samples:
            return np.array([])

        if most_recent:
            return np.array(data_samples[-1], dtype=float)

        return np.array(data_samples, dtype=float)

    def verify_connection(self) -> bool:
        """Verify SPI connection by checking chip ID"""
        chip_id = self.read_chip_id()
        return chip_id in self.CHIP_ID

    def read_chip_id(self) -> int:
        """Read and return chip ID"""
        return self._read_register(self.REG_CHIP_ID, 1)[0]

    def read_boot_status_bits(self) -> np.ndarray:
        """Read and return boot status"""
        return cast(np.ndarray, self._read_register(self.REG_BOOT_STATUS, 1, True))

    def read_host_status_bits(self) -> np.ndarray:
        """Read and return host status"""
        return cast(np.ndarray, self._read_register(self.REG_HOST_STATUS, 1, True))

    def read_interrupt_status_bits(self) -> np.ndarray:
        """Read and return interrupt status"""
        return cast(np.ndarray, self._read_register(self.REG_INT_STATUS, 1, True))

    def read_error_value(self) -> int:
        """Read and return error value"""
        return self._read_register(self.REG_ERROR_VALUE, 1)[0]

    def read_rom_version(self) -> int:
        """
        Read and return ROM version (16-bit register)

        Returns:
            (int) 0x142E for BHI260AP
        """
        b = self._read_register(self.REG_ROM_VERSION, 2)
        return int.from_bytes(bytes(b), byteorder="little")

    def read_kernel_version(self) -> int:
        """
        Read and return kernel version (16-bit register)

        Returns:
            (int) build number corresponding to kernel portion of firmware,
                    if none exists, returns 0
        """
        b = self._read_register(self.REG_KERNEL_VERSION, 2)
        return int.from_bytes(bytes(b), byteorder="little")

    def read_product_id(self) -> int:
        """
        Get product ID (Fuser2 Product ID register)

        Returns:
            (int) Product ID (should be 0x89 for BHI260AP)
        """
        return self._read_register(self.REG_FUSER2_PRODUCT_ID, 1)[0]

    def read_host_interrupt_bits(self) -> np.ndarray:
        """
        Read host interrupt setting

        Returns:
            np.ndarray: Bit array of host interrupt settings
        """
        return cast(np.ndarray, self._read_register(self.REG_HOST_INT_CTRL, 1, True))

    @property
    def power_state(self) -> bool:
        """
        Returns power state
        Returns:
            0 - active
            1 - sleeping
        """
        return bool(self.read_host_status_bits()[0])

    @property
    def firmware_idle(self) -> bool:
        """
        Returns firmware state
        Returns:
            0 - Firmware running
            1 - Firmware halted
        """
        return bool(self.read_boot_status_bits()[7])

    @property
    def host_interface_ready(self) -> bool:
        """
        Returns state of host interface
        Returns:
            0 - Not ready
            1 - Ready
        """
        return bool(self.read_boot_status_bits()[4])

    @property
    def firmware_verify_done(self) -> bool:
        """
        Returns status bit for firmware verification
        Returns:
            0 - Verification not done
            1 - Verification done
        """
        return bool(self.read_boot_status_bits()[5])

    @property
    def non_wakeup_fifo_status(self) -> int:
        """
        Returns state of Non-Wakup FIFO
        Returns:
            0 - No data
            1 - Immediate (sensor with 0 latency now has data)
            2 - Latency (latency timed out for sensor with non-zero latency)
            3 - Watermark
        """
        value = 0
        bits = self.read_interrupt_status_bits()[3:5]
        for i, bit in enumerate(reversed(bits)):
            value |= bit << i
        return value

    @property
    def is_streaming(self) -> bool:
        """
        Check if the BHI260AP sensor is streaming data.

        Returns:
            bool: True if streaming; otherwise, False.
        """
        return self._is_streaming

    @property
    def data(self) -> dict:
        """Get the latest parsed sensor data packets."""
        return self._sensor_data

    @property
    def acc_x(self) -> float:
        """
        Get the estimated raw acceleration along the x-axis in m/s².

        Returns:
            float: Raw acceleration (m/s²) along the x-axis.
        """
        try:
            data = self.accel
            return float(data[0])
        except IndexError:
            LOGGER.warning("Acceleration along x-axis not available.")
            return 0.0

    @property
    def acc_y(self) -> float:
        """
        Get the estimated raw acceleration along the y-axis in m/s².

        Returns:
            float: Raw acceleration (m/s²) along the y-axis.
        """
        try:
            data = self.accel
            return float(data[1])
        except IndexError:
            LOGGER.warning("Acceleration along y-axis not available.")
            return 0.0

    @property
    def acc_z(self) -> float:
        """
        Get the estimated raw acceleration along the z-axis in m/s².

        Returns:
            float: Raw acceleration (m/s²) along the z-axis.
        """
        try:
            data = self.accel
            return float(data[2])
        except IndexError:
            LOGGER.warning("Acceleration along z-axis not available.")
            return 0.0

    @property
    def gyro_x(self) -> float:
        """
        Get the measured gyroscopic value for the x-axis.

        Returns:
            float: Angular velocity (rad/s) along the x-axis.
        """
        try:
            data = self.gyro
            return float(data[0])
        except IndexError:
            LOGGER.warning("Gyroscope value for x-axis not available.")
            return 0.0

    @property
    def gyro_y(self) -> float:
        """
        Get the measured gyroscopic value for the y-axis.

        Returns:
            float: Angular velocity (rad/s) along the y-axis.
        """
        try:
            data = self.gyro
            return float(data[1])
        except IndexError:
            LOGGER.warning("Gyroscope value for y-axis not available.")
            return 0.0

    @property
    def gyro_z(self) -> float:
        """
        Get the measured gyroscopic value for the z-axis.

        Returns:
            float: Angular velocity (rad/s) along the z-axis.
        """
        try:
            data = self.gyro
            return float(data[2])
        except IndexError:
            LOGGER.warning("Gyroscope value for z-axis not available.")
            return 0.0

    @property
    def gyro(self, most_recent: bool = True) -> np.ndarray:
        """
        Returns latest gyroscope data [gx, gy, gz] (rad/s)
        """
        sensor_id = self.SENSOR_DICT.get("Gyro", self.SENSOR_ID_GYR)
        return self._get_sensor_data(sensor_id, most_recent)

    @property
    def accel(self, most_recent: bool = True) -> np.ndarray:
        """
        Returns latest acclerometer data [ax, ay, az] (m/s^2)
        """
        sensor_id = self.SENSOR_DICT.get("Accel", self.SENSOR_ID_ACC)
        return self._get_sensor_data(sensor_id, most_recent)

    @property
    def lin_accel(self, most_recent: bool = True) -> np.ndarray:
        """
        Returns latest linear acceleration data [ax, ay, az] (m/s^2)
        """
        sensor_id = self.SENSOR_DICT.get("LinAccel", self.SENSOR_ID_LIN_ACC)
        return self._get_sensor_data(sensor_id, most_recent)

    @property
    def gravity(self, most_recent: bool = True) -> np.ndarray:
        """
        Returns latest gravity data [ax, ay, az] (m/s^2)
        """
        sensor_id = self.SENSOR_DICT.get("Gravity", self.SENSOR_ID_GRAVITY)
        return self._get_sensor_data(sensor_id, most_recent)


class AxisTransform:
    """
    Transforms IMU raw axes (x, y, z) to orientation axes (roll, pitch, yaw).

    Supports arbitrary axis mappings with sign inversions.

    Examples:
        # Standard mapping
        transform = IMUAxisTransform(roll='x', pitch='y', yaw='z')

        # Rotated IMU (90° around z-axis)
        transform = IMUAxisTransform(roll='y', pitch='-x', yaw='z')

        # Inverted pitch
        transform = IMUAxisTransform(roll='x', pitch='-y', yaw='z')

    Author:
        Katharine Walters

    Date:
        01/13/2026

    """

    VALID_AXES: ClassVar[set[str]] = {"x", "y", "z", "-x", "-y", "-z"}

    def __init__(self, tag: str = "AxisTransform", roll: str = "x", pitch: str = "y", yaw: str = "z") -> None:
        """
        Initialize axis transformation.

        Args:
            tag: identifier for axis transform instance
            roll: IMU axis for roll ('x', 'y', 'z', '-x', '-y', or '-z')
            pitch: IMU axis for pitch
            yaw: IMU axis for yaw
        """
        # Validate inputs
        for name, axis in [("roll", roll), ("pitch", pitch), ("yaw", yaw)]:
            if axis not in self.VALID_AXES:
                raise ValueError(f"{name} axis '{axis}' must be one of {self.VALID_AXES}")

        # Store mapping
        self.roll_axis = roll
        self.pitch_axis = pitch
        self.yaw_axis = yaw

        # Create axis selection functions
        self._axis_funcs = {
            "x": lambda vec: vec[0],
            "y": lambda vec: vec[1],
            "z": lambda vec: vec[2],
            "-x": lambda vec: -vec[0],
            "-y": lambda vec: -vec[1],
            "-z": lambda vec: -vec[2],
        }

    def transform_vector(self, x: float, y: float, z: float) -> tuple:
        """
        Transform a 3D vector from IMU frame to orientation frame.

        Args:
            x: x component in IMU frame
            y: y component in IMU frame
            z: z component in IMU frame

        Returns:
            tuple: (roll_component, pitch_component, yaw_component)
        """
        vec = np.array([x, y, z])

        return (
            self._axis_funcs[self.roll_axis](vec),
            self._axis_funcs[self.pitch_axis](vec),
            self._axis_funcs[self.yaw_axis](vec),
        )

    def transform_accel(self, ax: float, ay: float, az: float) -> tuple:
        """Transform acceleration vector to orientation frame."""
        return self.transform_vector(ax, ay, az)

    def transform_gyro(self, gx: float, gy: float, gz: float) -> tuple:
        """Transform angular velocity vector to orientation frame."""
        return self.transform_vector(gx, gy, gz)

    def __repr__(self) -> str:
        return f"IMUAxisTransform(roll={self.roll_axis!r}, pitch={self.pitch_axis!r}, yaw={self.yaw_axis!r})"


if __name__ == "__main__":
    # TODO: Add sample code depicting usage.
    pass
