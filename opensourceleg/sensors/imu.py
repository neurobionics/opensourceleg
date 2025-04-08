"""
Module for interfacing with IMU sensors using the MSCL and Adafruit libraries.

This module provides two IMU sensor implementations:
  - LordMicrostrainIMU: Uses the MSCL library to interface with a Lord Microstrain IMU.
  - BNO055: Uses the Adafruit BNO055 library to interface with a Bosch BNO055 IMU.

Dependencies:
  - MSCL (for LordMicrostrainIMU): https://github.com/LORD-MicroStrain/MSCL/tree/master
  - adafruit_bno055, board, busio (for BNO055)

Ensure that the required libraries are installed and that the library paths are added
to PYTHONPATH or sys.path if necessary.
"""

import os
from typing import Any, Union

from opensourceleg.logging import LOGGER
from opensourceleg.sensors.base import IMUBase, check_sensor_stream


class LordMicrostrainIMU(IMUBase):
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
            import sys

            sys.path.append("/usr/share/python3-mscl")

            import mscl

            self.mscl = mscl
        except ImportError:
            LOGGER.warning(
                "Failed to import mscl. Please install the MSCL library from Lord Microstrain and append the path "
                "to the PYTHONPATH or sys.path. Checkout https://github.com/LORD-MicroStrain/MSCL/tree/master "
                "and https://lord-microstrain.github.io/MSCL/Documentation/MSCL%20API%20Documentation/index.html"
            )
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
            LOGGER.error("BNO055IMU requires adafruit_bno055, board, and busio packages. " f"Error: {e}")
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
            print("BNO055 IMU Not Found on i2c bus! Check wiring!")
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


if __name__ == "__main__":
    # TODO: Add sample code depicting usage.
    pass
