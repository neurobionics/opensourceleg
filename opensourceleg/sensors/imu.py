from typing import Union

import os
import time
from dataclasses import dataclass

from opensourceleg.logging import LOGGER
from opensourceleg.sensors.base import IMUBase, check_sensor_stream

try:
    import mscl
except ImportError:
    LOGGER.error(
        "Failed to import mscl. Please install the MSCL library from Lord Microstrain and append the path to the PYTHONPATH or sys.path. Checkout https://github.com/LORD-MicroStrain/MSCL/tree/master and https://lord-microstrain.github.io/MSCL/Documentation/MSCL%20API%20Documentation/index.html"
    )


class LordMicrostrainIMU(IMUBase):
    """
    Sensor class for the Lord Microstrain IMU.
    Requires the MSCL library from Lord Microstrain (see below for install instructions).

    As configured, this class returns euler angles (rad), angular rates (rad/s), and accelerations (g).


    Resources:
        * To install, download the pre-built package for raspian at https://github.com/LORD-MicroStrain/MSCL/tree/master
        * Full documentation for their library can be found at https://lord-microstrain.github.io/MSCL/Documentation/MSCL%20API%20Documentation/index.html.
    """

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baud_rate: int = 921600,
        frequency: int = 200,
    ):

        self._port = port
        self._baud_rate = baud_rate
        self._frequency = frequency
        self._is_streaming = False
        self._connection = None
        self._data = None

    def _configure_mip_channels(self):
        channels = mscl.MipChannels()
        channels.append(
            mscl.MipChannel(
                mscl.MipTypes.CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_EULER,
                mscl.SampleRate.Hertz(self.frequency),
            )
        )
        channels.append(
            mscl.MipChannel(
                mscl.MipTypes.CH_FIELD_ESTFILTER_ESTIMATED_ANGULAR_RATE,
                mscl.SampleRate.Hertz(self.frequency),
            )
        )
        channels.append(
            mscl.MipChannel(
                mscl.MipTypes.CH_FIELD_ESTFILTER_ESTIMATED_LINEAR_ACCEL,
                mscl.SampleRate.Hertz(self.frequency),
            )
        )
        channels.append(
            mscl.MipChannel(
                mscl.MipTypes.CH_FIELD_ESTFILTER_GPS_TIMESTAMP,
                mscl.SampleRate.Hertz(self.frequency),
            )
        )

        return channels

    def start(self):

        self._connection = mscl.Connection.Serial(self.port, self.baud_rate)
        self._node = mscl.InertialNode(self._connection)
        self._node.setActiveChannelFields(
            mscl.MipTypes.CLASS_ESTFILTER, self._configure_channels()
        )
        self._node.enableDataStream(mscl.MipTypes.CLASS_ESTFILTER)
        self._is_streaming = True

    @check_sensor_stream
    def stop(self):
        self._node.setToIdle()
        self._is_streaming = False

    @check_sensor_stream
    def ping(self):
        response = self._node.ping()

        if response.success():
            LOGGER.info(f"Successfully pinged the IMU at {self.port}")
        else:
            LOGGER.error(f"Failed to ping the IMU at {self.port}")

    def update(
        self, timeout: int = 500, max_packets: int = 1, return_packets: bool = False
    ):
        """
        Get IMU data from the Lord Microstrain IMU
        """
        data_packets = self._node.getDataPackets(
            timeout=timeout, maxPackets=max_packets
        )
        data_points = data_packets[-1].data()
        self._data = {data.channelName(): data.as_float() for data in data_points}

        if return_packets:
            return data_packets

    def __repr__(self) -> str:
        return f"IMULordMicrostrain"

    @property
    def port(self) -> str:
        return self._port

    @property
    def baud_rate(self) -> int:
        return self._baud_rate

    @property
    def frequency(self) -> int:
        return self._frequency

    @property
    def is_streaming(self) -> bool:
        return self._is_streaming

    @property
    def roll(self) -> float:
        """Returns estimated roll (rad)."""
        return self._data["estRoll"]

    @property
    def pitch(self) -> float:
        """Returns estimated pitch (rad)."""
        return self._data["estPitch"]

    @property
    def yaw(self) -> float:
        """Returns estimated yaw (rad)."""
        return self._data["estYaw"]

    @property
    def vel_x(self) -> float:
        """Returns estimated angular velocity about the x-axis (rad/s)."""
        return self._data["estAngularRateX"]

    @property
    def vel_y(self) -> float:
        """Returns estimated angular velocity about the y-axis (rad/s)."""
        return self._data["estAngularRateY"]

    @property
    def vel_z(self) -> float:
        """Returns estimated angular velocity about the z-axis (rad/s)."""
        return self._data["estAngularRateZ"]

    @property
    def acc_x(self) -> float:
        """Returns estimated linear acceleration along the x-axis (m/s^2)."""
        return self._data["estLinearAccelX"]

    @property
    def acc_y(self) -> float:
        """Returns estimated linear acceleration along the y-axis (m/s^2)."""
        return self._data["estLinearAccelY"]

    @property
    def acc_z(self) -> float:
        """Returns estimated linear acceleration along the z-axis (m/s^2)."""
        return self._data["estLinearAccelZ"]

    @property
    def timestamp(self) -> float:
        """Returns timestamp (s) of the data."""
        return self._data["estFilterGpsTimeTow"]


if __name__ == "__main__":
    pass
