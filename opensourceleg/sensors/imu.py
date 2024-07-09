from typing import Union

import os
import time
from dataclasses import dataclass

from opensourceleg.logging.logger import LOGGER


@dataclass
class IMUDataClass:
    """
    Dataclass for IMU data.
    Data is returned in the IMU frame.
    Angles are in rad.
    Velocities are in rad/s.
    Acceleration is in g.

    Author: Kevin Best
    https://github.com/tkevinbest
    """

    angle_x: float = 0
    """x direction Euler angle in rad"""
    angle_y: float = 0
    """y direction Euler angle in rad"""
    angle_z: float = 0
    """z direction Euler angle in rad"""
    velocity_x: float = 0
    """x direction rotational velocity in rad/s"""
    velocity_y: float = 0
    """y direction rotational velocity in rad/s"""
    velocity_z: float = 0
    """z direction rotational velocity in rad/s"""
    accel_x: float = 0
    """x direction acceleration in g"""
    accel_y: float = 0
    """y direction acceleration in g"""
    accel_z: float = 0
    """z direction acceleration in g"""
    imu_time_sta: float = 0
    imu_filter_gps_time_week_num: float = 0


class IMULordMicrostrain:
    """
    Sensor class for the Lord Microstrain IMU.
    Requires the MSCL library from Lord Microstrain (see below for install instructions).

    As configured, this class returns euler angles (rad), angular rates (rad/s), and accelerations (g).

    Example:
        imu = IMULordMicrostrain()
        imu.start_streaming()
        while in loop:
            imu.get_data()
        imu.stop_streaming()

    Resources:
        * To install, download the pre-built package for raspian at https://github.com/LORD-MicroStrain/MSCL/tree/master
        * Full documentation for their library can be found at https://lord-microstrain.github.io/MSCL/Documentation/MSCL%20API%20Documentation/index.html.
    """

    def __init__(
        self, port=r"/dev/ttyUSB0", baud_rate=921600, timeout=500, sample_rate=100
    ):
        import sys

        sys.path.append(r"/usr/share/python3-mscl/")
        import mscl as ms

        self.port = port
        self.baud_rate = baud_rate
        self.connection = ms.Connection.Serial(
            os.path.realpath(self.port), self.baud_rate
        )
        self.imu = ms.InertialNode(self.connection)
        self.timeout = timeout  # Timeout in (ms) to read the IMU
        time.sleep(0.5)

        # Configure data channels
        channels = ms.MipChannels()
        channels.append(
            ms.MipChannel(
                ms.MipTypes.CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_EULER,
                ms.SampleRate.Hertz(sample_rate),
            )
        )
        channels.append(
            ms.MipChannel(
                ms.MipTypes.CH_FIELD_ESTFILTER_ESTIMATED_ANGULAR_RATE,
                ms.SampleRate.Hertz(sample_rate),
            )
        )
        channels.append(
            ms.MipChannel(
                ms.MipTypes.CH_FIELD_ESTFILTER_ESTIMATED_LINEAR_ACCEL,
                ms.SampleRate.Hertz(sample_rate),
            )
        )
        channels.append(
            ms.MipChannel(
                ms.MipTypes.CH_FIELD_ESTFILTER_GPS_TIMESTAMP,
                ms.SampleRate.Hertz(sample_rate),
            )
        )

        self.imu.setActiveChannelFields(ms.MipTypes.CLASS_ESTFILTER, channels)
        self.imu.enableDataStream(ms.MipTypes.CLASS_ESTFILTER)
        self.imu.setToIdle()

        packets = self.imu.getDataPackets(
            self.timeout
        )  # Clean the internal circular buffer.
        self.imu_data = IMUDataClass()

    def start_streaming(self):
        self.imu.resume()

    def stop_streaming(self):
        self.imu.setToIdle()

    def get_data(self):
        """
        Get IMU data from the Lord Microstrain IMU
        """
        imu_packets = self.imu.getDataPackets(self.timeout)
        if len(imu_packets):
            # Read all the information from the first packet as float.
            raw_imu_data = {
                data_point.channelName(): data_point.as_float()
                for data_point in imu_packets[-1].data()
            }
            self.imu_data.angle_x = raw_imu_data["estRoll"]
            self.imu_data.angle_y = raw_imu_data["estPitch"]
            self.imu_data.angle_z = raw_imu_data["estYaw"]
            self.imu_data.velocity_x = raw_imu_data["estAngularRateX"]
            self.imu_data.velocity_y = raw_imu_data["estAngularRateY"]
            self.imu_data.velocity_z = raw_imu_data["estAngularRateZ"]
            self.imu_data.accel_x = raw_imu_data["estLinearAccelX"]
            self.imu_data.accel_y = raw_imu_data["estLinearAccelY"]
            self.imu_data.accel_z = raw_imu_data["estLinearAccelZ"]
            self.imu_data.imu_time_sta = raw_imu_data["estFilterGpsTimeTow"]
            self.imu_data.imu_filter_gps_time_week_num = raw_imu_data[
                "estFilterGpsTimeWeekNum"
            ]

        return self.imu_data

    def __repr__(self) -> str:
        return f"IMULordMicrostrain"


if __name__ == "__main__":
    pass
