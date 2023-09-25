import os
import sys
from dataclasses import dataclass
from time import sleep

import mscl as ms
import numpy as np


@dataclass
class IMUDataClass:
    """
    Dataclass for IMU data

    Author: Kevin Best
    https://github.com/tkevinbest
    """

    angle_x: float = 0
    angle_y: float = 0
    angle_z: float = 0
    velocity_x: float = 0
    velocity_y: float = 0
    velocity_z: float = 0
    accel_x: float = 0
    accel_y: float = 0
    accel_z: float = 0
    imu_time_sta: float = 0
    imu_filter_gps_time_week_num: float = 0
    thigh_angle_sagi: float = 0
    thigh_angle_coro: float = 0
    thigh_angle_trans: float = 0


class IMULordMicrostrain:
    def __init__(
        self, port=r"/dev/ttyUSB0", baud_rate=921600, timeout=500, sample_rate=100
    ):
        self.port = port
        self.baud_rate = baud_rate
        self.connection = ms.Connection.Serial(
            os.path.realpath(self.port), self.baud_rate
        )
        self.imu = ms.InertialNode(self.connection)
        self.timeout = timeout  # Timeout in (ms) to read the IMU
        sleep(0.5)

        # Configure data channels
        channels = ms.MipChannels()
        channels.append(
            ms.MipChannel(
                ms.MipTypes.CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_EULER,
                ms.SampleRate.Hertz(sampleRate),
            )
        )
        channels.append(
            ms.MipChannel(
                ms.MipTypes.CH_FIELD_ESTFILTER_ESTIMATED_ANGULAR_RATE,
                ms.SampleRate.Hertz(sampleRate),
            )
        )
        channels.append(
            ms.MipChannel(
                ms.MipTypes.CH_FIELD_ESTFILTER_ESTIMATED_LINEAR_ACCEL,
                ms.SampleRate.Hertz(sampleRate),
            )
        )
        channels.append(
            ms.MipChannel(
                ms.MipTypes.CH_FIELD_ESTFILTER_GPS_TIMESTAMP,
                ms.SampleRate.Hertz(sampleRate),
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
            self.imu_data.thigh_angle_sagi = self.imu_data.angle_x + np.deg2rad(39.38)
            self.imu_data.thigh_angle_coro = self.imu_data.angle_y
            self.imu_data.thigh_angle_trans = self.imu_data.angle_z

        return self.imu_data


if __name__ == "__main__":
    imu = IMULordMicrostrain(r"/dev/ttyS0", timeout=0, sample_rate=100)
    imu.start_streaming()

    for i in range(500):
        imu_data = imu.get_data()
        print(imu_data.thigh_angle_sagi)
        sleep(0.01)
