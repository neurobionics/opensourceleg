from typing import Any

import numpy as np

import opensourceleg.hardware.sensor.base as base

# from flexsea.device import Device


class DephySensor(base.SensorIMU):

    def __init__(self, device: "DephyActpack"):
        self.device = device
        self.data = None

    def start_streaming(self):
        print("Please ensure that the actpack is connected and streaming.")
        pass

    def stop_streaming(self):
        self.is_streaming = False

    def update(self, data: Any = None):
        if data is not None:
            self.data = data

        elif self.device.is_streaming:
            self.data = self.device.read()


class DephyIMU(base.SensorIMU):
    def __init__():
        pass

    def start_streaming(self):
        pass

    def stop_streaming(self):
        pass


class DephyJointEncoder(base.Encoder):
    pass


class DephyActpack:
    def __init__(self, port, baud_rate, **kwargs):
        super().__init__(port, baud_rate, **kwargs)
        self.imu = DephyIMU(self)

    def update(self):
        self.data = self.read()
        self.imu.update(self.data)
        self.joint_encoder.update(self.data)

    @property
    def data(self):
        return self.data


if __name__ == "__main__":
    dephy_actpack = DephyActpack(port="/dev/ttyUSB0", baud_rate=115200)
    my_dephy_imu = DephyIMU(dephy_actpack)
