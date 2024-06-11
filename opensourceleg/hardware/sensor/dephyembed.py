from typing import Any

import numpy as np
from flexsea.device import Device

import opensourceleg.hardware.sensor.base as base


class DephySensor(Device, base.SensorIMU):

    def __init__(self, port, baud_rate):
        Device.__init__(self, port=port, baud_rate=baud_rate)

    def start_streaming(self):
        self.is_streaming = True

    def stop_streaming(self):
        self.is_streaming = False

    def get_data(self) -> Any:
        return self.read()
