#!/usr/bin/python3
import sys
import time

import numpy as np

from opensourceleg.actuators.base import ActuatorBase
from opensourceleg.logging import LOGGER
from opensourceleg.robots.base import RobotBase
from opensourceleg.sensors.base import SensorBase


class OpenSourceLeg(RobotBase):
    """
    OSL class: This class is the main class for the Open Source Leg project. It
    contains all the necessary functions to control the leg.

    Returns:
        none: none
    """

    _instance = None

    @staticmethod
    def get_instance():
        if OpenSourceLeg._instance is None:
            OpenSourceLeg()
        return OpenSourceLeg._instance

    def __init__(
        self,
        frequency: int = 200,
        actuators: dict[str, ActuatorBase] = {},
        sensors: dict[str, SensorBase] = {},
    ) -> None:
        """
        Initialize the OSL class.

        Parameters
        ----------
        frequency : int, optional
            The frequency of the control loop, by default 200
        """

        self._frequency: int = frequency
        self._actuators: dict[str, ActuatorBase] = actuators
        self._sensors: dict[str, SensorBase] = sensors

    def start(self) -> None:
        """
        Start the Open-Source Leg
        """
        super().start()

    def stop(self) -> None:
        """
        Stop the Open-Source Leg
        """
        super().stop()

    def __repr__(self) -> str:
        return f"OSL"

    def home(self) -> None:
        pass

    def reset(self) -> None:
        pass

    @property
    def knee(self):
        pass

    @property
    def ankle(self):
        pass

    @property
    def loadcell(self):
        pass

    @property
    def imu(self):
        pass

    @property
    def is_homed(self) -> bool:
        return self._is_homed


if __name__ == "__main__":
    osl = OpenSourceLeg(frequency=200)
    osl.add_knee()
    osl.add_ankle()

    osl.add_loadcell()
    osl.add_imu()
