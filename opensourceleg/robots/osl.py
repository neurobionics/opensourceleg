#!/usr/bin/python3
from types import SimpleNamespace
from typing import NamedTuple, TypedDict

import sys
import time
from collections import namedtuple

import numpy as np

from opensourceleg.actuators.dephy import ActuatorBase, DephyActpack
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
        actuators={},
    ) -> None:
        """
        Initialize the OSL class.

        Parameters
        ----------
        frequency : int, optional
            The frequency of the control loop, by default 200
        """

        self._frequency: int = frequency

        # create a named tuple that is type defined for the actuators
        fields = [
            (key, float) for key in actuators.keys()
        ]  # Assuming all actuators are of type float
        # Dynamically create the NamedTuple class
        _actuators = NamedTuple("Actuators", fields)
        self._actuators = _actuators(**actuators)

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

    def update(self):
        return super().update()

    def __repr__(self) -> str:
        return f"OSL"

    def home(self) -> None:
        pass

    def reset(self) -> None:
        pass

    @property
    def is_homed(self) -> bool:
        return self._is_homed


if __name__ == "__main__":

    # # Type defined dictionary for the actuators
    # old_actuators: dict[str, ActuatorBase] = {
    #     "knee": DephyActpack(offline=True),
    #     "ankle": DephyActpack(offline=True),
    # }
    # print(old_actuators["knee"])

    # # Define the Actuators namedtuple with type hints
    # class Actuators(NamedTuple):
    #     knee: DephyActpack
    #     ankle: DephyActpack

    # actuators = Actuators(
    #     knee=DephyActpack(offline=True), ankle=DephyActpack(offline=True)
    # )

    # print(actuators.knee.tag, old_actuators["knee"].gear_ratio)
    pass
