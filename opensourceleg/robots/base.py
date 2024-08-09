from typing import NamedTuple

from abc import ABC, abstractmethod

from opensourceleg.actuators.base import ActuatorBase
from opensourceleg.sensors.base import SensorBase


class RobotBase(ABC):
    """
    Base class for all robots. They encapsulate all the hardware classes and provide a common interface to control them.

    Args:
        ABC (_type_): _description_
    """

    def __init__(
        self,
        frequency: int = 200,
        actuators: dict[str, ActuatorBase] = {},
        sensors: dict[str, SensorBase] = {},
    ):
        self._frequency = frequency
        self._actuators = actuators
        self._sensors = sensors

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    @abstractmethod
    def start(self):
        for actuator in self.actuators.items():
            actuator.start()

        for sensor in self.sensors.items():
            sensor.start()

    @abstractmethod
    def stop(self):
        for actuator in self.actuators.items():
            actuator.stop()

        for sensor in self.sensors.items():
            sensor.stop()

    # TODO: Should the actuator_tag come from the actuator object itself?
    def add_actuator(self, actuator_tag: str, actuator: ActuatorBase):
        self._actuators[actuator_tag] = actuator

    # TODO: Should the sensor tag come from the sensor object itself?
    def add_sensor(self, sensor_tag: str, sensor: SensorBase):
        self._sensors[sensor_tag] = sensor

    @abstractmethod
    def update(self):
        for sensor in self.sensors.items():
            sensor.update()

        for actuator in self.actuators.items():
            actuator.update()

    @abstractmethod
    def home(self):
        pass

    @abstractmethod
    def reset(self):
        pass

    @property
    def frequency(self):
        return self._frequency

    @property
    def actuators(self):
        return self._actuators

    @property
    def sensors(self):
        return self._sensors
