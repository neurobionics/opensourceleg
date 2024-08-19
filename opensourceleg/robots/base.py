from typing import Any, Dict, Generic, TypeVar

from abc import ABC, abstractmethod

from opensourceleg.actuators.base import ActuatorBase
from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.sensors.base import SensorBase

T = TypeVar("T")


class ActuatorCollection(Generic[T]):
    def __init__(self) -> None:
        self._map: dict[str, T] = {}


class RobotBase(ABC):
    def __init__(
        self,
        tag: str,
        frequency: int,
        actuators: ActuatorCollection[DephyActuator],
        sensors: dict[str, SensorBase],
    ):
        self._tag = tag
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
        pass

    @abstractmethod
    def stop(self):
        pass

    @abstractmethod
    def update(self):
        pass

    @property
    def tag(self):
        return self._tag

    @property
    def frequency(self):
        return self._frequency

    @property
    def actuators(self):
        return self._actuators

    @property
    def sensors(self):
        return self._sensors


if __name__ == "__main__":
    pass
