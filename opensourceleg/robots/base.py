from abc import ABC, abstractmethod
from typing import Any, Generic, TypeVar

from opensourceleg.actuators.base import ActuatorBase
from opensourceleg.logging import LOGGER
from opensourceleg.sensors.base import SensorBase

TActuator = TypeVar("TActuator", bound=ActuatorBase)
TSensor = TypeVar("TSensor", bound=SensorBase)


class RobotBase(ABC, Generic[TActuator, TSensor]):
    def __init__(
        self,
        tag: str,
        actuators: dict[str, TActuator],
        sensors: dict[str, TSensor],
    ) -> None:
        self._tag = tag
        self.actuators: dict[str, TActuator] = actuators
        self.sensors: dict[str, TSensor] = sensors

    def __enter__(self) -> "RobotBase":
        self.start()
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        self.stop()

    @abstractmethod
    def start(self) -> None:
        for actuator in self.actuators.values():
            LOGGER.debug(f"Calling start method of {actuator.tag}")
            actuator.start()

        for sensor in self.sensors.values():
            LOGGER.debug(f"Calling start method of {sensor}")
            sensor.start()

    @abstractmethod
    def stop(self) -> None:
        for actuator in self.actuators.values():
            LOGGER.debug(f"Calling stop method of {actuator.tag}")
            actuator.stop()

        for sensor in self.sensors.values():
            LOGGER.debug(f"Calling stop method of {sensor}")
            sensor.stop()

    @abstractmethod
    def update(self) -> None:
        for actuator in self.actuators.values():
            actuator.update()

        for sensor in self.sensors.values():
            sensor.update()

    @property
    def tag(self) -> str:
        return self._tag

    # You can pipe values from the actuators to custom @property methods
    # to get the values from the actuators with dot notation
    # @property
    # def knee(self) -> TActuator:
    #     return self.actuators["knee"]


if __name__ == "__main__":
    pass
