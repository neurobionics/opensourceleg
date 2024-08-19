from typing import Any, Dict, Generic, List, NamedTuple, TypeVar

from opensourceleg.actuators.base import ActuatorBase
from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.actuators.moteus import MoteusActuator
from opensourceleg.robots.base import RobotBase
from opensourceleg.sensors.base import SensorBase

T = TypeVar("T")


class OpenSourceLeg(RobotBase):
    def __init__(
        self,
        tag: str,
        frequency: int,
        actuators: dict[str, Any],
        sensors: dict[str, Any],
    ):
        super().__init__(tag, frequency, actuators, sensors)

    def start(self):
        for actuator in self.actuators.values():
            actuator.start()

        for sensor in self.sensors.values():
            sensor.start()

    def stop(self):
        for actuator in self.actuators.values():
            actuator.stop()

        for sensor in self.sensors.values():
            sensor.stop()

    def update(self):
        for actuator in self.actuators.values():
            actuator.update()

        for sensor in self.sensors.values():
            sensor.update()


if __name__ == "__main__":
    actuators_type: T = ActuatorBase

    actuators_dict = {
        "knee": DephyActuator("knee"),
        "ankle": MoteusActuator("ankle"),
    }

    actuators_collection = NamedTuple(
        "ActuatorCollection",
    )
