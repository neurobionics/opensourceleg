from typing import Union

from opensourceleg.actuators.base import ActuatorBase
from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.actuators.moteus import MoteusActuator
from opensourceleg.logging import LOGGER
from opensourceleg.robots.base import RobotBase, TActuator, TSensor
from opensourceleg.sensors.base import IMUBase, LoadcellBase, SensorBase


class OpenSourceLeg(RobotBase[TActuator, TSensor]):
    def start(self):
        super().start()

    def stop(self):
        super().stop()

    def update(self):
        super().update()

    def home(self):
        for actuator in self.actuators.values():
            actuator.home()

    def make_encoder_maps(self):
        pass

    @property
    def knee(self) -> Union[TActuator, ActuatorBase]:
        try:
            return self.actuators["knee"]
        except KeyError:
            LOGGER.error(
                "Knee actuator not found. Please check for `knee` key in the actuators dictionary."
            )
            exit(1)

    @property
    def ankle(self) -> Union[TActuator, ActuatorBase]:
        try:
            return self.actuators["ankle"]
        except KeyError:
            LOGGER.error(
                "Ankle actuator not found. Please check for `ankle` key in the actuators dictionary."
            )
            exit(1)

    @property
    def loadcell(self) -> Union[TSensor, LoadcellBase]:
        try:
            return self.sensors["loadcell"]
        except KeyError:
            LOGGER.error(
                "Loadcell sensor not found. Please check for `loadcell` key in the sensors dictionary."
            )
            exit(1)

    @property
    def joint_encoder_knee(self) -> Union[TSensor, SensorBase]:
        try:
            return self.sensors["joint_encoder_knee"]
        except KeyError:
            LOGGER.error(
                "Knee joint encoder sensor not found. Please check for `joint_encoder_knee` key in the sensors dictionary."
            )
            exit(1)

    @property
    def joint_encoder_ankle(self) -> Union[TSensor, SensorBase]:
        try:
            return self.sensors["joint_encoder_ankle"]
        except KeyError:
            LOGGER.error(
                "Ankle joint encoder sensor not found. Please check for `joint_encoder_ankle` key in the sensors dictionary."
            )
            exit(1)


if __name__ == "__main__":
    osl = OpenSourceLeg[DephyActuator, SensorBase](
        tag="opensourceleg",
        actuators={
            "knee": DephyActuator("knee", offline=True),
            "ankle": MoteusActuator("ankle", offline=True),
        },
        sensors={},
    )
    print(osl.actuators["knee"])
    print(osl.knee.tag)
