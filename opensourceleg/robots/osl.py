import time
from typing import Union

import numpy as np

from opensourceleg.actuators.base import CONTROL_MODES, ActuatorBase
from opensourceleg.actuators.dephy import DEFAULT_POSITION_GAINS, DephyLegacyActuator
from opensourceleg.logging import LOGGER
from opensourceleg.robots.base import RobotBase, TActuator, TSensor
from opensourceleg.sensors.base import LoadcellBase, SensorBase
from opensourceleg.sensors.loadcell import SRILoadcell


class OpenSourceLeg(RobotBase[TActuator, TSensor]):
    """
    Open Source Leg (OSL) class derived from RobotBase.
    """

    def start(self) -> None:
        """
        Start the OSL.
        """
        super().start()

    def stop(self) -> None:
        """
        Stop the OSL.
        """
        super().stop()

    def update(self) -> None:
        """
        Update the robot.
        """
        super().update()

    def home(self) -> None:
        """
        Call the home method for all actuators.
        """
        for actuator in self.actuators.values():
            actuator.home()

    def make_encoder_maps(self) -> None:
        """
        Make encoder maps for all actuators.
        """
        for actuator in self.actuators.values():
            if hasattr(actuator, "make_encoder_map"):
                actuator.make_encoder_map()
            else:
                LOGGER.warning(f"[{actuator.__repr__()}] No encoder map method found. Skipping.")

    @property
    def knee(self) -> Union[TActuator, ActuatorBase]:
        """
        Get the knee actuator.

        Returns:
            Union[TActuator, ActuatorBase]: The knee actuator.
        """
        try:
            return self.actuators["knee"]
        except KeyError:
            LOGGER.error("Knee actuator not found. Please check for `knee` key in the actuators dictionary.")
            exit(1)

    @property
    def ankle(self) -> Union[TActuator, ActuatorBase]:
        """
        Get the ankle actuator.

        Returns:
            Union[TActuator, ActuatorBase]: The ankle actuator.
        """
        try:
            return self.actuators["ankle"]
        except KeyError:
            LOGGER.error("Ankle actuator not found. Please check for `ankle` key in the actuators dictionary.")
            exit(1)

    @property
    def loadcell(self) -> Union[TSensor, LoadcellBase]:
        """
        Get the loadcell sensor.

        Returns:
            Union[TSensor, LoadcellBase]: The loadcell sensor.
        """
        try:
            return self.sensors["loadcell"]
        except KeyError:
            LOGGER.error("Loadcell sensor not found. Please check for `loadcell` key in the sensors dictionary.")
            exit(1)

    @property
    def joint_encoder_knee(self) -> Union[TSensor, SensorBase]:
        """
        Get the knee joint encoder sensor.

        Returns:
            Union[TSensor, SensorBase]: The knee joint encoder sensor.
        """
        try:
            return self.sensors["joint_encoder_knee"]
        except KeyError:
            LOGGER.error(
                "Knee joint encoder sensor not found."
                "Please check for `joint_encoder_knee` key in the sensors dictionary."
            )
            exit(1)

    @property
    def joint_encoder_ankle(self) -> Union[TSensor, SensorBase]:
        """
        Get the ankle joint encoder sensor.

        Returns:
            Union[TSensor, SensorBase]: The ankle joint encoder sensor.
        """
        try:
            return self.sensors["joint_encoder_ankle"]
        except KeyError:
            LOGGER.error(
                "Ankle joint encoder sensor not found."
                "Please check for `joint_encoder_ankle` key in the sensors dictionary."
            )
            exit(1)


if __name__ == "__main__":
    frequency = 200

    LOADCELL_MATRIX = np.array([
        (-5.45598, -1317.68604, 27.14505, 22.8468, -11.1176, 1283.02856),
        (-20.23942, 773.01343, -9.44841, -1546.70923, 21.78232, 744.52325),
        (-811.76398, -16.82792, -825.67261, 2.93904, -829.06409, 7.45233),
        (16.38306, 0.22658, -0.50331, -0.23233, -17.0822, -0.03632),
        (-9.81471, -0.03671, 19.47362, -0.161, -9.76819, 0.25571),
        (-0.51744, -20.6571, 0.18245, -20.42393, 0.01944, -20.38067),
    ])

    osl = OpenSourceLeg[DephyLegacyActuator, SensorBase](
        tag="opensourceleg",
        actuators={
            "knee": DephyLegacyActuator("knee", offline=False, frequency=frequency, gear_ratio=9 * (83 / 18)),
        },
        sensors={
            "loadcell": SRILoadcell(calibration_matrix=LOADCELL_MATRIX),
        },
    )

    with osl:
        osl.update()

        osl.knee.set_control_mode(CONTROL_MODES.POSITION)
        osl.knee.set_position_gains(
            kp=DEFAULT_POSITION_GAINS.kp,
            ki=DEFAULT_POSITION_GAINS.ki,
            kd=DEFAULT_POSITION_GAINS.kd,
            ff=DEFAULT_POSITION_GAINS.ff,
        )
        osl.knee.set_output_position(osl.knee.output_position + np.deg2rad(10))
        # osl.loadcell.calibrate()

        while True:
            try:
                osl.update()
                # print(osl.sensors["loadcell"].fz)
                time.sleep(1 / frequency)

            except KeyboardInterrupt:
                exit()
