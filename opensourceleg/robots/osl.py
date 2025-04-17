import os
import time
from typing import Optional, Union

import numpy as np

from opensourceleg.actuators.base import CONTROL_MODES, ActuatorBase
from opensourceleg.actuators.dephy import DEFAULT_CURRENT_GAINS, DEFAULT_POSITION_GAINS, DephyLegacyActuator
from opensourceleg.logging import LOGGER
from opensourceleg.robots.base import RobotBase, TActuator, TSensor
from opensourceleg.sensors.base import LoadcellBase, SensorBase
from opensourceleg.sensors.loadcell import DephyLoadcellAmplifier


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

    def home(
        self,
        homing_voltage: int = 2000,
        homing_frequency: int = 200,
        homing_direction: Optional[dict[str, int]] = None,
        output_position_offset: Optional[dict[str, float]] = None,
        current_threshold: int = 5000,
        velocity_threshold: float = 0.001,
    ) -> None:
        """
        Call the home method for all actuators.

        Args:
            homing_voltage: The voltage to apply to the actuators during homing.
            homing_frequency: The frequency to apply to the actuators during homing.
            homing_direction: The direction to apply to the actuators during homing.
                Default is -1 for knee and ankle.
            output_position_offset: The offset to apply to the actuators during homing.
                Default is 0.0 for knee and 30.0 for ankle.
            current_threshold: The current threshold to apply to the actuators during homing. Default is 5000.
            velocity_threshold: The velocity threshold to apply to the actuators during homing. Default is 0.001.
        """
        if output_position_offset is None:
            output_position_offset = {"knee": 0.0, "ankle": np.deg2rad(30.0)}
        if homing_direction is None:
            homing_direction = {"knee": -1, "ankle": -1}
        for actuator in self.actuators.values():
            actuator.home(
                homing_voltage=homing_voltage,
                homing_frequency=homing_frequency,
                homing_direction=homing_direction[actuator.tag],
                output_position_offset=output_position_offset[actuator.tag],
                current_threshold=current_threshold,
                velocity_threshold=velocity_threshold,
            )

        LOGGER.info(
            "OSL homing complete. If you'd like to create or load encoder maps to "
            "correct for nonlinearities, call `make_encoder_linearization_map()` method."
        )

    def make_encoder_linearization_map(
        self,
        overwrite: bool = False,
    ) -> None:
        """
        This method makes a lookup table to calculate the position measured by the joint encoder.
        This is necessary because the magnetic output encoders are nonlinear.
        By making the map while the joint is unloaded, joint position calculated by motor position * gear ratio
        should be the same as the true joint position. Output from this function is a file containing a_i values
        parameterizing the map.

        Eqn:
            position = sum from i=0^5 (a_i*counts^i)

        Author:
            Kevin Best (tkbest@umich.edu),
            Senthur Ayyappan (senthura@umich.edu)
        """
        for actuator_key in self.actuators:
            if f"joint_encoder_{actuator_key}" in self.sensors:
                self._create_linear_joint_mapping(
                    actuator_key=actuator_key,
                    encoder_key=f"joint_encoder_{actuator_key}",
                    overwrite=overwrite,
                )
            else:
                LOGGER.warning(
                    f"[{actuator_key}] No joint encoder found. Skipping. "
                    f"Encoder tags should be of the form 'joint_encoder_{actuator_key}'."
                )

    def _create_linear_joint_mapping(
        self,
        actuator_key: str,
        encoder_key: str,
        overwrite: bool = False,
    ) -> None:
        _actuator: ActuatorBase = self.actuators[actuator_key]
        _encoder: SensorBase = self.sensors[encoder_key]

        if not _actuator.is_homed:
            LOGGER.warning(
                msg=f"[{str.upper(_actuator.tag)}] Please home the {_actuator.tag} joint before making the encoder map."
            )
            return None

        if os.path.exists(f"./{_encoder.tag}_linearization_map.npy") and not overwrite:
            LOGGER.info(msg=f"[{str.upper(_encoder.tag)}] Encoder map exists. Skipping encoder map creation.")
            _encoder.set_encoder_map(np.load(f"./{_encoder.tag}_linearization_map.npy"))  # type: ignore[attr-defined]
            LOGGER.info(
                msg=f"[{str.upper(_encoder.tag)}] Encoder map loaded from " f"'./{_encoder.tag}_linearization_map.npy'."
            )
            return None

        _actuator.set_control_mode(mode=CONTROL_MODES.CURRENT)
        _actuator.set_current_gains(
            kp=DEFAULT_CURRENT_GAINS.kp,
            ki=DEFAULT_CURRENT_GAINS.ki,
            kd=DEFAULT_CURRENT_GAINS.kd,
            ff=DEFAULT_CURRENT_GAINS.ff,
        )

        time.sleep(0.1)

        _actuator.set_output_torque(value=0.0)

        _joint_encoder_array = []
        _output_position_array = []

        LOGGER.info(
            msg=f"[{str.upper(_actuator.tag)}] Please manually move the {_actuator.tag} joint numerous times through "
            f"its full range of motion for 10 seconds."
        )
        input("Press any key when you are ready to start.")

        _start_time: float = time.time()

        # TODO: Switch to SoftRealtimeLoop since it has reset method now
        while time.time() - _start_time < 10:
            try:
                LOGGER.info(
                    msg=f"[{str.upper(_actuator.tag)}] Mapping the {_actuator.tag} "
                    f"joint encoder: {(10 - time.time() + _start_time):.2f} seconds left."
                )
                _actuator.update()
                _encoder.update()

                _joint_encoder_array.append(_encoder.position)  # type: ignore[attr-defined]
                _output_position_array.append(_actuator.output_position)
                time.sleep(1 / _actuator.frequency)

            except KeyboardInterrupt:
                LOGGER.warning(msg="Encoder map interrupted.")
                return None

        LOGGER.info(msg=f"[{str.upper(_actuator.tag)}] You may now stop moving the {_actuator.tag} joint.")

        _power = np.arange(4.0)
        _a_mat = np.array(_joint_encoder_array).reshape(-1, 1) ** _power
        _beta = np.linalg.lstsq(_a_mat, _output_position_array, rcond=None)
        _coeffs = _beta[0]

        _encoder.set_encoder_map(np.polynomial.polynomial.Polynomial(coef=_coeffs))  # type: ignore[attr-defined]

        np.save(file=f"./{_encoder.tag}_linearization_map.npy", arr=_coeffs)

        _actuator.set_control_mode(mode=CONTROL_MODES.VOLTAGE)
        _actuator.set_motor_voltage(value=0.0)

        LOGGER.info(
            msg=f"[{str.upper(_encoder.tag)}] Encoder map saved to './{_encoder.tag}_linearization_map.npy' and loaded."
        )

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
            "loadcell": DephyLoadcellAmplifier(calibration_matrix=LOADCELL_MATRIX),
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
