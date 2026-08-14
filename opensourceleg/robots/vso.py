import os
import time
from typing import Callable, Optional, Union, cast

import numpy as np

from opensourceleg.actuators.base import CONTROL_MODES, ActuatorBase
from opensourceleg.actuators.brushed import MaxonActuator
from opensourceleg.logging import LOGGER
from opensourceleg.robots.base import RobotBase, TActuator, TSensor
from opensourceleg.sensors.base import SensorBase


class VSO(RobotBase[TActuator, TSensor]):
    """
    Variable Stiffness Orthosis (VSO) class derived from RobotBase.
    """

    def start(self) -> None:
        """Start the VSO."""
        super().start()

    def stop(self) -> None:
        """Stop the VSO."""
        super().stop()

    def update(self) -> None:
        """Update the VSO."""
        super().update()

    def home(
        self,
        homing_pwm: float = 0.25,
        sample_rate: float = 0.05,
        position_threshold: int = 200,
        home_zero: bool = True,
        timeout_s: float = 8.0,
        callbacks: Optional[dict[str, Callable]] = None,
    ) -> None:
        """
        Call the home method for all motors.

        Args:
            homing_pwm: The pwm to apply to the motors during homing.
            sample_rate: The time slept between each move during homing.
            position_threshold: Minimum error between the last encoder movement and now to stop homing.
            Default is 200 for the motor but may want to change if flex coupler attached.
            home_zero: Determines the direction of homing. Should be True to move to 0% stiffness and
                false to move to 100% stiffness.
            callbacks  Optional[dict[str, Callable]]:
                Optional dictionary of callback functions, one per motor, to be called when each motor's
                homing completes. Only one callback per motor is supported, and the tag must match.
                Each function should take no arguments and return None. If None, no callbacks are used.
        """

        LOGGER.info("Starting VSO homing routine.")

        for actuator in self.actuators.values():
            maxon = cast(MaxonActuator, actuator)
            callback = callbacks.get(actuator.tag, None) if callbacks is not None else None
            maxon.home(
                homing_pwm=homing_pwm,
                sample_rate=sample_rate,
                position_threshold=position_threshold,
                home_zero=home_zero,
                callback=callback,
                timeout_s=timeout_s,
            )

        LOGGER.info("VSO homing complete. Spring-support is at hard stop.")

    def make_encoder_linearization_map(
        self,
        overwrite: bool = False,
    ) -> None:
        """
        This method makes a lookup table to calculate the position measured by the joint encoder.

        This method is necessary because the magnetic output encoders are nonlinear. By making the map while
        the joint is unloaded, joint position calculated by motor position * gear ratio should be the same as
        the true joint position. Output from this function is a file containing a_i values parameterizing the map.

        Eqn:
            position = sum from i=0^5 (a_i*counts^i)

        Args:
            overwrite: If True, regenerate and overwrite any existing linearization map file.
                If False, load the existing file when one is found.
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
        """
        Fit and save a polynomial linearization map for a single joint encoder.

        Moves the actuator into position-control mode, prompts the user to manually sweep the joint through its
        full range of motion for 10 seconds, then solves a least-squares polynomial fit between raw encoder
        counts and motor-derived output position. The resulting coefficients are saved to
        ``<encoder_tag>_linearization_map.npy``.

        Args:
            actuator_key: Key for self.actuators, identifying the joint's actuator.
            encoder_key: Key for self.sensors identifying the joint's output encoder.
            overwrite: If True, regenerate the map even when a saved file already exists.
        """
        _actuator = cast(MaxonActuator, self.actuators[actuator_key])
        _encoder: SensorBase = self.sensors[encoder_key]

        if not _actuator.is_homed:
            LOGGER.warning(
                msg=f"[{str.upper(_actuator.tag)}] Please home the {_actuator.tag} joint before making the encoder map."
            )
            return None

        if os.path.exists(f"./{_encoder.tag}_linearization_map.npy") and not overwrite:
            LOGGER.info(msg=f"[{str.upper(_encoder.tag)}] Encoder map exists. Skipping encoder map creation.")
            _encoder.set_encoder_map(  # type: ignore[attr-defined]
                np.polynomial.polynomial.Polynomial(np.load(f"./{_encoder.tag}_linearization_map.npy"))
            )
            LOGGER.info(
                msg=f"[{str.upper(_encoder.tag)}] Encoder map loaded from './{_encoder.tag}_linearization_map.npy'."
            )
            return None

        _actuator.set_control_mode(mode=CONTROL_MODES.POSITION)
        _actuator.set_position_gains(0.015, 2, 0.001, 0.0)  # default value for position gain

        time.sleep(0.1)

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

        LOGGER.info(
            msg=f"[{str.upper(_encoder.tag)}] Encoder map saved to './{_encoder.tag}_linearization_map.npy' and loaded."
        )

    @property
    def ankle(self) -> Union[TActuator, ActuatorBase]:
        """
        Get the ankle actuator.

        Returns:
            Union[TActuator, ActuatorBase]: The ankle actuator.

        Raises:
            KeyError: If no actuator with tag ``ankle`` is registered.
        """
        try:
            return self.actuators["ankle"]
        except KeyError:
            LOGGER.error("Ankle actuator not found. Please check for `ankle` key in the actuators dictionary.")
            exit(1)

    @property
    def joint_encoder_ankle(self) -> Union[TSensor, SensorBase]:
        """
        Get the ankle joint encoder sensor.

        Returns:
            Union[TSensor, SensorBase]: The ankle joint encoder sensor.

        Raises:
            KeyError: If no sensor with tag ``joint_encoder_ankle`` is registered.
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

    vso = VSO[MaxonActuator, SensorBase](
        tag="VariableStiffnessOrthosis",
        actuators={
            "ankle": MaxonActuator(tag="ankle", offline=False, frequency=frequency),
        },
        sensors=dict[str, SensorBase](),
    )

    with vso:
        vso.update()

        while True:
            try:
                vso.update()
                time.sleep(1 / frequency)

            except KeyboardInterrupt:
                exit()
