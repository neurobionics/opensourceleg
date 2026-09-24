import time

import numpy as np

from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.actuators.brushed import MaxonActuator
from opensourceleg.logging.logger import Logger
from opensourceleg.sensors.encoderCounter import LS7366R

TIME_TO_STEP = 1.0
FREQUENCY = 200
DT = 1 / FREQUENCY

TARGET_PERC = [30.0]  # Target stiffness position in percentage. Edit here to change target position.


def _clamp_target_cts(actuator: MaxonActuator, desired_cts: int) -> int:
    """Clamp a target position (counts) into the safe slider range."""
    desired_mm = actuator.cts_to_mm(desired_cts)
    if desired_mm > actuator.slider_max_mm:
        return actuator.slider_max_counts
    if desired_mm < actuator.slider_min_mm:
        return actuator.slider_min_counts
    return desired_cts


def _command_pwm(actuator: MaxonActuator, pwm: float) -> None:
    """Set motor direction from the sign of `pwm`, then apply its magnitude."""
    if pwm < 0.0:
        actuator.set_motor_direction_backward()  # toward lower %
    else:
        actuator.set_motor_direction_forward()  # toward higher %

    actuator.set_motor_pwm(np.abs(pwm))


def go_to_position(actuator: MaxonActuator, target_perc: float, position_logger: Logger) -> bool:
    """Command the motor to move the slider to `target_perc`.

    Returns True if it arrived within min_error before the time limit, False if it timed out.

    Direction convention assumes the encoder counts UP in the forward direction.
    """
    desired_cts = _clamp_target_cts(actuator, actuator.perc_to_cts(target_perc))
    desired_mm = actuator.cts_to_mm(desired_cts)

    start_time = time.time()
    last_time = start_time

    while True:
        try:
            actuator.update()
            actuator.check_coupler_drift()

            # TODO: replace with a real check_ankle_position() on the orthosis.
            ankle_in_range = True
            if not ankle_in_range:
                actuator.stop()
                position_logger.warning("Ankle out of range - aborting slider motion.")
                return False

            if np.abs(desired_mm - actuator.motor_position_mm) < actuator.min_error:
                actuator.stop()
                return True

            current_time = time.time()
            dt = current_time - last_time
            if dt <= 0.0:
                dt = 1e-6  # guard against divide-by-zero in the PID
            last_time = current_time

            if current_time - start_time > actuator.time_limit:
                position_logger.warning("Slider may be jammed - check prototype (PWM set to zero for safety).")
                actuator.stop()
                return False

            error_cts = int(desired_cts - actuator.motor_position_cts)
            pwm = actuator.pid_ctrl_position(error_cts, dt)

            if pwm == 0.0:
                actuator.stop()
                return True

            _command_pwm(actuator, pwm)
            time.sleep(DT)

        except KeyboardInterrupt:
            actuator.stop()
            position_logger.warning("KeyboardInterrupt during slider motion.")
            return False


def position_control():
    position_logger = Logger(
        log_path="./logs",
        file_name="position_control",
    )

    # initialize actuator and encodercounter
    actuator = MaxonActuator(
        frequency=FREQUENCY,
        offline=False,
    )

    encoder_counter = LS7366R()

    with actuator:
        actuator.set_motor_encoder(encoder_counter)

        actuator.position_control_config()
        actuator.set_control_mode(CONTROL_MODES.POSITION)
        actuator.position_control_init()

        if not actuator._is_homed:
            position_logger.info("Actuator homing to 0% stiffness.")
            actuator.home(home_zero=True)
            encoder_counter.clear_counter()
            actuator._is_homed = True

            actuator.update()
            actuator.check_coupler_drift()
            position_logger.info(f"Homing complete. position = {actuator.motor_position_mm:.3f} mm)")

        for target_perc in TARGET_PERC:
            arrived = go_to_position(actuator, target_perc, position_logger)
            actuator.update()
            position_logger.info(
                f"target {target_perc:.2f} % -> {'arrived' if arrived else 'TIMED OUT'} "
                f"at {actuator.motor_position_mm:.3f} mm"
            )
            time.sleep(1.0)


if __name__ == "__main__":
    position_control()
