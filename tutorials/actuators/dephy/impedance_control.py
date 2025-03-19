import time

import numpy as np

from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.logging.logger import Logger
from opensourceleg.time import SoftRealtimeLoop

TIME_TO_STEP = 1.0
FREQUENCY = 1000
DT = 1 / FREQUENCY
GEAR_RATIO = 1.0


def impedance_control():
    impedance_logger = Logger(
        log_path="./logs",
        file_name="impedance_control",
    )
    actpack = DephyActuator(
        port="/dev/ttyACM0", gear_ratio=GEAR_RATIO, frequency=FREQUENCY, debug_level=0, dephy_log=False
    )
    clock = SoftRealtimeLoop(dt=DT)

    impedance_logger.set_stream_terminator("\r")

    with actpack:
        actpack.update()
        actpack.set_control_mode(mode=CONTROL_MODES.IMPEDANCE)
        actpack.set_impedance_gains()

        current_position = actpack.output_position
        command_position = current_position

        impedance_logger.track_function(lambda: actpack.output_position, "Output Position")
        impedance_logger.track_function(lambda: command_position, "Command Position")
        impedance_logger.track_function(lambda: actpack.motor_current, "Motor Current")
        impedance_logger.track_function(lambda: time.time(), "Time")

        for t in clock:
            actpack.update()

            if t > TIME_TO_STEP:
                command_position = current_position + np.pi / 2

            actpack.set_output_position(value=command_position)

            impedance_logger.info(
                f"Time: {t}; "
                f"Command Position: {command_position}; "
                f"Output Position: {actpack.output_position}; "
                f"Motor Current: {actpack.motor_current}",
            )

            impedance_logger.update()


if __name__ == "__main__":
    impedance_control()
