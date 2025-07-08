import time

import numpy as np

from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.logging.logger import Logger
from opensourceleg.utilities import SoftRealtimeLoop

TIME_TO_STEP = 1.0
FREQUENCY = 1000
DT = 1 / FREQUENCY
GEAR_RATIO = 1.0


def position_control():
    position_logger = Logger(
        log_path="./logs",
        file_name="position_control",
    )
    actpack = DephyActuator(
        port="/dev/ttyACM0", gear_ratio=GEAR_RATIO, frequency=FREQUENCY, debug_level=0, dephy_log=False
    )
    clock = SoftRealtimeLoop(dt=DT)

    with actpack:
        actpack.set_control_mode(mode=CONTROL_MODES.POSITION)
        actpack.set_position_gains()

        actpack.update()
        current_position = actpack.output_position
        command_position = current_position

        position_logger.track_function(lambda: actpack.output_position, "Output Position")
        position_logger.track_function(lambda: command_position, "Command Position")
        position_logger.track_function(lambda: time.time(), "Time")

        for t in clock:
            if t > TIME_TO_STEP:
                command_position = current_position + (1 / 2) * np.pi
                actpack.set_output_position(value=command_position)

            actpack.update()

            position_logger.info(f"Time: {t}; \
                                 Command Position: {command_position}; \
                                 Output Position: {actpack.output_position}")
            position_logger.update()


if __name__ == "__main__":
    position_control()
