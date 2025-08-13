import time

import numpy as np
from opensourceleg_rs import Logger

from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.utilities import SoftRealtimeLoop

TIME_TO_STEP = 1.0
FREQUENCY = 1000
DT = 1 / FREQUENCY
GEAR_RATIO = 1.0


def position_control():
    Logger.update_log_file_configuration(
        log_directory="./logs",
        log_name="position_control.log",
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

        Logger.track_functions({
            "Output_Position": lambda: actpack.output_position,
            "Command_Position": lambda: command_position,
            "Time": lambda: time.time(),
        })

        for t in clock:
            if t > TIME_TO_STEP:
                command_position = current_position + (1 / 2) * np.pi
                actpack.set_output_position(value=command_position)

            actpack.update()

            Logger.info(f"Time: {t}; \
                                 Command Position: {command_position}; \
                                 Output Position: {actpack.output_position}")
            Logger.record()


if __name__ == "__main__":
    position_control()
