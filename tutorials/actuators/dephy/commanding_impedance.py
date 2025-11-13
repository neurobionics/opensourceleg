import time

import numpy as np

from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.rust import Logger
from opensourceleg.utilities import SoftRealtimeLoop

TIME_TO_STEP = 1.0
FREQUENCY = 1000
DT = 1 / FREQUENCY
GEAR_RATIO = 41.5  # 1.0


def impedance_control():
    Logger.update_log_file_configuration(
        log_directory="./logs",
        log_name="impedance_control.log",
    )
    actpack = DephyActuator(
        port="/dev/ttyACM0",
        gear_ratio=GEAR_RATIO,
        frequency=FREQUENCY,
        debug_level=0,
        dephy_log=False,
    )
    clock = SoftRealtimeLoop(dt=DT)

    with actpack:
        actpack.update()
        actpack.set_control_mode(mode=CONTROL_MODES.IMPEDANCE)
        actpack.set_impedance_cc_pidf_gains()
        actpack.set_output_impedance()

        current_position = actpack.output_position
        command_position = current_position

        Logger.track_functions({
            "Output_Position": lambda: actpack.output_position,
            "Command_Position": lambda: command_position,
            "Motor_Current": lambda: actpack.motor_current,
            "Time": lambda: time.time(),
        })

        for t in clock:
            actpack.update()

            if t > TIME_TO_STEP:
                command_position = current_position + np.deg2rad(6)

            actpack.set_output_position(value=command_position)

            Logger.info(
                f"Time: {t}; "
                f"Command Position: {command_position}; "
                f"Output Position: {actpack.output_position}; "
                f"Motor Current: {actpack.motor_current}",
            )

            Logger.record()


if __name__ == "__main__":
    impedance_control()
