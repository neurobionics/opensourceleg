import time

import numpy as np

import opensourceleg.actuators.dephy as Dephy
from opensourceleg.logging.logger import LOGGER

actpack = Dephy.DephyActuator(
    port="/dev/ttyACM0",
    gear_ratio=9 * (83 / 18),
)
with actpack:
    try:
        actpack.home()
        actpack.set_control_mode(mode=actpack.CONTROL_MODES.POSITION)
        while True:
            actpack.update()
            current_position = actpack.output_position
            actpack.set_position_gains(
                # if no input, then default gains are applied
            )
            actpack.set_output_position(value=current_position + np.pi / 8)
            LOGGER.info(
                "".join(
                    f"Motor Position: {actpack.output_position}\t"
                    + f"Motor Voltage: {actpack.motor_voltage}\t"
                    + f"Motor Current: {actpack.motor_current}\t"
                )
            )
            input("Enter to Continue")
            time.sleep(1)

    except KeyboardInterrupt:
        exit()
