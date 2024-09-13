import time

import opensourceleg.actuators.dephy as Dephy
from opensourceleg.logging.logger import LOGGER

actpack = Dephy.DephyActuator(
    port="/dev/ttyACM0",
    gear_ratio=1.0,
)
with actpack:
    try:
        while True:
            actpack.update()
            LOGGER.info(
                "".join(
                    f"Motor Position: {actpack.motor_position}\t"
                    + f"Motor Voltage: {actpack.motor_voltage}\t"
                    + f"Motor Current: {actpack.motor_current}\t"
                )
            )
            time.sleep(0.1)

    except KeyboardInterrupt:
        exit()
