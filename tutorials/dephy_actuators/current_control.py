import time

import opensourceleg.actuators.dephy as Dephy
from opensourceleg.logging.logger import LOGGER

actpack = Dephy.DephyActpack(
    port="/dev/ttyACM0",
    gear_ratio=1.0,
)
with actpack:
    try:
        actpack.set_control_mode(mode=actpack.CONTROL_MODES.CURRENT)
        actpack.set_current_gains(
            # if no input, then default gains are applied
        )
        actpack.set_motor_current(value=0)  # in mA
        actpack.update()
        LOGGER.info(
            "".join(
                f"Motor Position: {actpack.motor_position}\t"
                + f"Motor Voltage: {actpack.motor_voltage}\t"
                + f"Motor Current: {actpack.motor_current}\t"
            )
        )
        input("Press Enter to continue...")
        time.sleep(0.1)
        while True:
            actpack.set_motor_current(value=200)  # in mA
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
