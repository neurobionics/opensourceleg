import time

import opensourceleg.actuators.dephy as Dephy
from opensourceleg.logging.logger import LOGGER

actpack = Dephy.DephyActuator(
    port="/dev/ttyACM0",
    gear_ratio=1.0,
)
with actpack:
    try:
        print("Case:", actpack.case_temperature)
        print("Winding:", actpack.winding_temperature)
        while True:
            print(actpack._data)
            actpack.update()
            LOGGER.info(
                "".join(
                    f"Motor Position: {actpack.motor_position}\t"
                    + f"Motor Voltage: {actpack.motor_voltage}\t"
                    + f"Motor Current: {actpack.motor_current}\t"
                    + f"Case Temperature: {actpack.case_temperature}"
                    + f"Winding Temperature: {actpack.winding_temperature}"
                )
            )
            time.sleep(0.005)

    except Exception:
        exit()
