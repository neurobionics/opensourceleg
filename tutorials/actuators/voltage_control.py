import opensourceleg.actuators.dephy as Dephy
from opensourceleg.logging.logger import LOGGER
import time

actpack = Dephy.DephyActpack(
    port="/dev/ttyACM0", 
    gear_ratio=9.0, 
)

try:
    actpack.start()
    actpack.set_control_mode(mode = actpack.CONTROL_MODES.VOLTAGE)
    while True: 
        actpack.set_motor_voltage(value = 3000)             # in mV
        actpack.update()
        LOGGER.info("".join(
              f"Motor Position: {actpack.motor_position}\t"
            + f"Motor Voltage: {actpack.motor_voltage}\t"
            + f"Motor Current: {actpack.motor_current}\t"
            )
        )
        time.sleep(0.1)

except KeyboardInterrupt:
    actpack.stop()