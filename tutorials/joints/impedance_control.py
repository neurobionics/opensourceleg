import opensourceleg.actuators.dephy as Dephy
from opensourceleg.actuators.base import ControlGains
from opensourceleg.logging.logger import LOGGER
import time
import numpy as np

actpack = Dephy.DephyActpack(
    port="/dev/ttyACM0", 
    gear_ratio= 9 * (83 / 18), 
)
with actpack:
    try:
        actpack.home()
        actpack.set_control_mode(mode = actpack.CONTROL_MODES.IMPEDANCE)
        actpack.update()
        k = 50
        b = 400
        current_position = actpack.output_position
        while True: 
            actpack.update()
            current_position = actpack.output_position
            k+=100
            actpack.set_impedance_gains(
                kp=40, 
                ki=400, 
                k=k, 
                b=b, 
                ff=128,
            )
            actpack.set_output_position(
                value = current_position + np.pi/8
            )

            LOGGER.info("".join(
                f"Motor Position: {actpack.motor_position}\t"
                + f"Motor Voltage: {actpack.motor_voltage}\t"
                + f"Motor Current: {actpack.motor_current}\t"
                )
            )
            input("Press Enter to continue...")
            time.sleep(0.2)

    except KeyboardInterrupt:
        exit()