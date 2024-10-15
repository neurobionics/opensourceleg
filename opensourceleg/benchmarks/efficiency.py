from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.logging import LOGGER
from opensourceleg.time import SoftRealtimeLoop
from opensourceleg.actuators.base import CONTROL_MODES
from flexsea.device import Device



import numpy as np
import time

FREQUENCY = 500

if __name__ == "__main__":
    driving_motor = DephyActuator(port="/dev/ttyACM0", gear_ratio=9.0, frequency=FREQUENCY)
    clock = SoftRealtimeLoop(dt=1/FREQUENCY, report=True)

    with driving_motor:
        driving_motor.set_control_mode(CONTROL_MODES.VOLTAGE)
        driving_motor.set_motor_voltage(3000)


        for t in clock:
            driving_motor.update()
            driving_motor.set_motor_voltage(2000)
            LOGGER.info(f"{driving_motor.output_position}")








