import numpy as np
import time
from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.logging.logger import Logger
from opensourceleg.time import SoftRealtimeLoop

TIME_TO_STEP = 1.0
FREQUENCY = 1000
DT = 1 / FREQUENCY

def impedance_control():
    impedance_logger = Logger(
        log_path="./logs",
        file_name="impedance_control",
    )
    actpack = DephyActuator(
        port="/dev/ttyACM0",
        gear_ratio=9.0,
        frequency=FREQUENCY,
        debug_level=0,
        dephy_log=False
    )
    clock = SoftRealtimeLoop(dt=DT)

    with actpack:
        actpack.set_control_mode(mode=CONTROL_MODES.IMPEDANCE)
        actpack.set_impedance_gains()

        actpack.update()
        current_position = actpack.output_position

        k = 150
        b = 600

        actpack.set_motor_impedance(
            k=k,
            b=b,
        )
        actpack.set_output_position(value=current_position + np.pi / 2)

        impedance_logger.track_variable(lambda: actpack.output_position, "Output Position")
        impedance_logger.track_variable(lambda: actpack.motor_position, "Motor Position")
        impedance_logger.track_variable(lambda: actpack.motor_current, "Motor Current")
        impedance_logger.track_variable(lambda: time.time(), "Time")

        for t in clock:
            actpack.update()
            # impedance_logger.info(f"Time: {t}; \
            #                         Command Position: {current_position + np.pi / 2}; \
            #                         Motor Position: {actpack.motor_position}; \
            #                         Motor Voltage: {actpack.motor_voltage}; \
            #                         Motor Current: {actpack.motor_current}")

            impedance_logger.update()

if __name__ == "__main__":
    impedance_control()