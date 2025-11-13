import numpy as np

from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.rust import Logger
from opensourceleg.utilities import SoftRealtimeLoop

FREQUENCY = 200
DT = 1 / FREQUENCY
GEAR_RATIO = 9 * (83 / 18)


def home_joint():
    Logger.update_log_file_configuration(
        log_directory="./logs",
        log_name="homing_joint.log",
    )
    actuator = DephyActuator(
        port="/dev/ttyACM0",
        tag="knee",
        gear_ratio=GEAR_RATIO,
        frequency=FREQUENCY,
        dephy_log=False,
    )

    clock = SoftRealtimeLoop(dt=DT)

    with actuator:
        actuator.home(
            homing_voltage=2000,
            homing_frequency=FREQUENCY,
            homing_direction=-1,
            output_position_offset=0.0,
            current_threshold=5000,
            velocity_threshold=0.001,
        )

        actuator.set_control_mode(CONTROL_MODES.CURRENT)
        actuator.set_current_gains()

        actuator.set_output_torque(0)

        # homing_logger.set_stream_terminator("\r")
        for t in clock:
            actuator.update()

            Logger.info(
                f"Time: {t:.2f}; Output Position: {np.rad2deg(actuator.output_position):.2f}; "
                f"Winding Temp: {actuator.winding_temperature:.2f}; "
                f"Current: {actuator.motor_current:.2f} mA;"
            )


if __name__ == "__main__":
    home_joint()
