import numpy as np

from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.logging.logger import Logger
from opensourceleg.time import SoftRealtimeLoop

TIME_TO_STEP = 1.0
FREQUENCY = 200
DT = 1 / FREQUENCY
GEAR_RATIO = 9 * (83 / 18)


def home_joint():
    homing_logger = Logger(
        log_path="./logs",
        file_name="homing_joint",
    )
    knee = DephyActuator(
        port="/dev/ttyACM0",
        tag="knee",
        gear_ratio=GEAR_RATIO,
        frequency=FREQUENCY,
        dephy_log=False,
    )
    ankle = DephyActuator(
        port="/dev/ttyACM1",
        tag="ankle",
        gear_ratio=GEAR_RATIO,
        frequency=FREQUENCY,
    )

    clock = SoftRealtimeLoop(dt=DT)

    with knee, ankle:
        knee.home(
            homing_voltage=2000,
            homing_frequency=FREQUENCY,
            homing_direction=-1,
            output_position_offset=0.0,
            current_threshold=5000,
            velocity_threshold=0.001,
        )

        ankle.home(
            homing_voltage=2000,
            homing_frequency=FREQUENCY,
            homing_direction=-1,
            output_position_offset=np.deg2rad(30.0),
            current_threshold=5000,
            velocity_threshold=0.001,
        )

        knee.set_control_mode(CONTROL_MODES.CURRENT)
        knee.set_current_gains()

        ankle.set_control_mode(CONTROL_MODES.CURRENT)
        ankle.set_current_gains()

        knee.set_motor_current(0)
        ankle.set_motor_current(0)

        # homing_logger.set_stream_terminator("\r")
        for t in clock:
            knee.update()
            ankle.update()

            homing_logger.info(
                # f"Time: {t:.2f}; Knee Position: {knee.motor_position:.2f}; "
                # f"Ankle Position: {ankle.motor_position:.2f}; "
                f"Time: {t:.2f}; Knee Output Position: {np.rad2deg(knee.output_position):.2f}; "
                f"Ankle Output Position: {np.rad2deg(ankle.output_position):.2f}; "
                f"Knee Winding Temp: {knee.winding_temperature:.2f}; "
                f"Ankle Winding Temp: {ankle.winding_temperature:.2f}; "
                # f"Knee Current: {knee.motor_current:.2f} mA; Ankle Current: {ankle.motor_current:.2f} mA;"
            )


if __name__ == "__main__":
    home_joint()
