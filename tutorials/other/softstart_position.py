import numpy as np

from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.actuators.dephy import DEFAULT_POSITION_GAINS, DephyActuator
from opensourceleg.logging.logger import Logger
from opensourceleg.math.math import SaturatingRamp
from opensourceleg.utilities.softrealtimeloop import SoftRealtimeLoop

FREQUENCY = 1000
DT = 1 / FREQUENCY
GEAR_RATIO = 1.0
SOFT_START_TIME = 1.0


def softstart_position_control():
    position_logger = Logger(
        log_path="./logs",
        file_name="position_control",
    )
    actpack = DephyActuator(
        port="/dev/ttyACM0", gear_ratio=GEAR_RATIO, frequency=FREQUENCY, debug_level=0, dephy_log=False
    )
    clock = SoftRealtimeLoop(dt=DT)
    soft_start_ramp = SaturatingRamp(SOFT_START_TIME)

    with actpack:
        actpack.set_control_mode(mode=CONTROL_MODES.POSITION)

        actpack.update()
        start_position = actpack.output_position

        for t in clock:
            ss_scale = soft_start_ramp.update(t)
            actpack.set_position_gains(
                DEFAULT_POSITION_GAINS.kp * ss_scale,
                DEFAULT_POSITION_GAINS.kd * ss_scale,
                DEFAULT_POSITION_GAINS.ki * ss_scale,
            )
            command_position = start_position + (1 / 2) * np.pi
            actpack.set_output_position(value=command_position)

            actpack.update()

            position_logger.info(
                f"Time: {t}; \
                                 Command Position: {command_position}; \
                                 Output Position: {actpack.output_position}"
            )


if __name__ == "__main__":
    softstart_position_control()
