from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.logging.logger import Logger
from opensourceleg.utilities import SoftRealtimeLoop

TIME_TO_STEP = 1.0
FREQUENCY = 1000
DT = 1 / FREQUENCY
GEAR_RATIO = 1.0

CURRENT_SETPOINT = 600  # mA


def current_control():
    current_logger = Logger(
        log_path="./logs",
        file_name="current_control",
    )
    actpack = DephyActuator(
        port="/dev/ttyACM0", gear_ratio=GEAR_RATIO, frequency=FREQUENCY, debug_level=0, dephy_log=False
    )
    clock = SoftRealtimeLoop(dt=DT)

    # current_logger.set_stream_terminator("\r")

    with actpack:
        actpack.set_control_mode(mode=CONTROL_MODES.CURRENT)
        actpack.set_current_gains()

        command_current = 0

        current_logger.track_function(lambda: actpack.motor_current, "Motor Current")
        current_logger.track_function(lambda: command_current, "Command Current")

        for t in clock:
            if t > TIME_TO_STEP:
                command_current = CURRENT_SETPOINT
                actpack.set_motor_current(value=command_current)  # in mA

            actpack.update()

            current_logger.info(
                f"Time: {t}; " f"Command Current: {command_current}; " f"Motor Current: {actpack.motor_current}",
            )
            current_logger.update()


if __name__ == "__main__":
    current_control()
